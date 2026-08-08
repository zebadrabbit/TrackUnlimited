#include "TUCoasterRide.h"

#include "ProceduralMeshComponent.h"
#include "RenderCore.h"                 // SetNearClipPlaneGlobals
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"

#include "TrackSpline/TrackClose.h"
#include "TrackSpline/TrackValidate.h"

#include "Camera/CameraComponent.h"
#include "CanvasItem.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Debug/DebugDrawService.h"
#include "DrawDebugHelpers.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "UObject/ConstructorHelpers.h"

// The ride's own event stream. Its own category so it can be filtered on its
// own — with transitions on it is the loudest thing in the log by a wide margin,
// and mixed into LogTemp it would bury everything else.
DEFINE_LOG_CATEGORY(LogTUEvents);

// The authored side -> the model's side. One place, so the two enums cannot
// drift into disagreeing about what Left means.
static EWalkway ToWalkwaySide(ETUWalkway In)
{
	switch (In)
	{
	case ETUWalkway::Left:  return EWalkway::Left;
	case ETUWalkway::Right: return EWalkway::Right;
	case ETUWalkway::Both:  return EWalkway::Both;
	default:                return EWalkway::None;
	}
}


namespace
{
	// The prototypes work in metres; Unreal works in centimetres.
	constexpr double MetresToUU = 100.0;

	constexpr double Pi = 3.14159265358979323846;
	double Deg(double D) { return D * Pi / 180.0; }
}

ATUCoasterRide::ATUCoasterRide()
{
	PrimaryActorTick.bCanEverTick = true;
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	Cars = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("Cars"));
	Cars->SetupAttachment(Root);
	Cars->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Cars->SetCastShadow(false);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Cars->SetStaticMesh(CubeMesh.Object);
	}

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Root);

	// PHASE 4. Three components rather than one: rails, spine and ties are three
	// materials — running rail is polished where the wheels touch it, spine is
	// painted structure, ties are neither — and a track style may want to replace
	// one of them without touching the others.
	//
	// COLLISION OFF on all three. A quarter of a million triangles of collision
	// geometry costs a great deal and buys nothing: the train is constrained to
	// the spline analytically and does not collide with anything, which is the
	// whole reason FTrain is 1D rather than a rigid body.
	auto MakeTrackMesh = [this](const TCHAR* Name)
	{
		UProceduralMeshComponent* C = CreateDefaultSubobject<UProceduralMeshComponent>(Name);
		C->SetupAttachment(Root);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->bUseAsyncCooking = true;
		return C;
	};
	RailMesh = MakeTrackMesh(TEXT("RailMesh"));
	SpineMesh = MakeTrackMesh(TEXT("SpineMesh"));
	TieMesh = MakeTrackMesh(TEXT("TieMesh"));

	// Seeded so a freshly placed actor has a ride in it. Everything about that
	// ride is data in the Details panel rather than code, which is the whole
	// point — Preset + bLoadPreset puts a known-good one back if an edit goes
	// wrong, or swaps in a different worked example to take apart.
	Segments = PresetLayout(Preset);

	// AND THE SESSION STARTS IN BUILD, not in Boot.
	//
	// This matters more than it looks: EditsAllowed() is false everywhere except
	// Build, and PostEditChangeProperty now refuses on it — so a session left in
	// Boot would silently reject every number typed into the Details panel, which
	// is the whole editing surface today.
	//
	// Boot exists to discover a crash sidecar before anything can overwrite it,
	// and an actor placed in a level has already skipped that: the level IS the
	// document. The menu is for the packaged shell, which starts somewhere else.
	Session.Enter(EAppMode::MainMenu);
	Session.Enter(EAppMode::Build);
}

// The drawing palette and its two primitives, HOISTED ABOVE EVERY PANEL THAT
// USES THEM. There are four now — the mode banner, the profile graph, the
// diagnostics list and the control panel — and a palette declared beside the
// last one only compiles for the last one.
//
// Shared deliberately rather than duplicated: the panels are meant to look like
// one instrument seen from different angles, and two copies of a colour drift
// the first time somebody adjusts one.
namespace
{
	// The drawing's own palette, so the panel and the splash are the same object
	// seen twice. Near-black ground, amber for what is working, cyan for what is
	// measured, red for what has stopped.
	const FLinearColor PanelGround(0.043f, 0.055f, 0.067f, 0.92f);
	const FLinearColor PanelRule(0.20f, 0.24f, 0.28f, 1.f);
	const FLinearColor PanelText(0.72f, 0.78f, 0.82f, 1.f);
	const FLinearColor PanelDim(0.42f, 0.47f, 0.52f, 1.f);
	const FLinearColor PanelAmber(0.98f, 0.62f, 0.16f, 1.f);
	const FLinearColor PanelCyan(0.35f, 0.74f, 1.00f, 1.f);
	const FLinearColor PanelGreen(0.35f, 0.82f, 0.45f, 1.f);
	const FLinearColor PanelRed(0.95f, 0.28f, 0.24f, 1.f);

	void PanelTile(UCanvas* C, float X, float Y, float W, float H, const FLinearColor& Col)
	{
		FCanvasTileItem Tile(FVector2D(X, Y), FVector2D(W, H), Col);
		Tile.BlendMode = SE_BLEND_Translucent;
		C->DrawItem(Tile);
	}

	void PanelLabel(UCanvas* C, float X, float Y, const FString& S, const FLinearColor& Col)
	{
		FCanvasTextItem Item(FVector2D(X, Y), FText::FromString(S), GEngine->GetSmallFont(), Col);
		Item.EnableShadow(FLinearColor::Black);
		C->DrawItem(Item);
	}

	// What a zone IS, for the module heading. FTrackZone drops the kind because the
	// physics does not care; the panel is the one place that has to say it.
	const TCHAR* ZoneKindName(ETUSegmentZone Kind)
	{
		switch (Kind)
		{
		case ETUSegmentZone::Lift:          return TEXT("LIFT");
		case ETUSegmentZone::Launch:        return TEXT("LAUNCH");
		case ETUSegmentZone::Brake:         return TEXT("TRIM");
		case ETUSegmentZone::BlockBrake:    return TEXT("BLOCK BRAKE");
		case ETUSegmentZone::Station:       return TEXT("STATION");
		case ETUSegmentZone::StationUnload: return TEXT("UNLOAD");
		case ETUSegmentZone::StationLoad:   return TEXT("LOAD");
		default:                            return TEXT("-");
		}
	}

	const TCHAR* PhaseName(EStationPhase P)
	{
		switch (P)
		{
		case EStationPhase::Empty:     return TEXT("EMPTY");
		case EStationPhase::Arriving:  return TEXT("ARRIVING");
		case EStationPhase::Unloading: return TEXT("UNLOADING");
		case EStationPhase::Loading:   return TEXT("LOADING");
		case EStationPhase::Securing:  return TEXT("SECURING");
		case EStationPhase::Ready:     return TEXT("READY");
		default:                       return TEXT("DEPARTING");
		}
	}
}

TArray<FTUTrackSegment> ATUCoasterRide::PresetLayout(ETUPresetLayout Which)
{
	switch (Which)
	{
	case ETUPresetLayout::FlatRig:         return FlatRigLayout();
	case ETUPresetLayout::OutAndBack:      return OutAndBackLayout();
	case ETUPresetLayout::TwoTrainCircuit: return TwoTrainCircuitLayout();
	case ETUPresetLayout::SmallBatch:      return SmallBatchLayout();
	default:                               return ReferenceLayout();
	}
}

void ATUCoasterRide::ApplyPresetTrainSetup(ETUPresetLayout Which)
{
	// A PRESET IS A WORKED EXAMPLE, AND THE TRAIN IS PART OF THE EXAMPLE. A
	// small-batch operation has small vehicles, and its platform positions are
	// sized for them — 10 m each, which fits a 6 m train and its clearance and
	// nothing longer. Loading that layout with the 15 m train every other preset
	// uses puts the stop mark PAST the end of every position, where nothing trips
	// it, and a train sent to park there crawls out of its block into the next
	// one. Measured: seven signalling violations inside four seconds.
	//
	// So the train comes with the layout. Only on an explicit preset load, never
	// on a rebuild — an author who has since chosen a different train keeps it.
	switch (Which)
	{
	case ETUPresetLayout::SmallBatch:
		TrainLengthM = 6.f;
		TrainCount = 5;
		break;
	case ETUPresetLayout::TwoTrainCircuit:
		TrainLengthM = 15.f;
		TrainCount = 2;
		break;
	default:
		TrainLengthM = 15.f;
		TrainCount = 1;   // every other preset has one holding place
		break;
	}
}

// Shared shapes for the layouts below. ReferenceLayout keeps its own copies
// deliberately: it is the fixture every published figure is measured from, and
// re-pointing it at new helpers would mean re-verifying all of them to save a
// dozen lines.
namespace
{

// Returns nothing on purpose. Handing back a reference into a TArray invites
// somebody to hold it across the next Add, which reallocates.
void AddStraight(TArray<FTUTrackSegment>& Out, double Length,
	ETUSegmentZone Zone = ETUSegmentZone::None, float ZoneSpeed = 0.f)
{
	FTUTrackSegment S;
	S.Kind = ETUSegmentKind::Straight;
	S.Length = static_cast<float>(Length);
	S.Zone = Zone;
	S.ZoneSpeed = ZoneSpeed;
	Out.Add(S);
}

// A vertical curve with no curvature step at either end: pitch curvature ramps
// 0 -> peak -> 0, so both joints stay continuous. Two Raw segments, because the
// authored vocabulary is still yaw-only and no Make* helper builds pitch.
void AddEasedPitch(TArray<FTUTrackSegment>& Out, double PitchDelta, double PeakCurvature,
	ETUSegmentZone Zone = ETUSegmentZone::None, float ZoneSpeed = 0.f)
{
	const double K = PitchDelta >= 0.0 ? PeakCurvature : -PeakCurvature;
	const double L = FMath::Abs(PitchDelta) / PeakCurvature;

	FTUTrackSegment In;
	In.Kind = ETUSegmentKind::Raw;
	In.Length = static_cast<float>(L);
	In.PitchCurvatureEnd = static_cast<float>(K);
	In.Zone = Zone;
	In.ZoneSpeed = ZoneSpeed;
	Out.Add(In);

	FTUTrackSegment Tail;
	Tail.Kind = ETUSegmentKind::Raw;
	Tail.Length = static_cast<float>(L);
	Tail.PitchCurvatureStart = static_cast<float>(K);
	Tail.Zone = Zone;
	Tail.ZoneSpeed = ZoneSpeed;
	Out.Add(Tail);
}

// Clothoid in, constant-radius hold, clothoid out, with roll ramping across the
// clothoids so neither curvature nor roll steps at a joint.
//
// Roll RATE still steps, and measurably: roll varies LINEARLY across a segment,
// so its rate is constant within one and changes abruptly at the ends. Every
// banked turn in this vocabulary has that, the reference layout included — the
// validator reports four such steps on it. Removing them needs roll rate itself
// to ease, which the authored vocabulary cannot currently say.
void AddBankedTurn(TArray<FTUTrackSegment>& Out, double Radius, double ArcLength,
	double EaseLength, double BankDegrees)
{
	FTUTrackSegment In;
	In.Kind = ETUSegmentKind::Clothoid;
	In.Length = static_cast<float>(EaseLength);
	In.CurvatureStart = 0.f;
	In.CurvatureEnd = static_cast<float>(1.0 / Radius);
	In.RollEndDegrees = static_cast<float>(BankDegrees);
	Out.Add(In);

	FTUTrackSegment Hold;
	Hold.Kind = ETUSegmentKind::Arc;
	Hold.Length = static_cast<float>(ArcLength);
	Hold.Radius = static_cast<float>(Radius);
	Hold.RollStartDegrees = Hold.RollEndDegrees = static_cast<float>(BankDegrees);
	Out.Add(Hold);

	FTUTrackSegment OutEase;
	OutEase.Kind = ETUSegmentKind::Clothoid;
	OutEase.Length = static_cast<float>(EaseLength);
	OutEase.CurvatureStart = static_cast<float>(1.0 / Radius);
	OutEase.CurvatureEnd = 0.f;
	OutEase.RollStartDegrees = static_cast<float>(BankDegrees);
	Out.Add(OutEase);
}

// The bank that cancels lateral G at a given speed on a given radius.
double BankDegreesFor(double SpeedMs, double Radius)
{
	return FMath::RadiansToDegrees(
		FMath::Atan((SpeedMs * SpeedMs) / (GravityMs2 * Radius)));
}

} // namespace

TArray<FTUTrackSegment> ATUCoasterRide::FlatRigLayout()
{
	// Three straights. Dead level, so gravity contributes nothing and what the
	// train does is purely launch, rolling resistance and drag — which is exactly
	// why this shape was the rig the RollingResistance correction was measured
	// on against NoLimits 2.
	//
	// Measured: 160 m, ends at station height, C2, 30.0 km/h, a flat +1.00 g
	// throughout, no validation issues at all.
	TArray<FTUTrackSegment> Out;
	AddStraight(Out, 20.0, ETUSegmentZone::Launch, 8.333f);  // 30 km/h
	AddStraight(Out, 90.0);
	AddStraight(Out, 50.0, ETUSegmentZone::Brake, 0.f);
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::OutAndBackLayout()
{
	// Measured: 20 segments, 631.4 m, ends within a millimetre of station height,
	// C2 to 1e-9, 96.9 km/h, vertical -0.10..+2.04 g, lateral 0.34, closest
	// self-approach 7.77 m. Gentler than the reference layout and it does not
	// pass through itself, which the reference does at 0.19 m.
	const double Up = Deg(25.0);
	const double Drop = Deg(-32.0);

	// Solved, not eyeballed: this is the drop that brings the ride back to
	// station height at the end. 18 m left it 4.64 m in the air.
	const double DropLength = 26.758;

	TArray<FTUTrackSegment> Out;
	AddStraight(Out, 20.0, ETUSegmentZone::Station, 4.f);      // station
	AddEasedPitch(Out, Up, 0.03, ETUSegmentZone::Lift, 4.f);   // into the climb
	AddStraight(Out, 62.0, ETUSegmentZone::Lift, 4.f);         // lift climb

	// Only the FIRST of the crest pair is powered, for the reason spelled out in
	// ReferenceLayout: the chain must run over the top and let go just past it.
	const int32 CrestFirst = Out.Num();
	AddEasedPitch(Out, Drop - Up, 0.05);
	Out[CrestFirst].Zone = ETUSegmentZone::Lift;
	Out[CrestFirst].ZoneSpeed = 4.f;

	AddStraight(Out, DropLength);
	AddEasedPitch(Out, -Drop, 0.015);                          // pull-out to level

	// The airtime hill: a lazy 14 degree rise into a sharp 30 degree fall, which
	// puts the whole ride's minimum vertical G here at -0.10 g.
	//
	// MEASURED, because the obvious claim about it turned out to be false: the
	// front car sees -0.14 g and the back -0.12, so on THIS hill the front gets
	// marginally the better airtime, not the back. Asymmetry alone does not buy
	// the back-row effect — PHASE0_FINDINGS records a crest where it is worth
	// 0.73 g, so the shape matters and this one is not that shape. Left as it is
	// because the ride is good; treat tuning it into a back-row hill as an open
	// exercise rather than a fix.
	//
	// The largest front-to-back spread on this layout is 0.55 g and it is in the
	// banked turn, not here — the cars are on different curvature there, which is
	// a different mechanism entirely.
	AddEasedPitch(Out, Deg(14.0), 0.006);
	AddEasedPitch(Out, Deg(-30.0), 0.030);
	AddEasedPitch(Out, Deg(16.0), 0.015);                      // back to level

	// 180 degree turnaround, banked for the speed it is actually taken at.
	AddBankedTurn(Out, 26.0, Pi * 26.0, 22.0, BankDegreesFor(20.0, 26.0));

	AddStraight(Out, 40.0);
	AddStraight(Out, 60.0, ETUSegmentZone::Brake, 0.f);        // brake run
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::TwoTrainCircuitLayout()
{
	// The only preset with enough blocks to run more than one train.
	//
	// Blocks are DERIVED: a boundary falls where a powered run starts or ends,
	// because that is the only place a device exists that can hold a train.
	//
	// CAPACITY IS NOT blocks/(1 + lookahead), which an earlier version of this
	// comment claimed and which is neither necessary nor sufficient — it allows
	// four trains at lookahead 1, where they collided, and forbids three at
	// lookahead 2, which run. The rule is
	//
	//     trains = (blocks that can STOP a train and LET IT GO) - 1
	//
	// and the minus one is the whole thing: one has to stay empty or every train
	// is standing exactly where the train behind it needs to go and the ring
	// cannot rotate. Asserted for rings of 3 to 8 in test_ridesignals.cpp, which
	// drives it on bare numbers because capacity is a signalling property with
	// nothing to do with geometry.
	//
	// It is also why a high-throughput ride is built with MORE block sections
	// rather than a longer train: sections buy trains, and trains buy capacity.
	// This preset has five and runs four; every other one has a station and a trim
	// brake — and a trim cannot let a train go again — so they have ONE, and one
	// place is one train, always.
	//
	// Do NOT read a real ride's brake count as its block count. Three different
	// things sit on the track and all look like a brake: BLOCK BRAKES, which are
	// these; EVACUATION ZONES, which are about getting riders off and need a
	// walkway rather than just a way to stop; and SAFETY CATCHES for rollback or a
	// defect. A large ride can have ~25 of the second and a handful of the third
	// while running far fewer trains than either count. Only block brakes feed the
	// formula, and neither of the other two is modelled at all — see SIGNALLING.md.
	//
	// The approach is TWO brake blocks with drive tyres between, not one long
	// brake, and both halves of that matter. Two adjacent Brake runs MERGE into a
	// single block, so "one long brake" holds exactly one train — and worse, the
	// second run's authored ZoneSpeed is silently discarded (measured: 6 m/s then
	// 2 m/s becomes one zone at 6). And a friction brake can hold a train but
	// cannot start one, also measured, so moving a stopped train from the outer
	// brake to the inner one needs powered track either way.
	//
	// ---- IT IS A CLOSED CIRCUIT, AND THE SHAPE IS WHY ----
	//
	// An OVAL closes analytically, with no solver: both turns the same way, each
	// EXACTLY 180 degrees, same radius, same easement. Then the two lateral
	// displacements (+2R, -2R) cancel, the along-track ones cancel, heading sums to
	// 360, and one scalar condition is left — the return leg's horizontal extent
	// must equal the two collinear outbound legs'. A 180 degree turn with easements
	// of length E needs arc = pi*R - E, because each easement turns E/(2R).
	//
	// The turnaround sits at the TOP of the hill, and that is not decoration. The
	// closure condition means every metre in the outbound leg is paid for TWICE, so
	// the first attempt — whole hill outbound, turnarounds at each end — came to
	// 1717 m and every variant of it STALLED, because softening the crest to tame
	// the G lengthens it (crest length is angle/curvature), which lengthens the
	// circuit, which costs more energy than a longer launch buys. Splitting the
	// hill across the turn is cheaper in three currencies at once: the train is
	// slowest at the top so the turn costs least lateral G, a small radius will do,
	// and the drop's horizontal extent lands in the return leg where it is needed.
	//
	// MEASURED, every figure re-derived through the authored path: 30 segments,
	// 1288.00 m, 8 blocks, C2 to 1e-9, closest self-approach 11.68 m, top
	// 136.8 km/h, vertical -0.53..+3.08 g, lateral 0.15 g, crest 48.5 m, 105 s,
	// peak roll rate 17.4 deg/s, ZERO curvature-step and ZERO roll-rate
	// diagnostics. The seam closes to 0.000000 m, 0.000084 degrees of heading and
	// 0.000000 degrees of roll.
	//
	// Those figures are the ride with every signal GREEN, which is what the ride
	// profile measures and the right thing for it to measure. Held at a red the
	// train stops wherever the block brake is, and none of the numbers above move
	// — a holding device only ever removes energy the layout had already spent.
	const double Up = Deg(26.0);       // pull-up out of the launch
	const double Dn = Deg(32.0);       // the drop out of the turnaround

	// The turn. Both are this, exactly, or the circuit does not close.
	const double R = 35.0;
	const double Ease = 50.0;          // 50, not 26: roll rate 34.0 -> 17.4 deg/s
	const double Arc = Pi * R - Ease;  // exactly 180 degrees, easements included

	// SOLVED, not eyeballed. The drop straight is the height lever — more of it
	// ends lower, monotonically — and the fill is then the horizontal one, which
	// cannot disturb the height because it is level.
	const double DropLen = 15.6847323;
	const double FillLen = 75.5024975;

	TArray<FTUTrackSegment> Out;

	// ---- LEG A, outbound. Station, launch, and the climb to the turnaround.
	AddStraight(Out, 26.0, ETUSegmentZone::Station, 1.5f);    // 1 STATION, drive tyres
	AddStraight(Out, 150.0, ETUSegmentZone::Launch, 38.f);    // 2 LAUNCH

	// Launch length matters more than the target: at the fixed 6 m/s^2 grip a
	// launch caps at sqrt(2*grip*length) whatever is asked for, so 70 m could
	// never exceed 29 m/s and three different targets all produced the same top
	// speed. 150 m is what makes 38 m/s actually reachable.
	AddEasedPitch(Out, Up, 0.0130);    // the +Gz peak lives HERE — highest v^2
	AddStraight(Out, 40.0);            // curvature on the track. 0.0130 holds it
	AddEasedPitch(Out, -Up, 0.0130);   // to +3.08 g. Levels off 48.5 m up.

	// ANTI-ROLLBACK over the launch and the whole climb, which is where a real
	// launched coaster puts it: a train that fails to make the top comes back down
	// this stretch and arrives in the station at speed, backwards.
	//
	// It changes NOTHING while the ride works — the train crests at 12.14 m/s with
	// margin — and that is exactly the property a safety device should have. Detune
	// the launch and it is the difference between a held train and a loose one.
	for (int32 i = 1; i < Out.Num(); ++i)
	{
		Out[i].bAntiRollback = true;
	}

	// ---- TURN 1, level, at the top, taken at 12.1 m/s. Being slow here is the
	// whole point of putting it at the top: a turnaround at launch speed would
	// need R ~ 119 m to stay under 2 g, and that radius doubles into the circuit.
	AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(14.2, R));

	// ---- LEG B, the return. Drop, airtime, then the mid-course block brake.
	AddEasedPitch(Out, -Dn, 0.0150);
	AddStraight(Out, DropLen);
	AddEasedPitch(Out, Dn, 0.0150);

	// One airtime hill, which is where the -0.53 g comes from. Symmetric, so it
	// returns to the height and the pitch it started at: up, over, level out.
	AddEasedPitch(Out, Deg(20.0), 0.024);
	AddEasedPitch(Out, Deg(-40.0), 0.024);
	AddEasedPitch(Out, Deg(20.0), 0.024);

	AddStraight(Out, FillLen);

	// 4 MID-COURSE BLOCK BRAKE, and closing the circuit is what made it one. It
	// takes the train at 26.40 m/s and stopping that at 6 m/s^2 needs 58.1 m — so
	// the 45 m it used to have could only ever trim, and 130 m can HOLD. That is a
	// third queueing position, on the far side of the circuit from the station.
	AddStraight(Out, 130.0, ETUSegmentZone::BlockBrake, 20.f);

	// ---- TURN 2, level, at station height, taken at 18.1 m/s.
	AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(18.1, R));

	// ---- LEG C, the approach. Collinear with leg A and ending exactly where the
	// station begins, which is what closes the circuit.
	AddStraight(Out, 24.0);

	// 6/7/8 THE APPROACH. The outer brake is where a second train waits while the
	// first is in the station; the inner one holds short of the station and is
	// what clears the station-entry signal once its train has stopped.
	//
	// All three are hold-capable, which is what makes this preset run two trains:
	// blocks 5, 6 and 7 can each stop a train AND let it go again, so a queue can
	// form outside the station instead of on the course. Measured arrival speeds
	// are 15.50, 5.77 and 2.80 m/s, needing 20.0, 2.8 and 0.7 m to stop — every one
	// inside its own block, and now so is the mid-course brake.
	//
	// The authored speeds are the RELEASE speeds. A holding device rests closed
	// and is commanded to these only while its permissive is granted, which is
	// why the two brakes may sensibly trim to 6 and 2 m/s and still come to a
	// dead stop when the signal is red.
	AddStraight(Out, 37.5, ETUSegmentZone::BlockBrake, 6.f);  // outer
	AddStraight(Out, 27.0, ETUSegmentZone::Lift, 4.f);        // transfer tyres
	AddStraight(Out, 37.5, ETUSegmentZone::BlockBrake, 2.f);  // inner
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::SmallBatchLayout()
{
	// THE SAME OVAL, RE-ZONED. Derived from TwoTrainCircuitLayout rather than
	// copied, because the closure is a property of the LEG LENGTHS and a copy
	// could drift from the shape it depends on with nothing to notice — the seam
	// would miss by a metre, look fine from the cockpit, and teleport the train
	// once a lap.
	//
	// Leg A opens with a 26 m station and a 150 m launch: 176 m of flat. Replace
	// that with 40 m of platform and a 136 m launch and it is the same 176 m, so
	// the circuit still closes to 0.000000 m and every G figure still holds. The
	// launch reaches its 38 m/s in 120 m either way.
	//
	// Verified in Prototypes/TrainPhysics/test_twotrains.cpp against the two-train
	// layout: identical total length, C2, seam within 1e-3 m, and it still gets
	// round on a 6 m train — which a shorter train is not guaranteed to do, since
	// less of it straddles each crest and it pays more of the height.
	TArray<FTUTrackSegment> Out = TwoTrainCircuitLayout();
	Out.RemoveAt(0, 2);

	// 10 m positions: a 6 m train plus its 1 m nose clearance, with room to spare.
	// Any shorter and the stop mark falls past the end of the device, where no
	// train can ever trip it — which the build now warns about.
	TArray<FTUTrackSegment> Head;
	AddStraight(Head, 10.0, ETUSegmentZone::StationUnload, 1.5f);  // riders off
	AddStraight(Head, 10.0, ETUSegmentZone::StationLoad, 1.5f);    // position 3, rear
	AddStraight(Head, 10.0, ETUSegmentZone::StationLoad, 1.5f);    // position 2
	AddStraight(Head, 10.0, ETUSegmentZone::StationLoad, 1.5f);    // position 1, front
	AddStraight(Head, 136.0, ETUSegmentZone::Launch, 38.f);

	// Identical kind AND identical speed, so nothing the zone walk can see tells
	// these three apart — they really are the same machine three times over, and
	// are still three machines. Without this they are one 30 m zone holding one
	// train, which is a platform that loads one at a time.
	Head[2].bStartsNewDevice = true;
	Head[3].bStartsNewDevice = true;

	Out.Insert(Head, 0);
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::ReferenceLayout()
{
	// The reference ride, as authored data rather than as code. Tuned in the
	// standalone harness first; these numbers give a 50.1 m crest, 105.2 km/h,
	// +0.66..+4.19 vertical G, 0.28 peak lateral and +1.13 G over the loop apex,
	// with the track ending at station height to within a millimetre.
	//
	// This used to be a sequence of AddSegment calls, which meant the layout was
	// only editable by recompiling. It is a list now because that is what the
	// project said track authoring should be from the start.
	const double Lift = Deg(25.0);
	const double Drop = Deg(-34.0);
	const double LoopRadius = 9.0;
	const double LoopEase = 54.0;
	const double TurnRadius = 32.0;
	const double BankDeg = FMath::RadiansToDegrees(
		FMath::Atan((26.5 * 26.5) / (GravityMs2 * TurnRadius)));

	// Both solved rather than eyeballed, and they do different jobs.
	//
	// The DROP sets the ride. Everything the train has at the loop comes from
	// the descent below the crest, so this is the only lever that changes how it
	// feels. 12.0 m was tuned when RollingResistance was 0.006 — a figure
	// justified against steel-on-steel, which is a railway rather than a coaster.
	// At the corrected 0.024 for polyurethane wheels, 12 m leaves the train
	// cresting the loop at +0.13 g: hanging on the track rather than held to it.
	// 24 m restores +1.13 g and the original G profile almost exactly.
	//
	// The LIFT sets the height and nothing else. Raising it changes no G number
	// on this layout at all — measured, across 75 to 115 m — because the chain
	// delivers the train to the crest at 4 m/s whatever height the crest is. So
	// it is free to use purely to close the ride back to station level, which is
	// what 90.99 does: deepening the drop by 12 m put the ending 6.710 m low.
	const double LiftClimb = 90.99;
	const double DropLength = 24.0;

	TArray<FTUTrackSegment> Out;

	auto Straight = [&Out](double Length, ETUSegmentZone Zone = ETUSegmentZone::None,
		float ZoneSpeed = 0.f) -> FTUTrackSegment&
	{
		FTUTrackSegment S;
		S.Kind = ETUSegmentKind::Straight;
		S.Length = static_cast<float>(Length);
		S.Zone = Zone;
		S.ZoneSpeed = ZoneSpeed;
		return Out[Out.Add(S)];
	};

	// A vertical curve with no curvature step at either end: pitch curvature
	// ramps 0 -> peak -> 0, so both joints stay continuous. Two Raw segments,
	// because no Make* helper builds pitch curvature — the authored vocabulary
	// is still yaw-only, which is exactly what ESegmentKind::Raw records.
	auto EasedPitch = [&Out](double PitchDelta, double PeakCurvature,
		ETUSegmentZone Zone = ETUSegmentZone::None, float ZoneSpeed = 0.f)
	{
		const double K = PitchDelta >= 0.0 ? PeakCurvature : -PeakCurvature;
		const double L = FMath::Abs(PitchDelta) / PeakCurvature;

		FTUTrackSegment In;
		In.Kind = ETUSegmentKind::Raw;
		In.Length = static_cast<float>(L);
		In.PitchCurvatureEnd = static_cast<float>(K);
		In.Zone = Zone;
		In.ZoneSpeed = ZoneSpeed;
		Out.Add(In);

		FTUTrackSegment Tail;
		Tail.Kind = ETUSegmentKind::Raw;
		Tail.Length = static_cast<float>(L);
		Tail.PitchCurvatureStart = static_cast<float>(K);
		Tail.Zone = Zone;
		Tail.ZoneSpeed = ZoneSpeed;
		Out.Add(Tail);
	};

	Straight(20.0, ETUSegmentZone::Station, 4.f);      // station
	EasedPitch(Lift, 0.03, ETUSegmentZone::Lift, 4.f); // into the climb
	Straight(LiftClimb, ETUSegmentZone::Lift, 4.f);    // lift climb

	// The chain has to run OVER the crest and stop AT it, and the window is
	// narrower than it looks in both directions.
	//
	// Release at the top of the climb and the train is stranded: the straight
	// tops out while the track is still rising. Carry on down the far side and
	// it is worse — MakeLift holds a fast train back as well as pulling a slow
	// one, so the chain sits on the train at 4 m/s down a 34-degree drop and
	// takes the ride's energy with it. Measured: powering both halves of the
	// crest instead of the first costs 10 km/h of top speed and flattens the
	// loop apex from +1.34 g to nothing.
	//
	// So only the FIRST of the pair is powered — the segment carrying the track
	// from +25 degrees down through level. Pitch crosses zero 17.4 m into a
	// 20.6 m segment, so the chain lets go just past the top.
	const int32 CrestFirst = Out.Num();
	EasedPitch(Drop - Lift, 0.05);
	Out[CrestFirst].Zone = ETUSegmentZone::Lift;
	Out[CrestFirst].ZoneSpeed = 4.f;

	Straight(DropLength);                              // drop
	EasedPitch(-Drop, 0.012);                             // pull-out

	// Teardrop loop: curvature eases in and out rather than stepping, so the
	// radius is large where the train is fastest. A circular loop at this speed
	// would pull over 9 G at the bottom, which is why real loops are not circles.
	//
	// Known defect, measured and left alone deliberately: being planar, its two
	// legs pass 0.19 m from each other. See PHASE0_FINDINGS.md — every cheap fix
	// was tried and costs more than the defect does.
	{
		FTUTrackSegment EaseIn;
		EaseIn.Kind = ETUSegmentKind::Raw;
		EaseIn.Length = static_cast<float>(LoopEase);
		EaseIn.PitchCurvatureEnd = static_cast<float>(1.0 / LoopRadius);
		Out.Add(EaseIn);

		FTUTrackSegment Crown;
		Crown.Kind = ETUSegmentKind::Raw;
		Crown.Length = static_cast<float>(2.0 * Pi * LoopRadius - LoopEase);
		Crown.PitchCurvatureStart = static_cast<float>(1.0 / LoopRadius);
		Crown.PitchCurvatureEnd = static_cast<float>(1.0 / LoopRadius);
		Out.Add(Crown);

		FTUTrackSegment EaseOut;
		EaseOut.Kind = ETUSegmentKind::Raw;
		EaseOut.Length = static_cast<float>(LoopEase);
		EaseOut.PitchCurvatureStart = static_cast<float>(1.0 / LoopRadius);
		Out.Add(EaseOut);
	}

	// Banked turn, clothoid in and out so neither curvature nor roll steps.
	// Path-relative, matching what this layout has always been: it follows an
	// inversion, so the frame arrives carrying whatever twist the loop left it,
	// and re-reading these numbers as bank-from-horizon would change the ride.
	{
		FTUTrackSegment In;
		In.Kind = ETUSegmentKind::Clothoid;
		In.Length = 26.f;
		In.CurvatureStart = 0.f;
		In.CurvatureEnd = static_cast<float>(1.0 / TurnRadius);
		In.RollEndDegrees = static_cast<float>(BankDeg);
		Out.Add(In);

		FTUTrackSegment Hold;
		Hold.Kind = ETUSegmentKind::Arc;
		Hold.Length = 55.f;
		Hold.Radius = static_cast<float>(TurnRadius);
		Hold.RollStartDegrees = Hold.RollEndDegrees = static_cast<float>(BankDeg);
		Out.Add(Hold);

		FTUTrackSegment OutEase;
		OutEase.Kind = ETUSegmentKind::Clothoid;
		OutEase.Length = 26.f;
		OutEase.CurvatureStart = static_cast<float>(1.0 / TurnRadius);
		OutEase.CurvatureEnd = 0.f;
		OutEase.RollStartDegrees = static_cast<float>(BankDeg);
		Out.Add(OutEase);
	}

	Straight(70.0, ETUSegmentZone::Brake, 0.f);           // brake run
	return Out;
}

void ATUCoasterRide::RebuildFromSegments()
{
	// A REBUILD IS NOT A TRANSITION. After this, channel 4 may be a different
	// block from the channel 4 being watched, so every stale baseline is a
	// transition that never happened — on most channels at once. Reseed.
	StateWatch.Forget();
	// And the show layer's own baseline, for the same reason: a layout that has
	// just gained or lost blocks has channels that no longer mean what they meant,
	// and carrying them across fires every cue on the rebuild.
	ShowPublisher.Rebuilt();

	FTrackDocument Doc;
	Doc.HeartlineHeight = 1.1;
	Doc.Segments.reserve(static_cast<std::size_t>(Segments.Num()));
	for (const FTUTrackSegment& S : Segments)
	{
		Doc.Segments.push_back(ToAuthored(S));
	}

	// Validate the AUTHORED list before building anything from it — the whole
	// reason TrackValidate exists is that the geometry cannot tell you a radius
	// was typed into a curvature field, it can only produce the consequences.
	//
	// KEPT, not only logged. The findings have been correct since Phase 0 and
	// invisible outside a log for just as long; the panel is what makes them
	// somewhere to go rather than something to scroll back for.
	LastDiagnostics = ValidateTrack(BuildSegments(Doc));
	for (const FTrackDiagnostic& D : LastDiagnostics)
	{
		UE_LOG(LogTemp, Warning, TEXT("Track segment %d: %s"),
			static_cast<int32>(D.SegmentIndex), UTF8_TO_TCHAR(D.Message.c_str()));
	}

	Track = ::BuildTrack(Doc);
	if (Track.NumSegments() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("TrackUnlimited: no usable segments; nothing to ride."));
		// Dropped rather than left pointing at the track that just failed to
		// build. An FTrain holds a reference to it, so keeping one would be a
		// train riding a track with no segments in it.
		Trains.Reset();
		StoppedForS.Reset();
		Signals.Reset();
		StopMarks.Reset();
		Drives.Reset();
		BlockSensors.Reset();
		Counter.Reset();
		return;
	}

	// Built once and handed to every train, rather than walked per train: the
	// zones ARE the track, and a second train that derived its own could disagree
	// with the first about where the brakes are.
	TArray<FTrackZone> Zones;
	ZoneReleaseSpeed.Reset();
	ZoneBrakeDecel.Reset();
	ZoneSpans.Reset();

	// Zones come from contiguous runs of segments carrying the same kind, so a
	// lift is however many segments in a row say "Lift" and moving one is an
	// edit rather than a recompile. Arc lengths are accumulated over the
	// segments the track actually ACCEPTED, so a rejected degenerate segment
	// cannot silently shift every zone boundary downstream of it.
	{
		ETUSegmentZone Open = ETUSegmentZone::None;
		double OpenS = 0.0;
		float OpenSpeed = 0.f;
		float OpenAccel = 6.f;
		float OpenDecel = 6.f;
		float OpenPad = 0.f;
		double AccS = 0.0;

		// Block boundaries fall out of this SAME walk, because they are the same
		// fact: a boundary is only meaningful where there is a device that can
		// hold a train, so one falls wherever a powered run starts or ends. No
		// new authored field, no new enumerator, and every track already placed
		// in a level gets blocks without being edited.
		//
		// It also inherits the accepted-geometry filter below for free, so a
		// rejected degenerate segment cannot shift a block boundary any more than
		// it can shift a zone boundary.
		std::vector<double> BlockStarts{0.0};
		auto AddBoundary = [&BlockStarts](double S)
		{
			// Ascending and unique. A zero-length block is one nothing can ever
			// be inside, and two boundaries at the same S would make one.
			if (S > BlockStarts.back())
			{
				BlockStarts.push_back(S);
			}
		};

		// Where each zone's stop mark gets bolted down. Filled in the same walk and
		// in the same order as ZoneReleaseSpeed, so the index is the zone's own.
		std::vector<double> StopMarkS;

		auto Close = [this, &Zones, &Open, &OpenS, &OpenSpeed, &OpenAccel, &OpenDecel,
			&OpenPad, &StopMarkS](double EndS)
		{
			if (Open == ETUSegmentZone::None || !(EndS > OpenS))
			{
				return;
			}
			// AUTHORED NOW, not one 6.0 for every device on every ride. That was
			// the same defect as the merged zone speeds before it: a number
			// nobody typed standing in for one they should have. NL2 exposes
			// exactly these two per device and it is right to — a chain hauling
			// at 0.61 g is nothing like a real chain.
			//
			// The kind still decides WHICH authority exists; these give the
			// magnitude of the ones it has.
			const double Accel = FMath::IsFinite(OpenAccel) && OpenAccel > 0.f
				? static_cast<double>(OpenAccel) : 6.0;
			const double Decel = FMath::IsFinite(OpenDecel) && OpenDecel > 0.f
				? static_cast<double>(OpenDecel) : 6.0;

			// THE PAD, if this device has one. Zero is "no pad", which is every
			// preset until somebody authors a rate -- and the zone then keeps the
			// negative BrakeLimit that means the second device is simply absent.
			const double Pad = FMath::IsFinite(OpenPad) && OpenPad > 0.f
				? static_cast<double>(OpenPad) : 0.0;

			// Sanitised HERE rather than trusted, because AddZone REFUSES a
			// malformed zone instead of storing it — and a refusal would leave the
			// train one zone shorter than ZoneReleaseSpeed, sliding every release
			// speed after it onto the wrong device. The field is clamped in the
			// editor, so this only catches a value that never came from the panel;
			// the point is that the two lists then cannot disagree at all, rather
			// than disagreeing rarely and silently.
			const double Speed = FMath::IsFinite(OpenSpeed) && OpenSpeed > 0.f
				? static_cast<double>(OpenSpeed) : 0.0;

			switch (Open)
			{
			case ETUSegmentZone::Lift:
				Zones.Add(FTrackZone{OpenS, EndS, Speed, Accel, Decel});
				break;
			case ETUSegmentZone::Launch:
				// NO BRAKING AUTHORITY, whatever was typed. That is what the
				// enumerator MEANS, and a number must not grant an authority the
				// device does not have -- a launch that could stop a train is a
				// different machine, and the dispatcher would park trains on it.
				Zones.Add(FTrackZone{OpenS, EndS, Speed, Accel, 0.0});
				break;
			case ETUSegmentZone::Brake:
				// And no tractive authority, for the same reason: a trim cannot
				// start a train, which is the whole distinction from a block brake.
				Zones.Add(FTrackZone{OpenS, EndS, Speed, 0.0, Decel});
				break;
			case ETUSegmentZone::BlockBrake:
			case ETUSegmentZone::Station:
			case ETUSegmentZone::StationUnload:
			case ETUSegmentZone::StationLoad:
				// Brakes AND drive tyres, so identical in shape to a lift chain.
				// The separate enumerators exist for the block boundary and for
				// the Details panel, not for the physics — what makes them hold a
				// train is having BOTH authorities, which is exactly what
				// FTrain::FindHoldZoneAt looks for.
				Zones.Add(FTrackZone{OpenS, EndS, Speed, Accel, Decel});
				break;
			default:
				return;
			}
			// A TRIM ALREADY IS A PAD, so it does not get a second one. Its whole
			// definition is a ceiling with no tractive authority, which is what
			// BrakeLimit means -- giving it another would be one device counted
			// twice and braking at the sum of its own two rates.
			if (Pad > 0.0 && Open != ETUSegmentZone::Brake && Zones.Num() > 0)
			{
				Zones.Last().BrakeLimit = Speed;
				Zones.Last().BrakeDecel = Pad;
			}
			ZoneBrakeDecel.Add(Open == ETUSegmentZone::Brake ? 0.0 : Pad);

			// The authored number is the RELEASE speed. A holding device spends
			// most of its life commanded to zero, so this is the only surviving
			// record of what it should open to.
			ZoneReleaseSpeed.Add(Speed);
			ZoneSpans.Add(FTUZoneSpan{OpenS, EndS, Open});

			// THE STOP MARK, surveyed rather than computed. Its nose clearance is
			// measured back from the far end of the device, because what the margin
			// exists to prevent is a train protruding into the next zone through a
			// defect. Clamped so a device barely longer than the train still puts
			// the mark where the whole train fits behind it.
			//
			// This is the ONE place train length is allowed to touch the holding
			// logic. Everything downstream reads a boolean.
			const double Mark = FMath::Max(OpenS + static_cast<double>(TrainLengthM),
				EndS - HoldNoseClearanceM);
			StopMarkS.push_back(Mark);

			// A DEVICE SHORTER THAN ITS TRAIN CANNOT HOLD IT, and the symptom is
			// vicious rather than obvious: the mark lands PAST the far end, no train
			// ever trips it, so a train sent to park there crawls straight out of
			// its own block into the next one and collides with whatever is
			// standing in it. Measured — a 10 m platform position and a 15 m train
			// gave seven signalling violations inside four seconds.
			//
			// Reported, not repaired, like every other authored-value check here:
			// clamping the mark back inside would produce a device that stops a
			// train with its nose hanging over the boundary, which is worse because
			// it looks like it worked.
			if (Mark > EndS && (Open == ETUSegmentZone::Station
				|| Open == ETUSegmentZone::StationLoad
				|| Open == ETUSegmentZone::StationUnload
				|| Open == ETUSegmentZone::BlockBrake || Open == ETUSegmentZone::Lift))
			{
				UE_LOG(LogTemp, Warning,
					TEXT("TrackUnlimited: the device at %.1f–%.1f m is %.1f m long and cannot "
						"hold a %.1f m train — its stop mark falls at %.1f m, past the end, so "
						"nothing will ever trip it. Lengthen the device or shorten the train."),
					OpenS, EndS, EndS - OpenS, TrainLengthM, Mark);
			}
		};

		// Anti-rollback runs, walked in the SAME pass and by the same rule as
		// zones — contiguous segments carrying the flag become one span — but kept
		// in their own list, because a catch is not a zone. It has no speed and
		// nothing commands it, and it overlaps zones freely: a lift hill is a
		// powered run and a ratchet at the same time, which a single enumerator
		// could not say.
		//
		// Deliberately NOT a block boundary either. A catch cannot release a train,
		// so it is no more a place to park one than a trim brake is.
		TArray<TPair<double, double>> CatchSpans;
		bool bCatchOpen = false;
		double CatchStartS = 0.0;

		for (int32 i = 0; i < Segments.Num(); ++i)
		{
			const double SegLength =
				BuildSegment(Doc.Segments[static_cast<std::size_t>(i)]).Length;
			if (!(SegLength > 0.0))
			{
				continue; // AddSegment refused it, so it occupies no arc length
			}
			if (Segments[i].bAntiRollback != bCatchOpen)
			{
				if (bCatchOpen)
				{
					CatchSpans.Add(TPair<double, double>(CatchStartS, AccS));
				}
				bCatchOpen = Segments[i].bAntiRollback;
				CatchStartS = AccS;
			}
			// A RUN ENDS WHERE THE DEVICE CHANGES, and the device is its kind AND
			// its speed. The speed half used to be missing: a run was defined by
			// kind alone, so a brake at 6 m/s followed immediately by one at 2 m/s
			// became a single zone targeting 6 and the 2 never existed. That is a
			// typed number the build throws away, which is exactly what this
			// project validates against everywhere else — and it was logged as a
			// warning rather than fixed, with the suggested workaround being to
			// wedge a different zone kind between the two.
			//
			// Splitting on it instead is both the smaller rule and the honest one:
			// two speeds is two devices, so it is two zones and two BLOCKS. That is
			// also what lets several holding devices be authored in a row at all —
			// a queue of brake sections keeping trains fed to a platform is not a
			// missing concept, it is block brakes, and until now ten of them in a
			// row authored as one block holding one train.
			//
			// No preset changes: every run in all four is a single speed already.
			const bool bKindChanged = Segments[i].Zone != Open;
			// AND ITS RATES, by exactly the argument that added speed. A brake
			// biting at 8 m/s^2 and one at 2 are two different machines, and
			// merging them discards a typed number to target whichever was
			// authored first -- which this project treats as a defect everywhere
			// else it appears.
			const bool bSpeedChanged = Open != ETUSegmentZone::None
				&& (!FMath::IsNearlyEqual(Segments[i].ZoneSpeed, OpenSpeed)
					|| !FMath::IsNearlyEqual(Segments[i].ZoneAccel, OpenAccel)
					|| !FMath::IsNearlyEqual(Segments[i].ZoneDecel, OpenDecel)
					|| !FMath::IsNearlyEqual(Segments[i].ZoneBrakeDecel, OpenPad));
			// And the author saying so outright, for devices that are identical in
			// every respect the walk can see and are still separate machines —
			// three loading positions on one platform, a queue of brake sections.
			const bool bDeclared = Segments[i].bStartsNewDevice
				&& Segments[i].Zone != ETUSegmentZone::None;
			if (bKindChanged || bSpeedChanged || bDeclared)
			{
				Close(AccS);
				AddBoundary(AccS);
				Open = Segments[i].Zone;
				OpenS = AccS;
				OpenSpeed = Segments[i].ZoneSpeed;
				OpenAccel = Segments[i].ZoneAccel;
				OpenDecel = Segments[i].ZoneDecel;
				OpenPad = Segments[i].ZoneBrakeDecel;
			}
			AccS += SegLength;
		}
		Close(AccS);
		if (bCatchOpen)
		{
			CatchSpans.Add(TPair<double, double>(CatchStartS, AccS));
		}

		// THE AUTHORED WALKWAYS, converted rather than derived. A person placed
		// these; this only turns start/stop/side into what the evacuation check
		// reads, and rejects the two ways a typed pair can be meaningless.
		WalkwaySpans.clear();
		for (const FTUWalkway& W : Walkways)
		{
			if (W.Side == ETUWalkway::None || !(W.EndS > W.StartS))
			{
				// Reported, never repaired, like every other authored-value check
				// here. Silently swapping a reversed pair would hide a typo behind
				// a catwalk that looks placed and is not where anybody put it.
				UE_LOG(LogTemp, Warning,
					TEXT("TrackUnlimited: walkway ignored — %.1f to %.1f m is not a span, "
						"or its side is None."), W.StartS, W.EndS);
				continue;
			}
			FWalkwaySpan Span;
			Span.StartS = W.StartS;
			Span.EndS = FMath::Min(static_cast<double>(W.EndS), AccS);
			Span.Side = ToWalkwaySide(W.Side);
			WalkwaySpans.push_back(Span);
		}

		// AN INVENTORY OF WHAT IS ACTUALLY ON THIS TRACK. The devices are the part
		// of a layout you cannot see, and the coloured rails only help once you are
		// looking at the right thing — so the build says what it found.
		//
		// It exists because of a specific confusion that is easy to hit and hard to
		// diagnose from the viewport: a station authored as a Lift is a lift. It
		// runs, it holds a train, it is drawn green, and it is wrong in nothing
		// except what it MEANS — no station process, no boarding sequence, and the
		// dispatch permissive missing its whole other half. A level saved before a
		// zone kind existed keeps the old kind until its preset is reloaded, and
		// nothing about that announces itself.
		{
			int32 ByKind[8] = {};
			for (const FTUZoneSpan& Z : ZoneSpans)
			{
				const int32 K = static_cast<int32>(Z.Kind);
				if (K >= 0 && K < 8) { ++ByKind[K]; }
			}
			const int32 Platforms_ =
				ByKind[static_cast<int32>(ETUSegmentZone::Station)]
				+ ByKind[static_cast<int32>(ETUSegmentZone::StationUnload)]
				+ ByKind[static_cast<int32>(ETUSegmentZone::StationLoad)];

			UE_LOG(LogTemp, Log,
				TEXT("TrackUnlimited: devices — %d station, %d unload, %d load, %d lift, "
					"%d launch, %d trim brake, %d block brake"),
				ByKind[static_cast<int32>(ETUSegmentZone::Station)],
				ByKind[static_cast<int32>(ETUSegmentZone::StationUnload)],
				ByKind[static_cast<int32>(ETUSegmentZone::StationLoad)],
				ByKind[static_cast<int32>(ETUSegmentZone::Lift)],
				ByKind[static_cast<int32>(ETUSegmentZone::Launch)],
				ByKind[static_cast<int32>(ETUSegmentZone::Brake)],
				ByKind[static_cast<int32>(ETUSegmentZone::BlockBrake)]);

			if (Platforms_ == 0 && ZoneSpans.Num() > 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("TrackUnlimited: no station on this track, so nothing runs a boarding "
						"sequence and the dispatch permissive is the interlocking alone. If it "
						"HAS a station, it is authored as a lift or a block brake — which works "
						"and is drawn green rather than blue. Reload the preset to fix a layout "
						"saved before the Station kind existed."));
			}
		}

		// Reported before it is used, not repaired silently — FRideSignals will
		// repair it either way, but a walk that produced something malformed is a
		// bug upstream and should say so rather than be absorbed.
		if (!FRideSignals::IsWellFormed(BlockStarts))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("TrackUnlimited: derived block boundaries are malformed; repairing."));
		}
		// A train needs somewhere to STAND: a device that can both stop it and
		// start it again. Drive tyres and block brakes qualify; a trim brake can
		// hold a train and never release it, and a launch can release one and
		// never hold it. So the number of trains the layout can run is the number
		// of hold-capable zones, and asking for more is refused rather than
		// granted into open course.
		TArray<double> HoldMidS;
		HoldZoneIndices.Reset();
		for (int32 z = 0; z < Zones.Num(); ++z)
		{
			if (Zones[z].MaxAccel > 0.0 && Zones[z].MaxDecel > 0.0)
			{
				// The MIDDLE, because that is where the dispatcher parks a held
				// train and where a whole train fits inside one block. Placed at a
				// zone's start instead, a 15 m train hangs back over the boundary —
				// and for the station, whose start is the seam, that means a train
				// placed there collides with one placed in the LAST block.
				HoldMidS.Add(0.5 * (Zones[z].StartS + Zones[z].EndS));
				HoldZoneIndices.Add(z);
			}
		}

		// ONE HOLDING PLACE MUST STAY FREE, or nothing can ever move: every train
		// is standing where the train behind it needs to go, and the ride gridlocks
		// without a single violation to show for it. MEASURED on this circuit,
		// which has five: four trains run clean and five never move at all.
		const int32 Wanted = FMath::Max(1, TrainCount);
		const int32 Running = FMath::Min(Wanted, FMath::Max(1, HoldMidS.Num() - 1));
		if (Running < Wanted)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("TrackUnlimited: %d trains asked for, %d run — the layout has %d place(s) "
					"that can both hold a train and release it, and one has to stay free for "
					"anything to move. Add a block brake or drive-tyre run to park another."),
				Wanted, Running, HoldMidS.Num());
		}

		// MEASURED, not authored. A layout either comes back to where it started
		// or it does not, and only the geometry knows — so nobody gets to tick a
		// "this is a circuit" box and be wrong about it. All three have to hold:
		// position, because a gap is a gap; HEADING, because a track that returns
		// to the right place pointing the wrong way is not joined; and ROLL,
		// because the rider would be flipped at the seam.
		//
		// A millimetre and a hundredth of a degree are far below anything a rider
		// or a mesh can resolve, and far above the ~1e-5 m that float-stored
		// authored values actually leave.
		{
			const double End = Track.TotalLength();
			const FTrackFrame A = Track.EvaluateAt(0.0);
			const FTrackFrame B = Track.EvaluateAt(End);
			const double Gap = Length(B.Position - A.Position);
			const double Head = FMath::RadiansToDegrees(FMath::Acos(
				FMath::Clamp(Dot(B.Tangent, A.Tangent), -1.0, 1.0)));
			const double Roll = FMath::RadiansToDegrees(FMath::Abs(B.Roll - A.Roll));
			bTrackIsCircuit = Gap < 1e-3 && Head < 0.01 && Roll < 0.01;

			UE_LOG(LogTemp, Log,
				TEXT("TrackUnlimited: seam %.6f m, %.6f deg heading, %.6f deg roll — %s"),
				Gap, Head, Roll,
				bTrackIsCircuit ? TEXT("CLOSED CIRCUIT, trains run laps")
								: TEXT("open layout, trains return to the station by teleport"));
		}

		// The switches go in with the track. Rebuilt wholesale for the same reason
		// the blocks are: move a brake run and you have moved its stop mark.
		StopMarks = MakeUnique<FTrackSensors>(StopMarkS);

		// And the motors, one per zone. A friction-only trim brake has no motor but
		// keeps its slot: an index that means the same thing in the zone list and
		// the drive list is worth more than the empty entry.
		//
		// ponytail: no ramp configured, so every output follows its command
		// instantly and nothing measured before drives existed moves. Give a device
		// a real FDriveSpec when a gentle station release is worth authoring, not
		// before — and note that a ramp slower than the zone's grip is the only kind
		// that changes anything.
		Drives = MakeUnique<FTrackDrives>(static_cast<std::size_t>(ZoneReleaseSpeed.Num()));
		ReportedDriveFault.Reset();   // new motors, so old faults are not this ride's

		// A platform for every station zone. One position each — a split operation
		// authors its unload and load as separate zones, which is why they are
		// separate kinds, and each gets its own sequence here.
		Platforms.Reset();
		for (int32 z = 0; z < ZoneSpans.Num(); ++z)
		{
			// NOT "Role": AActor has one, it is the network role, and shadowing it
			// is a warning-as-error here. The same class of collision CLAUDE.md
			// records for FFrame and FField, met on a member name instead of a type.
			EStationRole PlatformRole;
			switch (ZoneSpans[z].Kind)
			{
			case ETUSegmentZone::Station:       PlatformRole = EStationRole::Combined; break;
			case ETUSegmentZone::StationUnload: PlatformRole = EStationRole::Unload;   break;
			case ETUSegmentZone::StationLoad:   PlatformRole = EStationRole::Load;     break;
			default: continue;
			}
			FTUPlatform P;
			P.Zone = z;
			P.Process = FStationProcess(PlatformRole);
			P.Crew.UnloadSeconds = UnloadSeconds;
			P.Crew.LoadSeconds = LoadSeconds;
			P.Crew.SecureSeconds = RestraintCheckSeconds;
			P.Crew.Restraints.TravelSeconds = RestraintTravelSeconds;
			P.Crew.Restraints.Groups = FMath::Max(1, RestraintGroups);
			// Airgates: the same device in a different place, and they travel
			// faster than a restraint bar because they carry nothing.
			P.Crew.Gates.TravelSeconds = RestraintTravelSeconds * 0.5f;
			P.Crew.Gates.Groups = FMath::Max(1, RestraintGroups);
			Platforms.Add(P);
		}

		Signals = MakeUnique<FRideSignals>(BlockStarts, BlockBufferSeconds,
			static_cast<std::size_t>(FMath::Max(1, DispatchLookahead)),
			static_cast<std::size_t>(Running), bTrackIsCircuit);

		// Where those boundaries actually are on the track, walked once now rather
		// than integrated eleven times a frame to draw eleven posts.
		BuildBlockMarks();

		// THE PROGRAM'S IDENTITY, DERIVED FROM THE LAYOUT IT IS FOR.
		//
		// A digest of the derived blocks and zones — not of the authored segment
		// list, because two different segment lists that produce the same control
		// structure genuinely are the same program, and a geometry tweak that
		// moves no boundary should not invalidate it.
		//
		// The program is rebuilt here, so it matches by construction. What that
		// buys is the check: if the loaded program's identity ever DISAGREES with
		// the layout it is installed on, the controller refuses to run. That is
		// the detector for "I changed the code and the editor is still doing the
		// old thing" — a class that has bitten this project and had nothing
		// watching for it.
		{
			FSimDigest Id;
			for (double B : BlockStarts) { Id.Add(B); }
			for (const FTUZoneSpan& Z : ZoneSpans)
			{
				Id.Add(static_cast<int>(Z.Kind));
				Id.Add(Z.StartS);
				Id.Add(Z.EndS);
			}
			Plc.SetLayoutIdentity(Id.Value());
			Plc.LoadProgram(Id.Value());
		}

		// The second detection method: a switch at every block boundary, and a
		// counter that derives occupancy from their trips and nothing else.
		// Circuits only — see the header for why the ring wrap is a lie on an open
		// layout. Seeded once the trains are placed, further down.
		if (bTrackIsCircuit)
		{
			BlockSensors = MakeUnique<FTrackSensors>(BlockStarts);
			Counter = MakeUnique<FBlockCounter>(*BlockSensors);
		}
		else
		{
			BlockSensors.Reset();
			Counter.Reset();
		}

		// THE BRAKING-DISTANCE RULE, expressed at last. Tell the signalling which
		// blocks contain a device that can stop a train and let it go again, and
		// its permissive clears all the way to the next one of those rather than a
		// fixed count of blocks — because a train let into a block with nothing in
		// it is COMMITTED until it reaches somewhere it can stop.
		//
		// MEASURED, and this is not a refinement: with a fixed count, four trains
		// on this circuit COLLIDE — 14 violations at lookahead 1, 18 at lookahead 2
		// — because a train is granted a free block and finds the one beyond it
		// occupied on arrival. No single count can be right for a layout whose free
		// runs are 696 m and 184 m. With this, all four run clean.
		{
			std::vector<bool> CanHold(BlockStarts.size(), false);
			for (std::size_t b = 0; b < BlockStarts.size(); ++b)
			{
				const double S0 = BlockStarts[b];
				const double S1 = (b + 1 < BlockStarts.size()) ? BlockStarts[b + 1] : AccS;
				const double Mid = 0.5 * (S0 + S1);
				for (const FTrackZone& Z : Zones)
				{
					if (Mid >= Z.StartS && Mid <= Z.EndS
						&& Z.MaxAccel > 0.0 && Z.MaxDecel > 0.0)
					{
						CanHold[b] = true;
						break;
					}
				}
			}
			Signals->SetHoldingBlocks(CanHold);
		}

		{
			FString Where;
			for (const double S : Signals->Boundaries())
			{
				Where += FString::Printf(TEXT("%.1f  "), S);
			}
			UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: %d blocks, boundaries at %sm; "
				"lookahead %d, overlap %.1f s, %d train(s), %d holding place(s)"),
				static_cast<int32>(Signals->NumBlocks()), *Where,
				static_cast<int32>(Signals->Lookahead()), BlockBufferSeconds,
				Running, HoldMidS.Num());
		}

		FTrainConfig TrainConfig;
		TrainConfig.TrainLength = TrainLengthM;
		TrainConfig.bAllowRollback = bAllowRollback;

		Trains.Reset();
		StoppedForS.Init(0.f, Running);
		for (int32 t = 0; t < Running; ++t)
		{
			TUniquePtr<FTrain> New = MakeUnique<FTrain>(Track, TrainConfig);
			for (const FTrackZone& Z : Zones)
			{
				New->AddZone(Z);
			}
			for (const TPair<double, double>& C : CatchSpans)
			{
				New->AddAntiRollback(C.Key, C.Value);
			}
			// Mid-device, every one of them including the station — the same place
			// the dispatcher parks a held train, and the only placement that puts
			// a whole train inside a single block.
			New->Place(HoldMidS[t], 0.0);
			Trains.Add(MoveTemp(New));
		}

		BrakeStartS = AccS;
		for (int32 i = Segments.Num() - 1; i >= 0; --i)
		{
			if (Segments[i].Zone != ETUSegmentZone::Brake
				&& Segments[i].Zone != ETUSegmentZone::BlockBrake)
			{
				break;
			}
			BrakeStartS -= BuildSegment(Doc.Segments[static_cast<std::size_t>(i)]).Length;
		}
	}

	// Seed occupancy from where the trains actually are, before anything asks a
	// permissive. Without this the station block reads CLEAR with a train sitting
	// in it, and the first dispatch is granted against a lie.
	if (Signals)
	{
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			Signals->Update(static_cast<std::size_t>(t), Trains[t]->GetRearS(),
				Trains[t]->GetFrontS());
		}
	}

	// And SEED THE COUNTER the same way, which is the operator's sweep: walk the
	// ride, confirm where every train is, zero the counters. Without it the counter
	// is right in the middle of a run and wrong at the start of one — a train
	// beginning inside block 0 sends the block BEHIND it to minus one the moment
	// its tail leaves the first sensor, counted out of somewhere it was never
	// counted into. Real rides are swept for exactly this reason.
	if (BlockSensors && Signals)
	{
		// SCAN FIRST, THEN BUILD THE COUNTER. Its constructor snapshots the
		// sensors' edge totals, so a counter built afterwards starts from what the
		// switches already read and does not process the rising edges a train
		// parked on a boundary produced when the sensors were first covered. Built
		// before the scan, that train gets counted in AND seeded in — two trains in
		// a block holding one.
		ScanBlockSensors();
		Counter = MakeUnique<FBlockCounter>(*BlockSensors);

		// Seeded by SPAN rather than by centre block, because a train straddling a
		// boundary really is in both and the interlocking already says so. Asking
		// which blocks it holds is the only way to seed the two to agree — which is
		// the entire point of running them side by side.
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			for (std::size_t b = 0; b < Counter->NumBlocks(); ++b)
			{
				if (Signals->OccupiedBy(static_cast<std::size_t>(t), b))
				{
					Counter->Seed(b);
				}
			}
		}
	}

	// Where the ride's lowest structural point sits relative to the heartline
	// origin, so ToWorld can lift the whole thing onto the ground.
	//
	// The heartline is RIDER height, not track height: at the station the rails
	// hang 1.1 m below it and the spine another 0.45 m below those, so track
	// z = 0 is about 1.7 m above the bottom of the structure. Place the actor at
	// world z = 0 without accounting for that and the station buries itself —
	// the rails go under, and the camera, which rides the heartline exactly,
	// gets cut in half by the ground plane.
	//
	// Computed, not typed, and that is the point. The offset depends on the
	// whole layout: a banked or inverted section puts the structure somewhere
	// else entirely relative to the heartline, and every edit to the track moves
	// it again. A constant here would be a number someone has to remember to
	// re-derive, which is exactly how this ended up as a hand-tuned actor Z
	// twice already.
	{
		const double SpineRadius = Profile.SpineDiameter * 0.5;
		const double Total = Track.TotalLength();
		double Lowest = 0.0;

		FTrackFrame Walk = Track.EvaluateAt(0.0);
		double S = 0.0;
		for (;;)
		{
			const FTrackCrossSection Section =
				CrossSectionAt(Walk, Track.GetHeartlineHeight(), Profile);
			Lowest = FMath::Min3(Lowest, FMath::Min(Section.LeftRail.Z, Section.RightRail.Z),
				Section.SpineCentre.Z - SpineRadius);
			if (S >= Total)
			{
				break;
			}
			// 1 m steps. This is looking for a minimum over a smooth curve, not
			// resolving geometry — the sag between samples is far below the
			// clearance any real support structure needs anyway.
			const double Next = FMath::Min(S + 1.0, Total);
			Walk = Track.AdvanceFrom(Walk, S, Next);
			S = Next;
		}
		GroundOffsetM = -Lowest;
	}

	// Everything an author needs to know about the edit they just made, without
	// riding it. Each of these was a defect that took riding the track to find:
	// a discontinuity, an underground back half, and a loop passing through
	// itself. None of them are visible in a G trace.
	// Run the whole ride now, at edit time. An author needs to know a hill is
	// too tall BEFORE watching a train fail to crest it, and the extremes are
	// worth more with a position attached: "4.25 g" is a number, "4.25 g at
	// S=310 m" is somewhere to go and look.
	//
	// Run on train 0 with every holding device still OPEN, which is deliberate:
	// the profile answers "what does this layout do to a rider", not "what does
	// the signalling do to a timetable". A profile measured through a red would
	// report a stall at the first block brake and call the ride broken.
	Profile_ = RunRideProfile(*Trains[0], Track, 1.0);

	// ===================== WHAT THE DEVICES WILL ACTUALLY DO =====================
	//
	// HERE rather than in BuildDiagnostics, because this is the one place the
	// derived zones and the ride profile both exist. Deriving the zones a second
	// time to ask these questions would be a second derivation that can disagree
	// with the first, which this project treats as a defect wherever it appears.
	//
	// THE AUTHORITIES COME FROM THE ZONE, NOT FROM THE SEGMENT ENUM, and they come
	// from the SAME TEST the interlocking uses one screen down — MaxAccel > 0 and
	// MaxDecel > 0. Reading the enum here instead would be a second opinion about
	// what a block boundary is, and the day the two disagreed the panel would be
	// reassuring somebody about a layout the signalling was refusing to run.
	{
		LastDeviceFindings.clear();
		std::vector<FDeviceSpan> Devices;
		Devices.reserve(static_cast<std::size_t>(Zones.Num()));
		for (const FTrackZone& Z : Zones)
		{
			FDeviceSpan D;
			D.StartS = Z.StartS;
			D.EndS = Z.EndS;
			D.CommandedSpeed = Z.TargetSpeed;
			D.bCanHold = Z.MaxDecel > 0.0;
			D.bCanRelease = Z.MaxAccel > 0.0;
			// THE ZONE'S OWN RATE. The audit had a global service deceleration and
			// it did not match the Grip these are built with, so it was predicting
			// stopping distances the physics would never produce.
			// WHAT ACTUALLY STOPS THE TRAIN is the harder of the two devices, and
			// on a section with a pad that is the pad. Reading MaxDecel alone
			// would report a brake as too short that stops comfortably.
			D.DecelMs2 = FMath::Max(Z.MaxDecel, Z.BrakeDecel);
			D.bIsBlockBoundary = D.bCanHold && D.bCanRelease;

			// The name is only for the message. FTrackZone drops the kind on
			// purpose — a station, a block brake and a lift chain are the same
			// physics — so it is looked up by midpoint from the list that kept it.
			const double Mid = 0.5 * (Z.StartS + Z.EndS);
			D.Name = "device";
			for (const FTUZoneSpan& Span : ZoneSpans)
			{
				if (Mid >= Span.StartS && Mid <= Span.EndS)
				{
					D.Name = TCHAR_TO_UTF8(ZoneKindName(Span.Kind));
					break;
				}
			}
			Devices.push_back(D);
		}

		FDeviceAuditSettings Audit;
		Audit.TrainLengthM = static_cast<double>(TrainLengthM);
		Audit.NoseClearanceM = HoldNoseClearanceM;

		// A RIDE THAT DID NOT COMPLETE HAS NO SPEEDS TO READ, and reporting a
		// braking distance against a profile that stopped at 46 m would be the
		// envelope suite's own old failure — a verdict on a ride that did not
		// happen. The length and authority checks need no speed and still run;
		// this reports 0 past the end, which is silent rather than wrong.
		const FRideProfile& P = Profile_;
		LastDeviceFindings = AuditDevices(Devices, Audit, [&P](double S) -> double
		{
			if (!P.bCompleted || P.Samples.empty()) { return 0.0; }
			// Samples are ordered in S. A handful of devices against a few
			// thousand samples does not need a binary search to find them.
			double Best = 0.0;
			for (const FRideSample& Sm : P.Samples)
			{
				if (Sm.S > S) { break; }
				Best = Sm.Speed;
			}
			return Best;
		});
	}

	// BACK TO ITS HOLDING POSITION, not to zero. The profile run leaves train 0
	// wherever the ride ended, so it has to be put back — but putting it at 0.0
	// puts it ON THE SEAM, where a train straddles the boundary and legitimately
	// holds the first block and the last. That was harmless while nothing checked,
	// and the moment the counter cross-check arrived it tripped the ride on frame
	// one: the interlocking held blocks 0 and 10, the counter had been seeded for
	// block 0 only, and the two disagreed exactly as they are supposed to when one
	// of them is wrong.
	//
	// Which is the cross-check doing its job on its first run, against a line that
	// had been quietly wrong since before any of this existed.
	{
		// Mid of the first holding device, which is where it was placed to begin
		// with. Recomputed from HoldZoneIndices rather than reaching for the local
		// the placement loop used, which is long out of scope by here.
		double Home = 0.0;
		if (HoldZoneIndices.Num() > 0)
		{
			const FTrackZone Z =
				Trains[0]->GetZone(static_cast<std::size_t>(HoldZoneIndices[0]));
			Home = 0.5 * (Z.StartS + Z.EndS);
		}
		Trains[0]->Place(Home, 0.0);
	}

	// NOW shut them, once the profile has been taken. Brakes-on is the resting
	// state of real ride control, and the alternative — open until a dispatcher
	// notices — is open for exactly one frame every time, which is one frame of a
	// train being pushed through a red.
	//
	// And NOW close the circuit, for the same reason and in the same breath: the
	// profile walks arc length forwards and stops at the end, so on a lapping
	// train it would never stop and its samples would overwrite each other. The
	// ride profile is one lap, measured open; the ride itself laps.
	// Shut at the DRIVE now, not on each train. Every drive is preset — commanded
	// and already at its output, because a ride opens with its motors running and
	// one that ramps up from zero on the first frame of the session is a lift chain
	// standing still when the first train reaches it. Holding devices preset to
	// zero; everything else to what it was authored at.
	if (Drives)
	{
		for (int32 z = 0; z < ZoneReleaseSpeed.Num(); ++z)
		{
			Drives->Preset(static_cast<std::size_t>(z),
				HoldZoneIndices.Contains(z) ? 0.0 : ZoneReleaseSpeed[z]);
		}
	}
	for (const TUniquePtr<FTrain>& T : Trains)
	{
		for (const int32 Zi : HoldZoneIndices)
		{
			T->SetZoneTargetSpeed(static_cast<std::size_t>(Zi), 0.0);
		}
		T->SetCircuit(bTrackIsCircuit);
	}

	if (!Profile_.bCompleted)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrackUnlimited: the train does not get round — it %s at %.1f m, %.1f m up. ")
			TEXT("Everything past that point is unridden."),
			Profile_.bRolledBack ? TEXT("ROLLS BACK") : TEXT("stalls"),
			Profile_.StalledAtS, Profile_.StalledHeight);
	}
	else
	{
		UE_LOG(LogTemp, Log,
			TEXT("TrackUnlimited ride: %.0f s | top %.1f km/h at %.0f m | vertical %+.2f at %.0f m ")
			TEXT("to %+.2f at %.0f m | lateral %.2f at %.0f m | roll rate %.0f deg/s at %.0f m"),
			Profile_.Duration, Profile_.TopSpeed * 3.6, Profile_.TopSpeedS, Profile_.MinVerticalG,
			Profile_.MinVerticalGS, Profile_.MaxVerticalG, Profile_.MaxVerticalGS,
			Profile_.MaxAbsLateralG, Profile_.MaxAbsLateralGS, Profile_.MaxAbsRollRate,
			Profile_.MaxAbsRollRateS);
	}

	const FClosureGap Gap = MeasureClosure(Track, HeightTarget(Track));
	const FClearanceReport Clear = AnalyseSelfClearance(Track, Profile, 1.0, 12.0);
	UE_LOG(LogTemp, Log,
		TEXT("TrackUnlimited: %d segments, %.1f m, C2=%s | ends %+.2f m vs station | ")
		TEXT("closest approach %.2f m%s | sits %.2f m above the heartline origin"),
		static_cast<int32>(Track.NumSegments()), Track.TotalLength(),
		Track.IsCurvatureContinuous() ? TEXT("yes") : TEXT("NO"), Gap.HeightError,
		Clear.ClosestApproach,
		Clear.bStructureOverlaps ? TEXT(" (TRACK PASSES THROUGH ITSELF)") : TEXT(""),
		GroundOffsetM);

	// AFTER GroundOffsetM is known, because the port lifts every vertex by it —
	// the heartline origin is RIDER height and z = 0 in track space is about
	// 1.7 m above the bottom of the spine. Built here rather than beside
	// DrawTrack so both the editor preview and BeginPlay get it from one place.
	// THE STYLE IS THE SECTION AS WELL AS THE COLOUR, and it is applied HERE
	// rather than inside RebuildTrackMesh — which returns early when the mesh is
	// off, and would then leave the wireframe and the support planner reading a
	// gauge nobody chose. Everything downstream shares one Profile, so it has to
	// be set before any of them run.
	//
	// Changing a gauge without changing the paint would give a different track
	// that looks the same, which is exactly the confusion a "style" prevents — so
	// one pick moves both.
	{
		const FTUTrackStyle Style = ActiveStyle();
		Profile.Gauge = Style.GaugeM;
		Profile.RailDiameter = Style.RailDiameterM;
		Profile.SpineDrop = Style.SpineDropM;
		Profile.SpineDiameter = Style.SpineDiameterM;
		Profile.TieSpacing = Style.TieSpacingM;
		Profile.TieDiameter = Style.TieDiameterM;
	}

	RebuildTrackMesh();

	// AFTER everything else, because it reads the ride profile and the derived
	// blocks as well as the validator — and a panel showing findings about a
	// track that has just been replaced is worse than an empty one.
	BuildDiagnostics();
}

#if WITH_EDITOR
void ATUCoasterRide::PostEditChangeProperty(FPropertyChangedEvent& Event)
{
	// Before Super, deliberately: Super reruns the construction script, and
	// OnConstruction is what rebuilds and redraws. Resetting afterwards would
	// leave the preview showing the layout that was just replaced.
	if (bLoadPreset)
	{
		bLoadPreset = false;
		Segments = PresetLayout(Preset);
		ApplyPresetTrainSetup(Preset);
	}

	// ===================== EDITS ARE A MODE QUESTION =====================
	//
	// CONSTRAINT 1, ONE LEVEL UP. A ride that is running is not a ride being
	// edited, and an edit landing mid-lap would change the geometry under a
	// train — which on a circuit means changing the track a train is standing on.
	//
	// REFUSED rather than deferred: applying it when the mode changes back would
	// mean somebody's edit taking effect minutes later with no memory of having
	// made it. The message says which mode to be in, because a field that
	// silently reverts is indistinguishable from one that is broken.
	if (!Session.EditsAllowed())
	{
		UE_LOG(LogTUEvents, Warning,
			TEXT("edit refused: the ride is in %s. [Tab] back to BUILD to change the track."),
			UTF8_TO_TCHAR(AppModeName(Session.Mode())));
		Super::PostEditChangeProperty(Event);
		return;
	}

	// Rebuild and redraw happen in OnConstruction, so a typed number gets an
	// answer immediately — the drawn track, plus total length, continuity, where
	// it ends up and whether it hits itself. The viewport stays a read-only
	// preview: this is feedback, not manipulation.
	Super::PostEditChangeProperty(Event);
}
#endif

FVector ATUCoasterRide::ToLocal(const FVec3& V) const
{
	// Mirror Y: the prototype frame is right-handed, Unreal is left-handed.
	// Lift by GroundOffsetM: the heartline origin is RIDER height, not track
	// height, so z = 0 in track space is about 1.7 m above the bottom of the
	// spine. See RebuildFromSegments for why this is computed rather than typed.
	return FVector(V.X, -V.Y, V.Z + GroundOffsetM) * MetresToUU;
}

FVector ATUCoasterRide::ToLocalDirection(const FVec3& V) const
{
	// The same mirror and NOTHING ELSE. A normal put through ToLocal would come
	// back as a point a metre and a half above the origin rather than a
	// direction, which reads as lighting that is subtly wrong everywhere.
	return FVector(V.X, -V.Y, V.Z);
}

FVector ATUCoasterRide::ToWorld(const FVec3& V) const
{
	return GetActorLocation() + ToLocal(V);
}

FQuat ATUCoasterRide::ToWorldRotation(const FTrackFrame& Frame) const
{
	// Measured, not guessed: mirroring Y flips handedness, so the rider's LEFT
	// must be negated to become Unreal's +Y (right). Using the mirrored lateral
	// directly gives an exactly inverted basis, and the track still looks
	// self-consistent while being mirrored — see Docs/PHASE0_FINDINGS.md.
	const FVector Forward(Frame.Tangent.X, -Frame.Tangent.Y, Frame.Tangent.Z);
	const FVector Right(-Frame.Lateral.X, Frame.Lateral.Y, -Frame.Lateral.Z);
	const FVector Up(Frame.Up.X, -Frame.Up.Y, Frame.Up.Z);

	// Built from the basis directly. The frame is already exactly orthonormal,
	// so going via FRotator angles would only reintroduce error.
	return FMatrix(Forward, Right, Up, FVector::ZeroVector).ToQuat();
}

void ATUCoasterRide::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	if (PlayerInputComponent == nullptr)
	{
		return;
	}

	// ponytail: direct key bindings, no Enhanced Input mapping context and no
	// input assets. Enhanced Input's real advantages — rebinding, modifiers,
	// layered contexts — only arrive with content assets to hold them, and
	// creating those objects transiently in code buys the machinery without
	// most of the benefit. This is ~20 lines and works. Move to Enhanced Input
	// when a shipped build needs users to rebind keys, which is a Phase 6
	// concern rather than a today one.
	PlayerInputComponent->BindKey(EKeys::C, IE_Pressed, this, &ATUCoasterRide::CycleCameraMode);

	// THE OPERATOR. Space is the dispatch button and is bound on BOTH edges,
	// because the release is half the safety rule — a control that only ever
	// reports "pressed" is a control that can be wedged. Backspace stops the ride
	// and End resets it, deliberately far apart on the keyboard: the two are
	// nowhere near each other on a real panel either.
	PlayerInputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this,
		&ATUCoasterRide::PressDispatch);
	PlayerInputComponent->BindKey(EKeys::SpaceBar, IE_Released, this,
		&ATUCoasterRide::ReleaseDispatch);
	// BACKSPACE IS WANTED TWICE — the emergency stop, and deleting a digit. It
	// dispatches on whether a field has focus rather than being bound twice,
	// because two handlers on one key both fire.
	PlayerInputComponent->BindKey(EKeys::BackSpace, IE_Pressed, this,
		&ATUCoasterRide::KeyBackspace);
	// MONITORED RESET: pressed AND released, both bound, because the reset happens
	// on the release. A key bound only on press is a taped button.
	PlayerInputComponent->BindKey(EKeys::End, IE_Pressed, this,
		&ATUCoasterRide::PressResetButton);
	PlayerInputComponent->BindKey(EKeys::End, IE_Released, this,
		&ATUCoasterRide::ReleaseResetButton);
	PlayerInputComponent->BindKey(EKeys::P, IE_Pressed, this,
		&ATUCoasterRide::CyclePanelView);
	// NOT [A] — that is strafe-left, and acknowledging a fault every time somebody
	// moved the free camera is exactly the kind of accidental press the whole
	// acknowledge/reset ordering exists to prevent.
	//
	// A real console separates ACKNOWLEDGE and RESET physically, onto different
	// coloured fields. On a keyboard that separation cannot be meaningful, so the
	// ordering is enforced in software instead: a reset is REFUSED while anything
	// is unacknowledged.
	PlayerInputComponent->BindKey(EKeys::Home, IE_Pressed, this,
		&ATUCoasterRide::AcknowledgeFaults);

	// ONLY THE MOUSE IS AN AXIS HERE. BindAxisKey asserts on IsAxis1D(), and a
	// letter key is a BUTTON — WASDQE were bound this way and were always wrong,
	// surviving only because `ensure` logs and carries on. They are polled in
	// PollMovementKeys instead, which is also six bindings and six methods less.
	PlayerInputComponent->BindAxisKey(EKeys::MouseX, this, &ATUCoasterRide::AxisLookYaw);
	PlayerInputComponent->BindAxisKey(EKeys::MouseY, this, &ATUCoasterRide::AxisLookPitch);

	// HOLD RIGHT TO LOOK. The cursor is free wherever there are rows to click, so
	// a camera that turned on every mouse move would make reaching for a row a
	// gamble on where it had gone by the time you got there. Right rather than
	// left because left is already select, and every editor with a viewport and
	// a panel side by side splits them exactly this way.
	PlayerInputComponent->BindKey(EKeys::RightMouseButton, IE_Pressed, this,
		&ATUCoasterRide::BeginLookDrag);
	PlayerInputComponent->BindKey(EKeys::RightMouseButton, IE_Released, this,
		&ATUCoasterRide::EndLookDrag);

	// ORBIT. [F] frames the whole track, which is the key you press constantly
	// once a validation warning points somewhere and you have no idea where. The
	// wheel zooms multiplicatively, so one notch means the same proportion of the
	// distance at 10 m and at 1000 m.
	PlayerInputComponent->BindKey(EKeys::F, IE_Pressed, this,
		&ATUCoasterRide::FrameWholeTrack);

	// THE RIDE PROFILE, as a graph you can read a number off. [G] cycles the
	// channel and [H] hides it — one at a time rather than four overlaid, because
	// four traces on one axis is a picture rather than a reading.
	PlayerInputComponent->BindKey(EKeys::G, IE_Pressed, this,
		&ATUCoasterRide::CycleProfileChannel);
	PlayerInputComponent->BindKey(EKeys::H, IE_Pressed, this,
		&ATUCoasterRide::ToggleProfileGraph);
	PlayerInputComponent->BindKey(EKeys::V, IE_Pressed, this,
		&ATUCoasterRide::ToggleDiagnostics);
	// [Z] frames the selected segment; clicking a diagnostics row selects one and
	// frames it in the same gesture, which is the whole point of a finding
	// carrying a place.
	PlayerInputComponent->BindKey(EKeys::Z, IE_Pressed, this,
		&ATUCoasterRide::FrameSelectedSegment);
	// ONE BINDING, dispatching by mode. Two handlers on one key both fire and
	// which one "wins" depends on registration order — the exact ambiguity
	// FInputMap's conflict test exists to warn about, and shipping it in the same
	// repository as that test would be poor form.
	PlayerInputComponent->BindKey(EKeys::LeftMouseButton, IE_Pressed, this,
		&ATUCoasterRide::ClickPrimary);
	// BUILD / OPERATE / RIDE. The mode decides the camera, the panels and whether
	// edits are accepted at all — which is what makes it a mode rather than a
	// label on a screen.
	PlayerInputComponent->BindKey(EKeys::Tab, IE_Pressed, this,
		&ATUCoasterRide::CycleAppMode);

	// [F2] — every overlay off, for a screenshot. Not routed through
	// IsTypingInField: F2 is not a character, so it cannot be part of a number
	// somebody is entering, and a function key that stopped working inside the
	// segment editor would be a worse surprise than one that always works.
	PlayerInputComponent->BindKey(EKeys::F2, IE_Pressed, this,
		&ATUCoasterRide::ToggleOverlays);

	// THE SEGMENT EDITOR. Numeric entry only — constraint 1 holds absolutely, and
	// a digit key IS typed entry rather than a stepper wearing its name.
	PlayerInputComponent->BindKey(EKeys::B, IE_Pressed, this,
		&ATUCoasterRide::ToggleSegmentEditor);
	// [ and ] walk the selection, and frame it in orbit — the answer to "which
	// index is that piece", which the list could not previously give.
	PlayerInputComponent->BindKey(EKeys::LeftBracket, IE_Pressed, this,
		&ATUCoasterRide::SelectPrevSegment);
	PlayerInputComponent->BindKey(EKeys::RightBracket, IE_Pressed, this,
		&ATUCoasterRide::SelectNextSegment);
	PlayerInputComponent->BindKey(EKeys::Enter, IE_Pressed, this,
		&ATUCoasterRide::CommitField);
	PlayerInputComponent->BindKey(EKeys::Escape, IE_Pressed, this,
		&ATUCoasterRide::CancelField);
	PlayerInputComponent->BindKey(EKeys::Decimal, IE_Pressed, this,
		&ATUCoasterRide::TypePoint);
	// The digits, main row and numpad. Ten methods rather than ten lambdas,
	// because BindKey wants a no-argument method either way.
	const FKey Row[10] = {EKeys::Zero, EKeys::One, EKeys::Two, EKeys::Three, EKeys::Four,
		EKeys::Five, EKeys::Six, EKeys::Seven, EKeys::Eight, EKeys::Nine};
	const FKey Pad[10] = {EKeys::NumPadZero, EKeys::NumPadOne, EKeys::NumPadTwo,
		EKeys::NumPadThree, EKeys::NumPadFour, EKeys::NumPadFive, EKeys::NumPadSix,
		EKeys::NumPadSeven, EKeys::NumPadEight, EKeys::NumPadNine};
	void (ATUCoasterRide::*Digit[10])() = {
		&ATUCoasterRide::Type0, &ATUCoasterRide::Type1, &ATUCoasterRide::Type2,
		&ATUCoasterRide::Type3, &ATUCoasterRide::Type4, &ATUCoasterRide::Type5,
		&ATUCoasterRide::Type6, &ATUCoasterRide::Type7, &ATUCoasterRide::Type8,
		&ATUCoasterRide::Type9};
	for (int32 i = 0; i < 10; ++i)
	{
		PlayerInputComponent->BindKey(Row[i], IE_Pressed, this, Digit[i]);
		PlayerInputComponent->BindKey(Pad[i], IE_Pressed, this, Digit[i]);
	}
	PlayerInputComponent->BindKey(EKeys::Hyphen, IE_Pressed, this,
		&ATUCoasterRide::TypeMinus);
	PlayerInputComponent->BindKey(EKeys::Subtract, IE_Pressed, this,
		&ATUCoasterRide::TypeMinus);

	// THE SIM CLOCK. Watching a buffer count down at quarter speed is how
	// somebody learns what the interlocking is doing, and stepping one scan at a
	// time is how they see a permissive drop the frame a restraint opens.
	PlayerInputComponent->BindKey(EKeys::Pause, IE_Pressed, this,
		&ATUCoasterRide::TogglePause);
	// And the full stop: one scan when the ride is what you are looking at, a
	// decimal point when a field is.
	PlayerInputComponent->BindKey(EKeys::Period, IE_Pressed, this,
		&ATUCoasterRide::KeyPeriod);
	PlayerInputComponent->BindKey(EKeys::Comma, IE_Pressed, this,
		&ATUCoasterRide::SlowDown);
	PlayerInputComponent->BindKey(EKeys::Slash, IE_Pressed, this,
		&ATUCoasterRide::SpeedUp);
	PlayerInputComponent->BindKey(EKeys::MouseScrollUp, IE_Pressed, this,
		&ATUCoasterRide::OrbitZoomIn);
	PlayerInputComponent->BindKey(EKeys::MouseScrollDown, IE_Pressed, this,
		&ATUCoasterRide::OrbitZoomOut);

	PlayerInputComponent->BindKey(EKeys::LeftShift, IE_Pressed, this,
		&ATUCoasterRide::BoostOn);
	PlayerInputComponent->BindKey(EKeys::LeftShift, IE_Released, this,
		&ATUCoasterRide::BoostOff);
}

void ATUCoasterRide::CycleCameraMode()
{
	switch (CameraMode)
	{
	case ETUCameraMode::Rider: CameraMode = ETUCameraMode::Chase; break;
	case ETUCameraMode::Chase: CameraMode = ETUCameraMode::Free; break;
	case ETUCameraMode::Free:  CameraMode = ETUCameraMode::Orbit; break;
	default: CameraMode = ETUCameraMode::Rider; break;
	}
	// Re-seed on the way in, so the free camera starts from wherever you were
	// just looking rather than teleporting you somewhere unrecognisable.
	bFreeInitialised = false;
}

EEditKind ATUCoasterRide::KindOf(ETUSegmentKind K)
{
	switch (K)
	{
	case ETUSegmentKind::Arc:      return EEditKind::Arc;
	case ETUSegmentKind::Clothoid: return EEditKind::Clothoid;
	case ETUSegmentKind::Helix:    return EEditKind::Helix;
	default:                       return EEditKind::Straight;
	}
}

double ATUCoasterRide::ReadField(const FTUTrackSegment& S, EEditField F) const
{
	switch (F)
	{
	case EEditField::Length:         return S.Length;
	case EEditField::Radius:         return S.Radius;
	case EEditField::CurvatureStart: return S.CurvatureStart;
	case EEditField::CurvatureEnd:   return S.CurvatureEnd;
	case EEditField::ClimbAngle:     return S.ClimbAngleDegrees;
	case EEditField::Turns:          return S.Turns;
	case EEditField::Roll:           return S.RollEndDegrees;
	case EEditField::ZoneSpeed:      return S.ZoneSpeed;
	default:                         return 0.0;
	}
}

void ATUCoasterRide::WriteField(FTUTrackSegment& S, EEditField F, double V)
{
	switch (F)
	{
	case EEditField::Length:         S.Length = static_cast<float>(V); break;
	case EEditField::Radius:         S.Radius = static_cast<float>(V); break;
	case EEditField::CurvatureStart: S.CurvatureStart = static_cast<float>(V); break;
	case EEditField::CurvatureEnd:   S.CurvatureEnd = static_cast<float>(V); break;
	case EEditField::ClimbAngle:     S.ClimbAngleDegrees = static_cast<float>(V); break;
	case EEditField::Turns:          S.Turns = static_cast<float>(V); break;
	case EEditField::Roll:           S.RollEndDegrees = static_cast<float>(V); break;
	case EEditField::ZoneSpeed:      S.ZoneSpeed = static_cast<float>(V); break;
	default: break;
	}
}

void ATUCoasterRide::DrawSegmentEditor(UCanvas* Canvas)
{
	if (!bShowSegmentEditor || !Canvas || !GEngine) { return; }

	EditorRowRects.Reset();
	EditorRowField.Reset();

	const float Row = 15.f;
	const float W = 380.f;   // wider since a row now carries arc length and a zone tag

	// LOWER RIGHT, because it is the only corner nothing else wants. The upper
	// left is the telemetry readout's and this drew straight over it; the upper
	// right belongs to the diagnostics panel, which is 620 wide.
	//
	// ANCHORED TO THE CANVAS rather than typed, so it stays in the corner at any
	// resolution — and the tooltip's two lines are counted into the anchor, or
	// the thing that explains a field is the thing that falls off the bottom of
	// the screen.
	const float BodyH = 24.f + Row * 20.f;
	const float TipH = 6.f + Row * 2.f;
	const float Ox = Canvas->SizeX - W - 20.f;
	float Y = Canvas->SizeY - (BodyH + TipH) - 20.f;

	PanelTile(Canvas, Ox - 8.f, Y - 8.f, W + 16.f, BodyH, PanelGround);

	if (Segments.Num() == 0)
	{
		PanelLabel(Canvas, Ox, Y, TEXT("SEGMENTS   [B] hide"), PanelDim);
		PanelLabel(Canvas, Ox, Y + 20.f,
			UTF8_TO_TCHAR(EmptyStateFor(EPanelKind::SegmentList)), PanelDim);
		return;
	}

	// EDITS ARE A MODE QUESTION, and the panel says so rather than simply
	// ignoring keystrokes — a field that has stopped accepting numbers with no
	// explanation is indistinguishable from a broken one.
	// NOT bEditable -- AActor already has one under WITH_EDITORONLY_DATA, and a
	// local that shadows it compiles in a packaged build and fails only in the
	// editor. Named for what it asks rather than for what it is.
	const bool bEditsAllowed = Session.EditsAllowed();
	PanelLabel(Canvas, Ox, Y, bEditsAllowed
		? TEXT("SEGMENTS   [B] hide   click a field, type, Enter")
		: TEXT("SEGMENTS   read-only while the ride runs   [Tab] to BUILD"),
		bEditsAllowed ? PanelDim : PanelAmber);
	Y += 20.f;

	// ---- The list. Windowed around the selection, because a 23-segment layout
	// fits and a CSV import of four thousand does not — and a list that drew all
	// of them would be a wall rather than a panel.
	const int32 Window = 8;
	const int32 First = FMath::Max(0, FMath::Min(SelectedSegment - Window / 2,
		Segments.Num() - Window));
	const int32 Last = FMath::Min(Segments.Num(), First + Window);

	// WHERE, AND NOT ONLY WHAT. A row that says only its kind and its length
	// cannot be matched to anything else on the screen, and the question somebody
	// actually has is the reverse of the one the list answers: not "what is
	// segment 12" but "which index is the piece I am looking at".
	//
	// Arc length is what makes that answerable, because it is the coordinate
	// everything else here already uses — the ride-profile graph is plotted
	// against S, its scrubber reads out in S, every diagnostics row carries S,
	// and REFERENCE_LAYOUT.md publishes its zones as S ranges. Without this
	// column the only way across was to select an index, press [Z], and look.
	//
	// The zone tag is the other half: adding a brake means finding somewhere that
	// is not already a device, and a list that did not say which segments are
	// devices made that a second pass through the Details panel.
	double SAt = 0.0;
	for (int32 i = 0; i < FMath::Max(0, First); ++i)
	{
		SAt += static_cast<double>(Segments[i].Length);
	}
	for (int32 i = FMath::Max(0, First); i < Last; ++i)
	{
		EditorRowRects.Add(FVector4(Ox, Y, Ox + W, Y + Row));
		EditorRowField.Add(-1000 - i);
		const bool bSel = i == SelectedSegment;
		if (bSel) { PanelTile(Canvas, Ox - 4.f, Y - 1.f, W + 8.f, Row, PanelRule); }
		const bool bDevice = Segments[i].Zone != ETUSegmentZone::None;
		PanelLabel(Canvas, Ox + 4.f, Y,
			FString::Printf(TEXT("%2d  %-8s %6.1f m  @%.0f  %s"), i,
				*UEnum::GetDisplayValueAsText(Segments[i].Kind).ToString(),
				Segments[i].Length, SAt,
				bDevice ? ZoneKindName(Segments[i].Zone) : TEXT("")),
			bSel ? PanelCyan : (bDevice ? PanelAmber : PanelText));
		SAt += static_cast<double>(Segments[i].Length);
		Y += Row;
	}
	if (Segments.Num() > Window)
	{
		// NO SILENT WINDOWING, same rule as the diagnostics list.
		PanelLabel(Canvas, Ox + 4.f, Y,
			FString::Printf(TEXT("   %d of %d"), Last - FMath::Max(0, First), Segments.Num()),
			PanelDim);
		Y += Row;
	}

	if (SelectedSegment < 0 || SelectedSegment >= Segments.Num()) { return; }
	const FTUTrackSegment& Seg = Segments[SelectedSegment];
	Y += 8.f;

	// ---- The fields, per kind. EditConditionHides, reimplemented — and the
	// visibility rule lives in the tested model rather than being asked again
	// here, or the two would drift the first time a kind gained a field.
	const EEditKind Kind = KindOf(Seg.Kind);
	for (std::size_t f = 0; f < static_cast<std::size_t>(EEditField::Count); ++f)
	{
		const EEditField F = static_cast<EEditField>(f);
		if (!KindUsesField(Kind, F)) { continue; }
		if (F == EEditField::ZoneSpeed && Seg.Zone == ETUSegmentZone::None) { continue; }
		if (F == EEditField::StartsNewDevice && Seg.Zone == ETUSegmentZone::None) { continue; }

		EditorRowRects.Add(FVector4(Ox, Y, Ox + W, Y + Row));
		EditorRowField.Add(static_cast<int32>(f));

		const bool bFocus = FocusedField == F;
		if (bFocus) { PanelTile(Canvas, Ox - 4.f, Y - 1.f, W + 8.f, Row, PanelRule); }

		// A CHOICE IS NOT A NUMBER, and that is why these two rows were skipped
		// rather than merely unfinished: a device kind is an enumeration and
		// "starts a new device" is a flag, and neither has anything to type into
		// a numeric field. The panel could not show them, so changing a brake to
		// a block brake meant leaving play for the Details panel — which for the
		// one edit somebody makes most often is the wrong place to send them.
		//
		// CLICK CYCLES. That is not the direct manipulation constraint 1 rules
		// out: nothing is dragged, nothing is placed, and it is the same discrete
		// pick the Details panel's dropdown already offers — only reachable
		// without leaving the ride you are looking at.
		const bool bChoice = (F == EEditField::ZoneKind || F == EEditField::StartsNewDevice);
		FString Value;
		if (F == EEditField::ZoneKind)
		{
			Value = Seg.Zone == ETUSegmentZone::None
				? FString(TEXT("none")) : FString(ZoneKindName(Seg.Zone));
		}
		else if (F == EEditField::StartsNewDevice)
		{
			Value = Seg.bStartsNewDevice ? TEXT("yes") : TEXT("no");
		}
		else
		{
			// THE VALUE, AND A CARET WHILE TYPING. What is shown mid-edit is the
			// buffer rather than the stored number, because showing the stored one
			// would make typing look like it was doing nothing.
			Value = bFocus
				? FieldBuffer + TEXT("_")
				: FString::Printf(TEXT("%.4g"), ReadField(Seg, F));
		}

		// UNITS ARE ALWAYS SHOWN WHERE THERE IS ONE. Turns is a count and has
		// none, which is a distinction rather than a gap.
		PanelLabel(Canvas, Ox + 4.f, Y, UTF8_TO_TCHAR(FieldName(F)), PanelDim);
		PanelLabel(Canvas, Ox + 150.f, Y,
			bChoice ? FString::Printf(TEXT("%s  <click"), *Value)
			        : FString::Printf(TEXT("%s %s"), *Value, UTF8_TO_TCHAR(FieldUnit(F))),
			bFocus ? PanelCyan : (bChoice ? PanelAmber : PanelText));
		Y += Row;
	}

	// ---- The tooltip for whatever is focused, because a numeric box with no
	// context is a box somebody types 1000 into to see what happens.
	if (FocusedField != EEditField::Count)
	{
		const FFieldHelp Help = HelpFor(FocusedField);
		Y += 6.f;
		PanelLabel(Canvas, Ox, Y, UTF8_TO_TCHAR(Help.Tooltip), PanelDim);
		if (Help.bHasRange)
		{
			PanelLabel(Canvas, Ox, Y + Row,
				FString::Printf(TEXT("typically %.4g to %.4g %s   ·  Enter to apply, Esc to cancel"),
					Help.TypicalMin, Help.TypicalMax, UTF8_TO_TCHAR(FieldUnit(FocusedField))),
				PanelDim);
		}
	}
}

void ATUCoasterRide::ClickSegmentEditor()
{
	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC || !bShowSegmentEditor) { return; }
	float Mx = 0.f, My = 0.f;
	if (!PC->GetMousePosition(Mx, My)) { return; }

	for (int32 i = 0; i < EditorRowRects.Num() && i < EditorRowField.Num(); ++i)
	{
		const FVector4& R = EditorRowRects[i];
		if (Mx < R.X || Mx > R.Z || My < R.Y || My > R.W) { continue; }

		const int32 Action = EditorRowField[i];
		if (Action <= -1000)
		{
			// SELECTING A DIFFERENT SEGMENT BREAKS THE EDIT RUN, or typing here,
			// clicking away and coming back would be ONE undo step covering both.
			CancelField();
			SelectedSegment = -1000 - Action;
			FrameSelectedSegment();
		}
		else if (Session.EditsAllowed())
		{
			const EEditField F = static_cast<EEditField>(Action);

			// A CHOICE CYCLES ON CLICK rather than taking focus, because there is
			// nothing to type into it. Committed immediately — unlike a number,
			// which waits for Enter because "3" on the way to "30" is a rebuild
			// nobody asked for. A pick has no half-typed state to protect.
			if (F == EEditField::ZoneKind || F == EEditField::StartsNewDevice)
			{
				CancelField();
				FTUTrackSegment& S = Segments[SelectedSegment];
				if (F == EEditField::StartsNewDevice)
				{
					S.bStartsNewDevice = !S.bStartsNewDevice;
				}
				else
				{
					// Wraps through every zone including None, so the way to
					// remove a device is the same gesture as adding one.
					const uint8 Count = static_cast<uint8>(ETUSegmentZone::StationLoad) + 1;
					S.Zone = static_cast<ETUSegmentZone>((static_cast<uint8>(S.Zone) + 1) % Count);

					// LEAVING A DEVICE CLEARS ITS FLAG. bStartsNewDevice on an
					// unpowered segment is a setting with nothing to act on, and
					// it would come back the moment a zone was set again — a
					// value somebody turned on for a device they have since
					// deleted, silently applying to its replacement.
					if (S.Zone == ETUSegmentZone::None) { S.bStartsNewDevice = false; }
				}
				RebuildFromSegments();
				return;
			}

			// Focusing a field starts EMPTY rather than pre-filled with the
			// current value. Pre-filling means the first keystroke has to be
			// select-all or the number becomes "3020" — and there is no
			// select-all here, so starting empty is the honest shape.
			FocusedField = F;
			FieldBuffer.Empty();
		}
		return;
	}
	// A click outside every row cancels rather than committing, because a
	// half-typed number applied because somebody looked elsewhere is worse than
	// one lost.
	CancelField();
}

void ATUCoasterRide::KeyBackspace()
{
	// EDITING WINS WHILE EDITING, and the operator's stop wins the rest of the
	// time. Binding both to Backspace would fire both — and an operator typing a
	// radius would E-stop the ride, which is not a UI wart but the worst kind of
	// surprise this project can produce.
	if (IsTypingInField()) { TypeBackspace(); }
	else                   { PressEmergencyStop(); }
}

void ATUCoasterRide::KeyPeriod()
{
	if (IsTypingInField()) { TypePoint(); }
	else                   { StepOneScan(); }
}

void ATUCoasterRide::TypeDigit(int32 D)
{
	if (FocusedField == EEditField::Count) { return; }
	FieldBuffer.AppendChar(static_cast<TCHAR>('0' + FMath::Clamp(D, 0, 9)));
}

void ATUCoasterRide::TypePoint()
{
	// ONE decimal point. A second one makes a string no parser accepts, and
	// silently dropping it is what every numeric field has always done.
	if (FocusedField == EEditField::Count || FieldBuffer.Contains(TEXT("."))) { return; }
	FieldBuffer.AppendChar('.');
}

void ATUCoasterRide::TypeMinus()
{
	// A minus is only a minus at the FRONT. Anywhere else it is a typo, and a
	// field that accepted "3-0" would produce a number nobody meant.
	if (FocusedField == EEditField::Count || FieldBuffer.Len() > 0) { return; }
	FieldBuffer.AppendChar('-');
}

void ATUCoasterRide::TypeBackspace()
{
	if (FocusedField == EEditField::Count || FieldBuffer.IsEmpty()) { return; }
	FieldBuffer.LeftChopInline(1);
}

void ATUCoasterRide::CommitField()
{
	if (FocusedField == EEditField::Count || !Session.EditsAllowed()
		|| SelectedSegment < 0 || SelectedSegment >= Segments.Num())
	{
		CancelField();
		return;
	}
	// AN EMPTY BUFFER IS A CANCEL, not a zero. Somebody who clicked a field and
	// pressed Enter meant "never mind", and writing 0 into a radius because of it
	// would be the most destructive possible reading.
	if (FieldBuffer.IsEmpty() || FieldBuffer == TEXT("-") || FieldBuffer == TEXT("."))
	{
		CancelField();
		return;
	}

	const double V = FCString::Atod(*FieldBuffer);
	WriteField(Segments[SelectedSegment], FocusedField, V);

	// COMMITTED ON ENTER, NEVER PER KEYSTROKE. "3" on the way to "30" is a
	// segment 3 m long and a rebuild nobody asked for — and on a closed circuit
	// it is a rebuild that briefly reports the track as not closing.
	FocusedField = EEditField::Count;
	FieldBuffer.Empty();
	RebuildFromSegments();
}

void ATUCoasterRide::CancelField()
{
	FocusedField = EEditField::Count;
	FieldBuffer.Empty();
}

void ATUCoasterRide::DrawMainMenu(UCanvas* Canvas)
{
	if (!Canvas || !GEngine || Session.Mode() != EAppMode::MainMenu) { return; }

	MenuRowRects.Reset();
	MenuRowAction.Reset();

	const float W = 620.f;
	const float Row = 20.f;
	const float Ox = 60.f;
	float Y = 80.f;

	PanelTile(Canvas, Ox - 20.f, 50.f, W + 40.f, 460.f, PanelGround);
	PanelLabel(Canvas, Ox, 56.f, TEXT("TRACKUNLIMITED"), PanelCyan);
	// AN OSS PROJECT'S MENU IS FREE ADVERTISING FOR CONTRIBUTION, and a version
	// string is the first thing anybody filing a bug is asked for.
	PanelLabel(Canvas, Ox + 200.f, 58.f,
		TEXT("free and open source  ·  github.com/zebadrabbit/TrackUnlimited"), PanelDim);

	// ---- START FROM A TEMPLATE.
	//
	// NOT AN EMPTY LIST. The first edit should be changing a number on something
	// that already runs, not authoring geometry from nothing — a completely
	// different and much harder first task.
	PanelLabel(Canvas, Ox, Y, TEXT("START"), PanelDim);
	Y += Row;
	for (std::size_t i = 0; i < NumTemplates(); ++i)
	{
		const FTemplate T = TemplateAt(i);
		MenuRowRects.Add(FVector4(Ox, Y, Ox + W, Y + Row));
		MenuRowAction.Add(static_cast<int32>(i));
		PanelLabel(Canvas, Ox + 12.f, Y, UTF8_TO_TCHAR(T.Name), PanelText);
		PanelLabel(Canvas, Ox + 230.f, Y + 2.f, UTF8_TO_TCHAR(T.Description), PanelDim);
		Y += Row;
	}

	// ---- RECENT.
	Y += 10.f;
	PanelLabel(Canvas, Ox, Y, TEXT("RECENT"), PanelDim);
	Y += Row;

	const std::vector<FTrackEntry> Rows =
		FTrackBrowser::Rows(std::vector<FTrackEntry>(), Browser.RecentList());
	if (Rows.empty())
	{
		PanelLabel(Canvas, Ox + 12.f, Y,
			UTF8_TO_TCHAR(EmptyStateFor(EPanelKind::RecentTracks)), PanelDim);
		Y += Row;
	}
	for (std::size_t i = 0; i < Rows.size(); ++i)
	{
		const FTrackEntry& E = Rows[i];
		MenuRowRects.Add(FVector4(Ox, Y, Ox + W, Y + Row));
		MenuRowAction.Add(-1000 - static_cast<int32>(i));

		// A MISSING FILE IS STILL LISTED AND STILL CLICKABLE, because the
		// commonest cause is an unplugged drive and "reconnect it and click
		// again" only works if it is still there to click. It is dimmed and it
		// says which kind of problem it is — one is "plug the drive back in" and
		// the other is "line 12 is wrong".
		PanelLabel(Canvas, Ox + 12.f, Y, UTF8_TO_TCHAR(E.Name.c_str()),
			E.IsUsable() ? PanelText : PanelAmber);
		PanelLabel(Canvas, Ox + 230.f, Y + 2.f,
			UTF8_TO_TCHAR(FTrackBrowser::Subtitle(E).c_str()), PanelDim);
		Y += Row;
	}

	// ---- OPEN, and quit.
	Y += 10.f;
	MenuRowRects.Add(FVector4(Ox, Y, Ox + W, Y + Row));
	MenuRowAction.Add(-1);
	PanelLabel(Canvas, Ox + 12.f, Y, TEXT("Open a track file..."), PanelText);
	Y += Row + 8.f;
	PanelLabel(Canvas, Ox, Y,
		TEXT("click to choose   ·   [Tab] once a track is open"), PanelDim);
}

void ATUCoasterRide::StartFromTemplate(int32 Index)
{
	if (Index < 0 || static_cast<std::size_t>(Index) >= NumTemplates()) { return; }
	const FTemplate T = TemplateAt(static_cast<std::size_t>(Index));

	// A TEMPLATE NAMES A PRESET rather than carrying its own geometry. Five
	// measured worked examples already ship, and a parallel set of starter
	// layouts would be a second set of tracks to keep working — drifting from the
	// ones every number in the docs is quoted from.
	switch (T.Preset)
	{
	case ETemplatePreset::FlatRig:         Preset = ETUPresetLayout::FlatRig; break;
	case ETemplatePreset::OutAndBack:      Preset = ETUPresetLayout::OutAndBack; break;
	case ETemplatePreset::TwoTrainCircuit: Preset = ETUPresetLayout::TwoTrainCircuit; break;
	case ETemplatePreset::SmallBatch:      Preset = ETUPresetLayout::SmallBatch; break;
	case ETemplatePreset::Blank:           Segments.Reset(); break;
	default:                               Preset = ETUPresetLayout::Reference; break;
	}
	if (T.Preset != ETemplatePreset::Blank)
	{
		Segments = PresetLayout(Preset);
		ApplyPresetTrainSetup(Preset);
	}

	Session.DidCreateNew(std::string());
	Session.Enter(EAppMode::Build);
	CameraMode = ETUCameraMode::Orbit;
	RebuildFromSegments();
	FrameWholeTrack();

	// WHAT TO TRY FIRST, next to the thing it describes. Not a tutorial sequence
	// — that is a thing to dismiss, and this project's argument is that the tool
	// tells you the truth plainly.
	UE_LOG(LogTUEvents, Log, TEXT("%s — %s"),
		UTF8_TO_TCHAR(T.Name), UTF8_TO_TCHAR(T.WhatToTryFirst));
}

void ATUCoasterRide::ClickPrimary()
{
	// The menu owns the click while it is up; everything else is the editor's.
	if (Session.Mode() == EAppMode::MainMenu) { ClickMainMenu(); return; }
	// The editor gets first refusal because it is the panel somebody is working
	// in; a click that misses every one of its rows falls through to the
	// diagnostics list, which is the other clickable thing on screen.
	const int32 Before = SelectedSegment;
	ClickSegmentEditor();
	if (SelectedSegment == Before && FocusedField == EEditField::Count)
	{
		ClickDiagnostics();
	}
}

void ATUCoasterRide::ClickMainMenu()
{
	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC || Session.Mode() != EAppMode::MainMenu) { return; }

	float Mx = 0.f, My = 0.f;
	if (!PC->GetMousePosition(Mx, My)) { return; }

	for (int32 i = 0; i < MenuRowRects.Num() && i < MenuRowAction.Num(); ++i)
	{
		const FVector4& R = MenuRowRects[i];
		if (Mx < R.X || Mx > R.Z || My < R.Y || My > R.W) { continue; }

		const int32 Action = MenuRowAction[i];
		if (Action >= 0)
		{
			StartFromTemplate(Action);
		}
		else if (Action == -1)
		{
			// THE OS FILE DIALOG IS EDITOR-ONLY, and that is a real limit rather
			// than an oversight: IDesktopPlatform does not ship in a packaged
			// game. The browser list above is the primary path precisely because
			// it works everywhere, and this is the convenience on top of it.
			//
			// A packaged build needs a different picker, and pretending otherwise
			// by calling this unguarded would compile in the editor and fail to
			// link in the thing anybody actually downloads.
#if WITH_EDITOR
			UE_LOG(LogTUEvents, Log, TEXT("open: needs the desktop file dialog"));
#else
			UE_LOG(LogTUEvents, Warning,
				TEXT("open: a packaged build has no OS file dialog yet; use the recent list"));
#endif
		}
		else
		{
			const int32 Which = -1000 - Action;
			if (Which >= 0 && Which < static_cast<int32>(Browser.NumRecent()))
			{
				const std::string& Path = Browser.RecentAt(static_cast<std::size_t>(Which));
				UE_LOG(LogTUEvents, Log, TEXT("open recent: %s"), UTF8_TO_TCHAR(Path.c_str()));
			}
		}
		return;
	}
}

void ATUCoasterRide::FrameStation()
{
	// WHERE AN OPERATOR STANDS. Chase follows a train around the ride, which is
	// the wrong view for a console: the question in Operate is "may this train
	// go", and that is asked and answered at the platform.
	//
	// The station is the first holding zone, which the block walk already
	// derived — nothing here is authored or searched for.
	double Start = 0.0, End = 0.0;
	bool bFound = false;
	for (int32 i = 0; i < Segments.Num(); ++i)
	{
		const double L = static_cast<double>(Segments[i].Length);
		if (!bFound && (Segments[i].Zone == ETUSegmentZone::Station
			|| Segments[i].Zone == ETUSegmentZone::StationLoad
			|| Segments[i].Zone == ETUSegmentZone::StationUnload))
		{
			Start = End;
			bFound = true;
		}
		End += L;
		if (bFound && End - Start > 30.0) { break; }
	}
	if (!bFound)
	{
		// A ride with no platform is a test rig, and framing the whole thing is
		// the honest answer rather than picking an arbitrary metre.
		FrameWholeTrack();
		return;
	}

	FCamBounds B;
	const int Steps = 10;
	for (int i = 0; i <= Steps; ++i)
	{
		const FTrackFrame F = Track.EvaluateAt(Start + (End - Start) * (double(i) / Steps));
		B.Add({F.Position.X, F.Position.Y, F.Position.Z});
	}
	// Stand back far enough to see the train AND the block ahead of it, because
	// the dispatch decision is about both.
	const double Pad = 25.0;
	B.Add({B.Min.X - Pad, B.Min.Y - Pad, B.Min.Z - 2.0});
	B.Add({B.Max.X + Pad, B.Max.Y + Pad, B.Max.Z + Pad});
	Orbit.Frame(B, Camera ? static_cast<double>(Camera->FieldOfView) : 90.0, 16.0 / 9.0);
	bOrbitFramed = true;
}

void ATUCoasterRide::CycleAppMode()
{
	// BUILD to OPERATE to RIDE and back. Never asks, because nothing is
	// discarded — the document is still open and still in memory, and a shell
	// that asked here would train people to click straight through the dialog
	// that matters. The session's own rules say so and are tested.
	const EAppMode Now = Session.Mode();
	EAppMode Want = EAppMode::Build;
	if (Now == EAppMode::Build)        { Want = EAppMode::Operate; }
	else if (Now == EAppMode::Operate) { Want = EAppMode::Ride; }

	if (!Session.Enter(Want))
	{
		const char* Why = Session.WhyNotEnter(Want);
		UE_LOG(LogTUEvents, Warning, TEXT("mode refused: %s"),
			Why ? UTF8_TO_TCHAR(Why) : TEXT("unknown"));
		return;
	}

	// THE MODE DECIDES THE VIEW, which is what makes it a mode rather than a
	// label. Operate is the operator's console and the ride seen from outside;
	// Ride is the seat; Build is the thing you are editing, framed.
	switch (Want)
	{
	case EAppMode::Operate:
		PanelView = ETUPanelView::Operator;
		// THE STATION, not a chase. Chase follows a train around the ride; the
		// question in Operate is "may this train go", and that is asked and
		// answered at the platform.
		CameraMode = ETUCameraMode::Orbit;
		FrameStation();
		break;
	case EAppMode::Ride:
		PanelView = ETUPanelView::Off;
		CameraMode = ETUCameraMode::Rider;
		break;
	default:
		PanelView = ETUPanelView::Off;
		CameraMode = ETUCameraMode::Orbit;
		break;
	}
}


void ATUCoasterRide::DrawModeBanner(UCanvas* Canvas)
{
	if (!Canvas || !GEngine) { return; }

	// SMALL AND ALWAYS THERE. Not a title screen — the mode is a fact about what
	// your keys do, and a person who cannot see it will press something and be
	// surprised. `UI_CONVENTIONS`: hue is never the only channel, so the mode is
	// spelled out rather than being a coloured border.
	const TCHAR* Name = UTF8_TO_TCHAR(AppModeName(Session.Mode()));
	const FLinearColor Ink = Session.EditsAllowed() ? PanelCyan : PanelAmber;

	FString Line = FString::Printf(TEXT("  %s   [Tab] mode"), Name);
	if (!Session.EditsAllowed())
	{
		// AND IT SAYS WHY THE EDITOR IS QUIET, because a panel that has simply
		// stopped accepting numbers with no explanation is indistinguishable from
		// a broken one.
		Line += TEXT("   editing is off while the ride is running");
	}
	if (Session.IsDirty()) { Line += TEXT("   *unsaved"); }

	// THE CLOCK IS ALWAYS STATED WHEN IT IS NOT 1x. A ride running at quarter
	// speed with nothing saying so is a ride somebody will report as too slow —
	// and a paused one is a ride they will report as broken.
	if (bSimPaused)
	{
		Line += TEXT("   PAUSED  [.] step  [Pause] run");
	}
	else if (!FMath::IsNearlyEqual(TimeScale, 1.f))
	{
		Line += FString::Printf(TEXT("   %.2fx  [,] slower  [/] faster"), TimeScale);
	}

	PanelTile(Canvas, 10.f, 6.f, 8.f + Line.Len() * 6.2f, 18.f, PanelGround);
	PanelLabel(Canvas, 12.f, 8.f, Line, Ink);
}

void ATUCoasterRide::ClickDiagnostics()
{
	// A FINDING WITHOUT A PLACE IS TRIVIA — the diagnostics model's rule, and this
	// is the half that makes it true. "Curvature implying a radius under 2 m" is
	// only useful if it takes you there.
	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC || !bShowDiagnostics) { return; }

	float Mx = 0.f, My = 0.f;
	if (!PC->GetMousePosition(Mx, My)) { return; }

	for (int32 i = 0; i < DiagRowRects.Num(); ++i)
	{
		const FVector4& R = DiagRowRects[i];
		if (Mx < R.X || Mx > R.Z || My < R.Y || My > R.W) { continue; }
		if (i >= static_cast<int32>(Diagnostics.Num())) { break; }

		const FDiagRow& Row = Diagnostics.At(static_cast<std::size_t>(i));
		if (Row.Target.Segment >= 0)
		{
			SelectedSegment = Row.Target.Segment;
			FrameSelectedSegment();
		}
		else if (Row.Target.S >= 0.0)
		{
			// A finding located by ARC LENGTH rather than by segment — anything
			// derived rather than authored. Frame the point itself; there is no
			// segment to select, and pretending there is would select the wrong
			// one whenever the walk and the authored list disagree.
			const FTrackFrame F = Track.EvaluateAt(Row.Target.S);
			FCamBounds B;
			B.Add({F.Position.X - 8.0, F.Position.Y - 8.0, F.Position.Z - 8.0});
			B.Add({F.Position.X + 8.0, F.Position.Y + 8.0, F.Position.Z + 8.0});
			Orbit.Frame(B, Camera ? static_cast<double>(Camera->FieldOfView) : 90.0, 16.0 / 9.0);
			bOrbitFramed = true;
			CameraMode = ETUCameraMode::Orbit;
		}
		return;
	}
}

void ATUCoasterRide::PollMovementKeys()
{
	MoveForward = MoveRight = MoveUp = 0.f;

	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC)
	{
		return;
	}

	// TYPING A NUMBER IS NOT FLYING. The segment editor takes digits, and A, S, D,
	// E and W are all one keystroke away from a field somebody is working in —
	// but more to the point, a camera that drifted while somebody typed a radius
	// would move the thing they are looking at out from under them.
	if (IsTypingInField())
	{
		return;
	}

	auto Down = [PC](const FKey& K) { return PC->IsInputKeyDown(K) ? 1.f : 0.f; };
	MoveForward = Down(EKeys::W) - Down(EKeys::S);
	MoveRight = Down(EKeys::D) - Down(EKeys::A);
	MoveUp = Down(EKeys::E) - Down(EKeys::Q);
}

void ATUCoasterRide::ApplyCursorMode()
{
	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC) { return; }

	// RIDE IS THE ONLY MODE WITH NOTHING TO CLICK. Everywhere else has rows, and
	// a captured mouse made every one of them keyboard-only.
	const bool bWant = Session.Mode() != EAppMode::Ride;

	// Applied only on a CHANGE. SetInputMode every frame resets Slate's capture
	// state, which shows up as a cursor that stutters and a drag that drops.
	if (bWant == bCursorShown && PC->bShowMouseCursor == bWant) { return; }
	bCursorShown = bWant;
	PC->bShowMouseCursor = bWant;

	if (bWant)
	{
		// GameAndUI rather than UIOnly: the key bindings have to keep working
		// while the cursor is free, because every one of them is still the
		// fastest way to do the thing it does.
		FInputModeGameAndUI Mode;
		Mode.SetHideCursorDuringCapture(false);
		PC->SetInputMode(Mode);
	}
	else
	{
		PC->SetInputMode(FInputModeGameOnly());
	}
}

void ATUCoasterRide::StepSelection(int32 Delta)
{
	if (Segments.Num() == 0) { return; }

	// NOT WHILE TYPING. A bracket is not a digit, so nothing would be corrupted —
	// but moving the selection out from under a half-entered number and then
	// applying it on Enter would write it to the wrong segment, which is the one
	// outcome this panel must never produce.
	if (IsTypingInField()) { return; }

	// WRAPS, because the layouts are rings as often as not and a selection that
	// stopped dead at segment 0 would make walking backwards past the station
	// impossible on exactly the presets where it is most wanted.
	const int32 N = Segments.Num();
	SelectedSegment = SelectedSegment < 0
		? (Delta > 0 ? 0 : N - 1)
		: ((SelectedSegment + Delta) % N + N) % N;

	if (CameraMode == ETUCameraMode::Orbit)
	{
		FrameSelectedSegment();
	}
}

void ATUCoasterRide::FrameSelectedSegment()
{
	// NOTHING SELECTED FALLS BACK TO THE WHOLE TRACK rather than doing nothing. A
	// key that silently did nothing reads as broken, and "show me everything" is
	// the answer somebody pressing it almost certainly wanted anyway.
	if (SelectedSegment < 0 || SelectedSegment >= Segments.Num())
	{
		FrameWholeTrack();
		return;
	}

	// Walk to the selected segment's own span and frame only that. The walk is
	// the same one the mesher uses, so nothing extra is integrated.
	double Start = 0.0;
	for (int32 i = 0; i < SelectedSegment; ++i)
	{
		Start += static_cast<double>(Segments[i].Length);
	}
	const double End = Start + static_cast<double>(Segments[SelectedSegment].Length);

	// Sampled directly across the span rather than walked: a handful of EvaluateAt
	// calls on a BOUNDED range, where walking the whole track to find one segment
	// would be the O(track length) mistake this project keeps having to unlearn.
	FCamBounds B;
	const int Steps = 12;
	for (int i = 0; i <= Steps; ++i)
	{
		const double S = Start + (End - Start) * (static_cast<double>(i) / Steps);
		const FTrackFrame F = Track.EvaluateAt(S);
		B.Add({F.Position.X, F.Position.Y, F.Position.Z});
	}
	// A dead-straight segment is a line with no thickness, and framing a line
	// puts the camera on top of it. Pad by the track's own swept width so there
	// is always something to look at.
	const double Pad = TrackWidth(Profile) * 2.0;
	B.Add({B.Min.X - Pad, B.Min.Y - Pad, B.Min.Z - Pad});
	B.Add({B.Max.X + Pad, B.Max.Y + Pad, B.Max.Z + Pad});

	Orbit.Frame(B, Camera ? static_cast<double>(Camera->FieldOfView) : 90.0, 16.0 / 9.0);
	bOrbitFramed = true;
	CameraMode = ETUCameraMode::Orbit;

	UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: framed segment %d (%.1f-%.1f m)"),
		SelectedSegment, Start, End);
}

void ATUCoasterRide::CycleProfileChannel()
{
	switch (ProfileChannel)
	{
	case ETUProfileChannel::VerticalG: ProfileChannel = ETUProfileChannel::LateralG; break;
	case ETUProfileChannel::LateralG:  ProfileChannel = ETUProfileChannel::Speed; break;
	case ETUProfileChannel::Speed:     ProfileChannel = ETUProfileChannel::RollRate; break;
	default:                           ProfileChannel = ETUProfileChannel::VerticalG; break;
	}
	// Showing the graph is implied by asking to change its channel. A key that
	// silently changed something invisible would be a key nobody trusts.
	bShowProfileGraph = true;
}

void ATUCoasterRide::FrameWholeTrack()
{
	// Walked rather than guessed: the frames the mesher already produces, swept
	// into a bounding box. Nothing extra is integrated for this.
	FCamBounds B;
	for (const FTrackFrame& F : WalkTrack(Track, 2.0))
	{
		B.Add({F.Position.X, F.Position.Y, F.Position.Z});
	}
	if (!B.IsValid()) { return; }

	// FOV from the camera itself, aspect from the viewport, because framing that
	// checked one axis puts a long low layout off both sides of an ultrawide —
	// and this project's layouts are all long and low.
	double Aspect = 16.0 / 9.0;
	if (GEngine && GEngine->GameViewport)
	{
		const FVector2D Size = GEngine->GameViewport->Viewport->GetSizeXY();
		if (Size.Y > 0.f) { Aspect = static_cast<double>(Size.X / Size.Y); }
	}
	Orbit.Frame(B, Camera ? static_cast<double>(Camera->FieldOfView) : 90.0, Aspect);
	bOrbitFramed = true;

	UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: framed %.0f m of track from %.0f m"),
		Track.TotalLength(), Orbit.Distance);
}

void ATUCoasterRide::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// This is the live preview. OnConstruction runs when the actor is placed,
	// when the level loads, when the actor is moved, and after any property
	// change — which is exactly the set of moments the drawn track is stale.
	RebuildFromSegments();

#if WITH_EDITOR
	// Editor worlds only. In PIE, BeginPlay does the drawing, and doing it here
	// as well would just draw the same lines twice.
	//
	// ponytail: flushes ALL persistent debug lines in the world, so two of these
	// actors in one level would erase each other's preview. Fine while the
	// vertical slice is the only thing in the level; the real fix is a line
	// batcher component per actor, or the Phase 4 track mesh making this whole
	// function unnecessary.
	if (UWorld* World = GetWorld())
	{
		if (!World->IsGameWorld())
		{
			FlushPersistentDebugLines(World);
			if (bDrawTrack)
			{
				DrawTrack();
			}
			DrawRideProfile();
		}
	}
#endif
}

void ATUCoasterRide::BeginPlay()
{
	Super::BeginPlay();
	RebuildFromSegments();

	// THE CABINET GETS POWER, and the ride opens the way a real one does: an
	// operator walks the course, declares it clear, and turns the key.
	//
	// Done here rather than left to the player because every measured figure in
	// this project was taken with the ride simply running, and a default that
	// needed a keypress before anything moved would silently invalidate all of
	// them. The sequence is real; performing it automatically at open is the
	// stand-in, exactly as FAutoStationCrew stands in for platform staff.
	Plc.PowerOn();
	Plc.DeclareCourseClear();
	SetPlcMode(EPlcMode::Run);

	if (bDrawTrack)
	{
		DrawTrack();
	}
	DrawRideProfile();

	// The debug canvas rather than a UMG widget, deliberately: a generated panel
	// has no fixed set of elements to lay out in an asset, and this needs no asset
	// at all. It draws in play and not in the editor viewport, which is right — a
	// control room is something you look at while the ride is running.
	PanelDrawHandle = UDebugDrawService::Register(TEXT("Game"),
		FDebugDrawDelegate::CreateUObject(this, &ATUCoasterRide::DrawControlPanel));
}

void ATUCoasterRide::EndPlay(const EEndPlayReason::Type Reason)
{
	// Unregistered explicitly: the service holds the delegate, and a stale one
	// fires into a destroyed actor.
	if (PanelDrawHandle.IsValid())
	{
		UDebugDrawService::Unregister(PanelDrawHandle);
		PanelDrawHandle.Reset();
	}
	Super::EndPlay(Reason);
}


// ===================== THE RIDE PROFILE, AS A GRAPH =====================
//
// The view that makes a spike DIAGNOSABLE. The in-world traces show WHERE
// something happens; this shows WHAT, against a labelled axis you can read a
// number off — and they are different questions, which is why both exist.
//
// Drawn on the debug canvas, exactly as the control panel is, and for the same
// reason: it needs no asset, so it works the moment somebody presses the key
// rather than after somebody authors a widget. `UI_CONVENTIONS.md` picked UMG
// for composed panels and custom Slate for DRAWN ones, and this is a drawn one —
// the canvas is the same OnPaint in a cheaper coat.
//
// Every number comes from `GraphAxis.h`, which is tested: nice-number ticks, a
// range grown to a multiple of the step, zero kept on screen for signed
// channels, and a scrubber that round-trips.
// ===================== THE DIAGNOSTICS PANEL =====================
//
// `TrackValidate.h` has produced exactly the right data since Phase 0 and none of
// it has ever been visible outside a log. This is the log becoming a panel.
//
// The rules it draws by are in `Prototypes/Shell/DiagnosticsModel.h` and are
// tested there: severity then LOCATION, whole-ride statements last, height called
// out on its own row saying LOW or HIGH, and a stalled ride hiding every derived
// number.
//
// REPORT, NEVER REPAIR — IN THE UI TOO. There is no fix button and nowhere to put
// one: a row has a place to GO and no action to TAKE. Measured rather than
// principled — clamping a degenerate arc to a straight yields a plausible 1.00 g
// and a clean continuity pass, which is worse than visible breakage.
void ATUCoasterRide::BuildDiagnostics()
{
	Diagnostics.Clear();

	// WHAT THE DEVICES WILL DO, which the validator structurally cannot say.
	// It reads the authored values; these depend on the train's length and on
	// the speed at that point, which depends on every metre of track before it.
	// Computed during the rebuild where the zones and the profile both exist.
	for (const FDeviceFinding& D : LastDeviceFindings)
	{
		FDiagTarget T;
		T.S = D.S;
		Diagnostics.Add(D.bIsError ? EDiagSeverity::Error : EDiagSeverity::Warning,
			"Devices", D.What, T);
	}

	// The validator's own findings, verbatim. Its messages already name fields
	// and quantities, so rewording them here would be a second voice saying
	// nearly the same thing.
	for (const FTrackDiagnostic& D : LastDiagnostics)
	{
		FDiagTarget T;
		T.Segment = static_cast<int>(D.SegmentIndex);
		Diagnostics.Add(D.bIsError ? EDiagSeverity::Error : EDiagSeverity::Warning,
			"Geometry", D.Message, T);
	}

	// Closure, with HEIGHT on its own row. The vertical slice shipped 8.5 m low
	// because plan view looked closed, and one combined number hides exactly that.
	if (!bTrackIsCircuit)
	{
		const FTrackFrame A = Track.EvaluateAt(0.0);
		const FTrackFrame B = Track.EvaluateAt(Track.TotalLength());
		Diagnostics.AddClosure(B.Position.X - A.Position.X,
			B.Position.Y - A.Position.Y,
			B.Position.Z - A.Position.Z, 0.05);
	}

	// The ride itself, and a stall hides every number derived from a run that did
	// not happen.
	double Top = 0.0, PeakG = 0.0, PeakAt = 0.0, Lap = 0.0;
	for (const FRideSample& S : Profile_.Samples)
	{
		Top = FMath::Max(Top, S.Speed);
		if (FMath::Abs(S.VerticalG) > FMath::Abs(PeakG)) { PeakG = S.VerticalG; PeakAt = S.S; }
		Lap = S.Time;
	}
	Diagnostics.AddRideProfile(Profile_.bCompleted, Profile_.StalledAtS,
		Top, PeakG, PeakAt, Lap);

	// Support placement, which refuses more than it places on a layout with a
	// helix — and the hole its refusals leave is the number an engineer asks for.
	if (bShowSupportFindings)
	{
		std::vector<FMeshFinding> Ignored;
		const FSupportPlan Plan = PlanSupports(WalkTrack(Track, 1.0),
			Track.GetHeartlineHeight(), Profile, FSupportSettings(),
			FlatGround(-GroundOffsetM));
		for (const FSupportFinding& F : Plan.Finding)
		{
			FDiagTarget T;
			T.S = F.S;
			Diagnostics.Add(EDiagSeverity::Warning, "Structure",
				TCHAR_TO_UTF8(*FString::Printf(TEXT("%s (%.0f m of track)"),
					UTF8_TO_TCHAR(F.What.c_str()), F.LengthM())), T);
		}
		if (Plan.LongestGapM > 0.0)
		{
			Diagnostics.Add(EDiagSeverity::Info, "Structure",
				TCHAR_TO_UTF8(*FString::Printf(TEXT("%d supports, longest unsupported run %.1f m"),
					static_cast<int32>(Plan.Leg.size()), Plan.LongestGapM)));
		}
	}

	Diagnostics.Sort();
}

void ATUCoasterRide::DrawDiagnosticsPanel(UCanvas* Canvas)
{
	if (!bShowDiagnostics || !Canvas || !GEngine) { return; }

	const float W = 620.f;
	const float Row = 14.f;
	const int32 Shown = FMath::Min(static_cast<int32>(Diagnostics.Num()), 14);
	const float H = 26.f + Row * FMath::Max(Shown, 1);
	const float Ox = Canvas->SizeX - W - 20.f;
	const float Oy = 20.f;

	PanelTile(Canvas, Ox - 8.f, Oy - 8.f, W + 16.f, H + 16.f, PanelGround);
	PanelLabel(Canvas, Ox, Oy,
		FString::Printf(TEXT("DIAGNOSTICS   %s   [V] hide"),
			UTF8_TO_TCHAR(Diagnostics.Summary().c_str())),
		Diagnostics.HasErrors() ? PanelRed : PanelDim);

	if (Diagnostics.Num() == 0)
	{
		// Empty states say what to DO, not what is absent — FirstRun.h owns the
		// words so every panel's is written once and in the same voice.
		PanelLabel(Canvas, Ox, Oy + 22.f,
			UTF8_TO_TCHAR(EmptyStateFor(EPanelKind::Diagnostics)), PanelDim);
		return;
	}

	// IMMEDIATE MODE: the rectangles are recorded as they are drawn and hit-tested
	// next click. There is no widget tree to keep in sync with the findings, which
	// is the whole reason this is cheap enough to rebuild every rebuild.
	DiagRowRects.Reset();

	float Y = Oy + 20.f;
	for (int32 i = 0; i < Shown; ++i)
	{
		const FDiagRow& R = Diagnostics.At(static_cast<std::size_t>(i));
		DiagRowRects.Add(FVector4(Ox, Y, Ox + W, Y + Row));

		// The selected row is banded, so clicking one and then looking at the
		// viewport does not lose which finding you were on.
		if (R.Target.Segment >= 0 && R.Target.Segment == SelectedSegment)
		{
			PanelTile(Canvas, Ox - 4.f, Y - 1.f, W + 8.f, Row, PanelRule);
		}
		const FLinearColor Ink = R.Severity == EDiagSeverity::Error ? PanelRed
			: (R.Severity == EDiagSeverity::Warning ? PanelAmber : PanelDim);

		// HUE IS NEVER THE ONLY CHANNEL — UI_CONVENTIONS. A severity glyph carries
		// the same fact as the colour, so the panel still reads in grey-scale and
		// to somebody who does not separate red from amber.
		const TCHAR* Mark = R.Severity == EDiagSeverity::Error ? TEXT("!!")
			: (R.Severity == EDiagSeverity::Warning ? TEXT(" !") : TEXT("  "));

		// AND WHERE IT IS, because a finding without a place is trivia. Once
		// there is a selection to drive, this column becomes the click target.
		FString Where;
		if (R.Target.S >= 0.0)            { Where = FString::Printf(TEXT("%6.0fm"), R.Target.S); }
		else if (R.Target.Segment >= 0)   { Where = FString::Printf(TEXT("  seg%d"), R.Target.Segment); }
		else                              { Where = TEXT("      "); }

		PanelLabel(Canvas, Ox, Y, FString::Printf(TEXT("%s %s  %s"),
			Mark, *Where, UTF8_TO_TCHAR(R.Text.c_str())), Ink);
		Y += Row;
	}

	if (Diagnostics.Num() > static_cast<std::size_t>(Shown))
	{
		// NO SILENT TRUNCATION. A list that stopped at fourteen without saying so
		// reads as "that is all of them", which is the one thing a diagnostics
		// panel must never imply.
		PanelLabel(Canvas, Ox, Y,
			FString::Printf(TEXT("   ... and %d more"),
				static_cast<int32>(Diagnostics.Num()) - Shown), PanelDim);
	}
}


void ATUCoasterRide::DrawProfileGraph(UCanvas* Canvas)
{
	if (!bShowProfileGraph || !Canvas || !GEngine) { return; }

	const std::vector<FRideSample>& Sample = Profile_.Samples;
	const float Wx = 560.f;
	const float Hy = 150.f;
	const float Ox = 20.f;
	const float Oy = Canvas->SizeY - Hy - 96.f;

	PanelTile(Canvas, Ox - 8.f, Oy - 24.f, Wx + 16.f, Hy + 58.f, PanelGround);

	// A STALLED RIDE SHOWS THE STALL AND NOTHING ELSE. The envelope suite already
	// had this failure — "within envelope, zero findings" over a train that
	// stalled at 46 m — and a trace drawn from a run that did not happen reads as
	// a result. Same rule the diagnostics model runs on.
	if (!Profile_.bCompleted)
	{
		PanelLabel(Canvas, Ox, Oy - 20.f, TEXT("RIDE PROFILE"), PanelDim);
		PanelLabel(Canvas, Ox, Oy + 8.f,
			FString::Printf(TEXT("the train does not complete: it stalls at %.1f m, %.1f m up"),
				Profile_.StalledAtS, Profile_.StalledHeight), PanelRed);
		PanelLabel(Canvas, Ox, Oy + 26.f,
			TEXT("no speed or G is shown, because there was no ride to measure"), PanelDim);
		return;
	}
	if (Sample.size() < 2)
	{
		PanelLabel(Canvas, Ox, Oy - 20.f, TEXT("RIDE PROFILE"), PanelDim);
		PanelLabel(Canvas, Ox, Oy + 8.f,
			TEXT("no ride to measure yet"), PanelDim);
		return;
	}

	// The channel on show. One at a time rather than four overlaid, because four
	// traces on one axis is a picture rather than a reading — and the per-channel
	// scale is exactly what the Phase 1 legibility card asked for.
	std::vector<double> Values;
	Values.reserve(Sample.size());
	const TCHAR* ChannelName = TEXT("");
	const TCHAR* ChannelUnit = TEXT("");
	bool bSigned = true;
	FLinearColor Ink = PanelCyan;

	switch (ProfileChannel)
	{
	case ETUProfileChannel::Speed:
		for (const FRideSample& S : Sample) { Values.push_back(S.Speed * 3.6); }
		ChannelName = TEXT("SPEED"); ChannelUnit = TEXT("km/h");
		bSigned = false; Ink = PanelCyan;
		break;
	case ETUProfileChannel::LateralG:
		for (const FRideSample& S : Sample) { Values.push_back(S.LateralG); }
		ChannelName = TEXT("LATERAL G"); ChannelUnit = TEXT("g");
		Ink = PanelAmber;
		break;
	case ETUProfileChannel::RollRate:
		// FIRST-CLASS, WITH ITS OWN AXIS. The one thing no G trace can ever show:
		// felt G models the rider as a point at the heartline, so spinning that
		// point costs exactly nothing.
		for (const FRideSample& S : Sample) { Values.push_back(S.RollRateDegPerSec); }
		ChannelName = TEXT("ROLL RATE"); ChannelUnit = TEXT("deg/s");
		Ink = PanelGreen;
		break;
	default:
		for (const FRideSample& S : Sample) { Values.push_back(S.VerticalG); }
		ChannelName = TEXT("VERTICAL G"); ChannelUnit = TEXT("g");
		Ink = PanelCyan;
		break;
	}

	const double Total = Sample.back().S;
	double Lo = Values[0], Hi = Values[0];
	for (double V : Values) { Lo = FMath::Min(Lo, V); Hi = FMath::Max(Hi, V); }
	const FAxis Axis = MakeAxis(Lo, Hi, 5, bSigned);

	// ---- Gridlines, at the nice numbers, LABELLED. An unlabelled gridline is
	// decoration; the whole reason for snapping the step is that each one has a
	// number you can read.
	for (std::size_t t = 0; t < Axis.TickCount(); ++t)
	{
		const double V = Axis.TickAt(t);
		const float Y = Oy + Hy - static_cast<float>(Axis.Fraction(V)) * Hy;
		// ZERO IS DRAWN BRIGHTER, because on a signed channel it is the line the
		// trace means anything against.
		const bool bZero = FMath::Abs(V) < Axis.Step * 0.001;
		PanelTile(Canvas, Ox, Y, Wx, 1.f, bZero ? PanelRule : FLinearColor(
			PanelRule.R * 0.6f, PanelRule.G * 0.6f, PanelRule.B * 0.6f, 1.f));
		PanelLabel(Canvas, Ox + Wx + 4.f, Y - 7.f,
			FString::Printf(TEXT("%.2f"), V), PanelDim);
	}

	// ---- The trace. One line segment per sample pair, clamped into the box.
	for (std::size_t i = 0; i + 1 < Values.size(); ++i)
	{
		const float X0 = Ox + static_cast<float>(Sample[i].S / Total) * Wx;
		const float X1 = Ox + static_cast<float>(Sample[i + 1].S / Total) * Wx;
		const float Y0 = Oy + Hy - static_cast<float>(Axis.Fraction(Values[i])) * Hy;
		const float Y1 = Oy + Hy - static_cast<float>(Axis.Fraction(Values[i + 1])) * Hy;
		FCanvasLineItem Line(FVector2D(X0, Y0), FVector2D(X1, Y1));
		Line.SetColor(Ink);
		Line.LineThickness = 1.6f;
		Canvas->DrawItem(Line);
	}

	// ---- The scrubber, and the train's own position on it.
	//
	// THE TRAIN IS ALWAYS DRAWN, because the most useful thing this panel does is
	// let you watch the trace and the ride at the same time and see which bit of
	// the graph is the bit you are on.
	if (!Trains.IsEmpty() && Trains[0])
	{
		const double S = Trains[0]->GetDistance();
		const float X = Ox + static_cast<float>(FMath::Clamp(S / Total, 0.0, 1.0)) * Wx;
		PanelTile(Canvas, X, Oy, 1.f, Hy, PanelAmber);
		PanelLabel(Canvas, X + 3.f, Oy + Hy - 12.f,
			FString::Printf(TEXT("%.0f m"), S), PanelAmber);
	}

	// ---- The heading, the extremes, and WHERE they happened.
	//
	// "4.25 g at 310 m" is somewhere to go and look; "4.25 g" is trivia. Same rule
	// the diagnostics model runs on, and the reason both say it.
	const FChannelExtremes E = ExtremesOf(Values, Total);
	PanelLabel(Canvas, Ox, Oy - 20.f,
		FString::Printf(TEXT("%s  (%s)   [G] channel   [H] hide"), ChannelName, ChannelUnit),
		PanelDim);
	PanelLabel(Canvas, Ox + 240.f, Oy - 20.f,
		FString::Printf(TEXT("peak %.2f at %.0f m      min %.2f at %.0f m"),
			E.Max, E.MaxAtS, E.Min, E.MinAtS), PanelText);
	PanelLabel(Canvas, Ox, Oy + Hy + 6.f, TEXT("0 m"), PanelDim);
	PanelLabel(Canvas, Ox + Wx - 46.f, Oy + Hy + 6.f,
		FString::Printf(TEXT("%.0f m"), Total), PanelDim);
}


void ATUCoasterRide::DrawControlPanel(UCanvas* Canvas, APlayerController* /*PC*/)
{
	// THE GRAPH RIDES ON THE SAME CALLBACK rather than registering a second
	// delegate. Two registrations can be added, removed and ordered
	// independently, and the failure — one panel drawing over the other
	// depending on which was registered first — is the kind that only appears
	// on somebody else's machine.
	//
	// Drawn FIRST so the control panel, which is the operator's, is never
	// obscured by the author's graph.
	// THE MENU IS NOT AN OVERLAY, so [F2] leaves it alone — hiding it would strand
	// somebody on a screen whose only controls are the ones just hidden.
	DrawMainMenu(Canvas);
	if (bHideOverlays) { return; }

	DrawSegmentEditor(Canvas);
	DrawModeBanner(Canvas);
	DrawProfileGraph(Canvas);
	DrawDiagnosticsPanel(Canvas);

	if (PanelView == ETUPanelView::Off || !Canvas || !Signals || !Drives || !GEngine)
	{
		return;
	}

	// TWO VIEWS OF ONE PANEL, and the split is the one every real installation
	// already makes rather than a detail level invented here. The operator's
	// question is "may this train go, and if not, what is holding it"; the
	// maintainer's is "what is this machine actually doing". Motor current answers
	// only the second, which is why an operator console does not carry it and why
	// hiding it from one view is more faithful than showing it to both.
	//
	// Same walk, same live reads, one branch per section. Nothing is computed for
	// a view it is not shown in, and nothing is cached for either.
	const bool bMaint = PanelView == ETUPanelView::Maintenance;

	// GENERATED FROM THE SAME LISTS AS THE RIDE. Nothing below is authored per
	// layout: the rows come from walking the blocks, the zones and the platforms
	// that RebuildFromSegments already derived, so a track with more blocks gets
	// more indicators without anything here being told about it.
	const int32 NumBlocks = static_cast<int32>(Signals->NumBlocks());
	const int32 NumDrives = static_cast<int32>(Drives->Num());
	const float Row = 13.f;
	const float Pad = 8.f;
	const float W = 470.f;
	const float StripH = bMaint ? 46.f + Row : 46.f;   // schematic, + the counts row
	const int32 Rows = 2                           // title, status
		+ 1 + NumDrives                            // DRIVES heading + VFD modules
		+ (bMaint ? 2 : 0)                         // CONTROLLER + DETECTION
		+ (Platforms.Num() > 0 ? 1 + Platforms.Num() + 2 : 0)    // + CONSOLE heading, lamps
		+ (EventLog.Num() > 0 ? 1 + FMath::Min(EventLog.Num(), 4) : 0);
	// The console row sat on the bottom edge, half off it: the section gaps are
	// worth about a row and a half between them and were not being counted.
	const float H = Pad * 2.f + Rows * Row + StripH + 34.f;

	const float X = 16.f;
	const float Y = FMath::Max(16.f, Canvas->SizeY - H - 16.f);

	PanelTile(Canvas, X, Y, W, H, PanelGround);
	PanelTile(Canvas, X, Y, W, 1.f, PanelRule);
	PanelTile(Canvas, X, Y + H - 1.f, W, 1.f, PanelRule);
	PanelTile(Canvas, X, Y, 1.f, H, PanelRule);
	PanelTile(Canvas, X + W - 1.f, Y, 1.f, H, PanelRule);

	// THE COLOUR BANDS, which are the one convention every real console shares:
	// the operating controls live on a GREEN field and the stop authority on a RED
	// one, physically separated, so a hand reaching for one is nowhere near the
	// other. On a screen there is no hand to misplace, but the grouping still does
	// the work of saying which half of the panel you are reading — and it costs two
	// tiles.
	//
	// The third console adds YELLOW between them for lockout, which is not modelled
	// here and so is not drawn. A band for a control that does not exist would be
	// the panel telling its first lie.
	PanelTile(Canvas, X + 1.f, Y + 1.f, 3.f, H - 2.f, FLinearColor(0.16f, 0.55f, 0.24f, 1.f));
	PanelTile(Canvas, X + W - 4.f, Y + 1.f, 3.f, 34.f, FLinearColor(0.62f, 0.14f, 0.12f, 1.f));

	float Ty = Y + Pad;
	const float Lx = X + Pad + 4.f;   // clear of the green field band

	PanelLabel(Canvas, Lx, Ty, bMaint
		? TEXT("TRACKUNLIMITED  ·  MAINTENANCE")
		: TEXT("TRACKUNLIMITED  ·  RIDE CONTROL"), bMaint ? PanelAmber : PanelCyan);
	Ty += Row;

	// Top-level state, which is the one line an operator glances at.
	{
		const bool bStopped = Drives->IsEmergencyStopped();
		FString Status = FString::Printf(TEXT("%s   %d TRAIN%s"),
			bManualDispatch ? TEXT("MANUAL") : TEXT("AUTO"),
			Trains.Num(), Trains.Num() == 1 ? TEXT("") : TEXT("S"));
		if (bMaint)
		{
			// THE SCAN RATE IS AN ENGINEERING FACT, and overruns are the one thing
			// that makes it stop being a constant. A dropped backlog means the ride
			// ran slower than real time for a moment, which is safe and is still
			// something a maintainer wants to know happened.
			Status += FString::Printf(TEXT("   %d Hz SCAN"), SimHz);
			if (ScanOverruns > 0)
			{
				// THE SECONDS, NOT JUST THE COUNT. 545 overruns reads as "a bit
				// stuttery"; 54 s dropped says the ride on screen is not the ride
				// the model computed, and nothing watched across it can be judged.
				Status += FString::Printf(TEXT("   %d OVERRUN%s, %.1f s DROPPED"),
					ScanOverruns, ScanOverruns == 1 ? TEXT("") : TEXT("S"),
					ScanTimeDroppedS);
			}
			// SCAN NUMBER AND FINGERPRINT TOGETHER, never the digest alone. It is a
			// running hash, so two rides only agree AT THE SAME POINT — a digest
			// without its scan number is not a comparable quantity, it is a number
			// that happens to be printed.
			Status += FString::Printf(TEXT("   #%lld %08x"),
				static_cast<long long>(ScanNumber),
				static_cast<uint32>(SimFingerprint.Value() & 0xFFFFFFFFull));
		}
		if (Signals->Violations() > 0)
		{
			Status += FString::Printf(TEXT("   %d VIOLATION(S)"),
				static_cast<int32>(Signals->Violations()));
		}
		PanelLabel(Canvas, Lx, Ty, Status, Signals->Violations() > 0 ? PanelRed : PanelDim);

		// THE STOP AUTHORITY IN ITS OWN BOX, outlined red, away from everything
		// else. Straight off the console photo: the operating controls are grouped
		// inside a green outline and the stop controls inside a red one, so a hand
		// reaching for one is nowhere near the other. On a screen the outline is
		// the whole of that idea, and it is worth keeping.
		const float StopX = Lx + 292.f;
		const float StopW = W - Pad * 2.f - 292.f;
		PanelTile(Canvas, StopX, Ty - 2.f, StopW, 1.f, bStopped ? PanelRed : PanelDim);
		PanelTile(Canvas, StopX, Ty + Row, StopW, 1.f, bStopped ? PanelRed : PanelDim);
		PanelTile(Canvas, StopX, Ty - 2.f, 1.f, Row + 2.f, bStopped ? PanelRed : PanelDim);
		PanelTile(Canvas, StopX + StopW - 1.f, Ty - 2.f, 1.f, Row + 2.f,
			bStopped ? PanelRed : PanelDim);
		// THE CATEGORY IS PART OF THE STOP, so the panel says which one it is.
		// STOPPING and STOPPED are also different facts under a Cat 1 — power is
		// retained while the drives wind down — and an operator watching a train
		// still moving after they hit the button needs to know that is the stop
		// working rather than the stop failing.
		// WHY THE RIDE IS NOT RUNNING BELONGS ON THE OPERATOR'S VIEW, not only on
		// the engineering page. The controller module was maintenance-only, so a
		// faulted PLC showed as a ride that simply sat there — every drive
		// stopped, every platform "train not in position", and nothing anywhere
		// saying the controller had gone. That is precisely the complaint this
		// project wrote into PlcUnit.h and then shipped.
		FString StopText(TEXT("RUNNING"));
		const char* PlcWhy = Plc.WhyNotRun();
		if (!bStopped && PlcWhy != nullptr)
		{
			StopText = FString::Printf(TEXT("PLC — %s"), UTF8_TO_TCHAR(PlcWhy));
		}
		if (bStopped)
		{
			const bool bCat1 =
				Drives->EmergencyStopCategory() == FTrackDrives::EStopCategory::One;
			StopText = FString::Printf(TEXT("E-STOP %s · %s"),
				bCat1 ? (Drives->IsPowerRemoved() ? TEXT("CAT1") : TEXT("CAT1 STOPPING"))
					  : TEXT("CAT0"),
				UTF8_TO_TCHAR(Drives->EmergencyStopReason()));
		}
		PanelLabel(Canvas, StopX + 5.f, Ty, StopText,
			bStopped ? PanelRed : (PlcWhy != nullptr ? PanelAmber : PanelGreen));
		Ty += Row + 6.f;
	}

	// ---- THE CIRCUIT, AS A SCHEMATIC ----------------------------------------
	//
	// A real ride's HMI draws the TRACK and puts the lamps on it, rather than
	// listing blocks in a table. It is the better view for the same reason the
	// coloured rails are: you are looking for where something is, and a diagram
	// answers that in one glance where a table makes you read.
	//
	// Block widths are PROPORTIONAL TO BLOCK LENGTH, so the strip is a plan of the
	// circuit rather than a row of equal boxes — the 130 m mid-course brake looks
	// like the long block it is, and the four 10 m platform positions look like the
	// tight cluster they are. All of it comes off Signals->Boundaries().
	PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
	PanelLabel(Canvas, Lx, Ty, TEXT("CIRCUIT"), PanelDim);
	Ty += Row + 2.f;

	{
		const std::vector<double>& Bounds = Signals->Boundaries();
		const double Total = FMath::Max(1.0, Track.TotalLength());
		const float StripX = Lx;
		const float StripW = W - Pad * 2.f - 4.f;
		const float BoxY = Ty + 10.f;
		const float BoxH = 16.f;

		// EVERY BLOCK GETS A FLOOR. Strictly proportional widths made the four 10 m
		// platform positions three pixels each on a 1288 m circuit — the single most
		// interesting part of the ride, invisible, while the 696 m free run got
		// two-fifths of the panel to say nothing in.
		//
		// So each block takes a minimum, and the REMAINDER is shared out
		// proportionally. Long blocks still read as long and short ones still read
		// as short; what goes is only the claim that the strip is to scale, which is
		// a claim no control room screen makes either.
		const float MinW = 9.f;
		const float Floor = MinW * NumBlocks;
		const float Share = FMath::Max(0.f, StripW - Floor);

		auto BlockX = [&](std::size_t Upto) -> float
		{
			float Out = StripX;
			for (std::size_t i = 0; i < Upto; ++i)
			{
				const double A = Bounds[i];
				const double Bnd = (i + 1 < Bounds.size()) ? Bounds[i + 1] : Total;
				Out += MinW + Share * static_cast<float>((Bnd - A) / Total);
			}
			return Out;
		};

		for (int32 b = 0; b < NumBlocks; ++b)
		{
			const std::size_t B = static_cast<std::size_t>(b);
			const double S0 = Bounds[B];
			const double S1 = (B + 1 < Bounds.size()) ? Bounds[B + 1] : Total;
			const float X0 = BlockX(B);
			const float X1 = BlockX(B + 1);
			const float BW = FMath::Max(2.f, X1 - X0 - 1.f);

			const EBlockState State = Signals->GetState(B);
			const bool bOcc = State == EBlockState::Occupied;
			const bool bBuf = State == EBlockState::Buffer;
			FLinearColor Lamp = bOcc ? PanelAmber : (bBuf ? PanelCyan : PanelGreen);

			// A disagreement between the two detection methods paints the block RED
			// on the diagram, which is where somebody is already looking.
			if (Counter && B < Counter->NumBlocks()
				&& (Counter->IsOccupied(B) != Signals->Occupies(B) || Counter->IsOverOccupied(B)))
			{
				Lamp = PanelRed;
			}

			// Filled when it holds something, outlined when clear: an occupied
			// block should read from across a room.
			if (bOcc || bBuf)
			{
				PanelTile(Canvas, X0, BoxY, BW, BoxH, Lamp);
			}
			else
			{
				PanelTile(Canvas, X0, BoxY, BW, 1.f, Lamp);
				PanelTile(Canvas, X0, BoxY + BoxH - 1.f, BW, 1.f, Lamp);
				PanelTile(Canvas, X0, BoxY, 1.f, BoxH, Lamp);
				PanelTile(Canvas, X0 + BW - 1.f, BoxY, 1.f, BoxH, Lamp);
			}

			// The device under the block, in the SAME colours the rails use, so the
			// panel and the track view are the same object seen twice. Blocks and
			// zones come from one walk, so a device that starts in this block fills
			// it — there is no partial case to draw.
			for (const FTUZoneSpan& Z : ZoneSpans)
			{
				if (Z.StartS >= S0 - 0.01 && Z.StartS < S1)
				{
					PanelTile(Canvas, X0, BoxY + BoxH + 2.f, BW, 3.f,
						RailColourAt(Z.StartS + 0.01));
				}
			}
		}

		// EVERY TRAIN ON THE DIAGRAM, at its real position. The single most useful
		// thing a control room screen shows, and it costs one tile each. Placed
		// through the same widened mapping the blocks use, or a train would sit
		// somewhere other than the block the panel says it is in.
		auto SToX = [&](double S) -> float
		{
			const std::size_t B = Signals->BlockAt(S);
			const double A = Bounds[B];
			const double Bn = (B + 1 < Bounds.size()) ? Bounds[B + 1] : Total;
			const float X0 = BlockX(B);
			const float X1 = BlockX(B + 1);
			const double F = (Bn > A) ? (S - A) / (Bn - A) : 0.0;
			return X0 + static_cast<float>(F) * (X1 - X0);
		};

		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			const float Px = SToX(Trains[t]->GetDistance());
			PanelTile(Canvas, Px - 1.f, BoxY - 5.f, 3.f, BoxH + 10.f, PanelText);
			PanelLabel(Canvas, Px - 3.f, BoxY - 18.f, FString::Printf(TEXT("%d"), t), PanelText);
		}

		// Scale, so the strip is a drawing rather than a picture.
		PanelLabel(Canvas, StripX, BoxY + BoxH + 6.f, TEXT("0"), PanelDim);
		PanelLabel(Canvas, StripX + StripW - 44.f, BoxY + BoxH + 6.f,
			FString::Printf(TEXT("%.0f m"), Total), PanelDim);

		// MAINTENANCE: what the SECOND detection method counted, under the block it
		// counted it for. The strip above paints a disagreeing block red, which says
		// where; this says what the counter actually believes, which is the
		// difference between "something is wrong here" and a diagnosis. A block
		// reading 2 is a collision derived from switches alone; one reading below
		// zero is a missed trip or a bad seed, and is a LIE rather than a collision.
		//
		// Aligned to the blocks through the same widened mapping, or a number would
		// sit under a block it does not describe.
		if (bMaint && Counter)
		{
			for (int32 b = 0; b < NumBlocks; ++b)
			{
				const std::size_t B = static_cast<std::size_t>(b);
				if (B >= Counter->NumBlocks())
				{
					break;
				}
				const int N = Counter->TrainsIn(B);
				const bool bAgrees = Counter->IsOccupied(B) == Signals->Occupies(B);
				PanelLabel(Canvas, BlockX(B) + 2.f, BoxY + BoxH + 19.f,
					FString::Printf(TEXT("%d"), N),
					(!bAgrees || N > 1 || N < 0) ? PanelRed : PanelDim);
			}
		}
		Ty += StripH;
	}

	// ---- THE CONTROLLER ITSELF (maintenance only) ----------------------------
	//
	// The cabinet has a PLC in it, so the panel shows one — the same reasoning
	// that gave every drive a VFD module. Mode, scan health, and the identity of
	// the program it is running, which is the field a real installation cares
	// most about and the one that catches a program built for another layout.
	if (bMaint)
	{
		Ty += 4.f;
		PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
		PanelLabel(Canvas, Lx, Ty, TEXT("CONTROLLER"), PanelDim);

		const EPlcMode M = Plc.GetMode();
		const bool bRunning = Plc.OutputsEnabled();
		const TCHAR* ModeName = M == EPlcMode::Run ? TEXT("RUN")
			: (M == EPlcMode::Program ? TEXT("PROGRAM") : TEXT("STOPPED"));

		PanelTile(Canvas, Lx + 130.f, Ty + 2.f, 7.f, 7.f,
			bRunning ? PanelGreen : (Plc.IsFaulted() ? PanelRed : PanelAmber));
		PanelLabel(Canvas, Lx + 142.f, Ty, ModeName,
			bRunning ? PanelGreen : (Plc.IsFaulted() ? PanelRed : PanelAmber));

		// The program's identity, truncated. It is a digest of the derived blocks
		// and zones, so two rides showing the same one are running the same
		// control structure — which is the whole point of printing it.
		PanelLabel(Canvas, Lx + 226.f, Ty,
			FString::Printf(TEXT("PGM %08x"),
				static_cast<uint32>(Plc.ProgramIdentity() & 0xFFFFFFFFull)),
			Plc.ProgramMatchesLayout() ? PanelDim : PanelRed);

		// Why it is not running, when it is not. "The ride will not start" with
		// no reason attached is the commonest complaint about real ride control,
		// and the machine always knows.
		const char* Why = Plc.WhyNotRun();
		if (Why != nullptr)
		{
			PanelLabel(Canvas, Lx + 320.f, Ty, UTF8_TO_TCHAR(Why), PanelAmber);
		}
		Ty += Row;
	}

	// ---- DETECTION: the two methods, stated side by side (maintenance only) ----
	//
	// Agreement is the normal case and is still worth showing, because a second
	// means of detection that is never SEEN to agree is indistinguishable from one
	// that is not running. This is the line that says the cross-check is alive.
	//
	// Operator view leaves it out deliberately: a disagreement already trips the
	// E-stop and names itself in the events, and there is nothing an operator can
	// do with the per-block breakdown that the stop has not already done for them.
	if (bMaint)
	{
		Ty += 4.f;
		PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
		PanelLabel(Canvas, Lx, Ty, TEXT("DETECTION"), PanelDim);

		if (Counter)
		{
			const int32 N = FMath::Min(static_cast<int32>(Counter->NumBlocks()), NumBlocks);
			int32 Agree = 0;
			int32 Over = 0;
			for (int32 b = 0; b < N; ++b)
			{
				const std::size_t B = static_cast<std::size_t>(b);
				if (Counter->IsOccupied(B) == Signals->Occupies(B)) { ++Agree; }
				if (Counter->IsOverOccupied(B)) { ++Over; }
			}
			FString S = FString::Printf(TEXT("COUNTER v INTERLOCKING   %d/%d AGREE"), Agree, N);
			if (Over > 0)
			{
				S += FString::Printf(TEXT("   %d OVER"), Over);
			}
			PanelLabel(Canvas, Lx + 130.f, Ty, S,
				(Agree < N || Over > 0) ? PanelRed : PanelGreen);
		}
		else
		{
			// No counter on an open layout, and that is a property of the layout
			// rather than a fault: FBlockCounter counts over a RING, so sensor 0
			// going low is a train being PLACED at the start, not one leaving the
			// end. Saying so beats an empty row that looks like a dead instrument.
			PanelLabel(Canvas, Lx + 130.f, Ty,
				TEXT("SPAN ONLY — no counter on an open layout"), PanelDim);
		}
		Ty += Row;
	}

	// ---- DRIVES: a module per powered run --------------------------------------
	//
	// THE ONE SECTION THE TWO VIEWS GENUINELY DISAGREE ABOUT. Commanded against
	// output against motor feedback, and torque, are how you tell a slipping tyre
	// from a ramping one — engineering questions, on a page an engineer opens. An
	// operator dispatching trains needs one word per drive and cannot act on the
	// numbers, and a real console reflects that: motor current is not on it.
	//
	// So the operator gets the STATE and the maintainer gets the instrument, both
	// off the same live reading. Neither is a summary of the other — they are two
	// different questions about one drive.
	Ty += 4.f;
	PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
	// Headings at the SAME x as the values they head. Spelled out in one padded
	// string they drifted, because the small font is not monospace.
	PanelLabel(Canvas, Lx, Ty, TEXT("DRIVES"), PanelDim);
	if (bMaint)
	{
		PanelLabel(Canvas, Lx + 130.f, Ty, TEXT("CMD"), PanelDim);
		PanelLabel(Canvas, Lx + 178.f, Ty, TEXT("OUT"), PanelDim);
		PanelLabel(Canvas, Lx + 226.f, Ty, TEXT("MOTOR"), PanelDim);
		PanelLabel(Canvas, Lx + 288.f, Ty, TEXT("TORQUE"), PanelDim);
	}
	Ty += Row;

	for (int32 z = 0; z < NumDrives; ++z)
	{
		const std::size_t Z = static_cast<std::size_t>(z);
		const FDriveReading& R = Drives->Read(Z);
		const bool bFault = Drives->IsFaulted(Z);
		const ETUSegmentZone Kind = ZoneSpans.IsValidIndex(z)
			? ZoneSpans[z].Kind : ETUSegmentZone::None;

		PanelLabel(Canvas, Lx, Ty, FString::Printf(TEXT("Z%d %s"), z, ZoneKindName(Kind)),
			bFault ? PanelRed : PanelText);

		if (!bMaint)
		{
			// FOUR STATES, and every one is a fact the drive holds about ITSELF —
			// no inference about trains, which a drive has no way to make. RAMPING
			// is the state that only became sayable when a command stopped taking
			// effect instantly, and it is the one that explains a dispatch being
			// refused by the pre-launch term.
			const TCHAR* State = TEXT("STOPPED");
			FLinearColor Col = PanelDim;
			if (bFault)                          { State = TEXT("FAULT");   Col = PanelRed; }
			else if (!Drives->IsReady(Z))        { State = TEXT("RAMPING"); Col = PanelAmber; }
			else if (FMath::Abs(R.Output) > 0.01) { State = TEXT("RUNNING"); Col = PanelGreen; }

			PanelTile(Canvas, Lx + 130.f, Ty + 2.f, 7.f, 7.f, Col);
			PanelLabel(Canvas, Lx + 142.f, Ty, State, Col);
			Ty += Row;
			continue;
		}

		PanelLabel(Canvas, Lx + 130.f, Ty, FString::Printf(TEXT("%5.1f"), R.Commanded), PanelDim);
		PanelLabel(Canvas, Lx + 178.f, Ty, FString::Printf(TEXT("%5.1f"), R.Output), PanelText);
		PanelLabel(Canvas, Lx + 226.f, Ty, FString::Printf(TEXT("%5.1f"), R.Actual),
			R.bLoaded ? PanelCyan : PanelDim);

		// Torque as a bar, because a number that spends its life at 0 or 1 says
		// less at a glance than a bar that fills.
		const float BarX = Lx + 288.f;
		const float BarW = 96.f;
		PanelTile(Canvas, BarX, Ty + 3.f, BarW, 6.f, FLinearColor(0.12f, 0.14f, 0.16f, 1.f));
		PanelTile(Canvas, BarX, Ty + 3.f, BarW * static_cast<float>(R.Load), 6.f,
			bFault ? PanelRed : (R.Load > 0.99 ? PanelAmber : PanelCyan));
		if (bFault)
		{
			// ACKNOWLEDGED is worth a word of its own here and nowhere else: it is
			// the maintainer's own audit trail, and "faulted, nobody has looked" is
			// a different state from "faulted, seen, not yet cleared".
			PanelLabel(Canvas, BarX + BarW + 6.f, Ty,
				Drives->IsAcknowledged(Z) ? TEXT("FAULT ACK") : TEXT("FAULT"), PanelRed);
		}
		else
		{
			// THE STATUSWORD, which is what an engineering page on a real
			// installation shows. Four states this project invented are a fair
			// summary; 6041h is what the drive actually says, and a maintainer
			// who knows CiA 402 reads it without being taught anything.
			static const TCHAR* CiaNames[] = {
				TEXT("NOT RDY"), TEXT("SW ON DIS"), TEXT("RDY SW ON"), TEXT("SWITCHED ON"),
				TEXT("OP ENABLED"), TEXT("QUICK STOP"), TEXT("FLT REACT"), TEXT("FAULT")};
			const FCia402Drive& C = Drives->Cia402(Z);
			PanelLabel(Canvas, BarX + BarW + 6.f, Ty,
				FString::Printf(TEXT("%s %04x"),
					CiaNames[static_cast<int32>(C.State())], C.Statusword()),
				C.ProducesTorque() ? PanelDim : PanelAmber);
		}
		Ty += Row;
	}

	// ---- PLATFORMS: where each one is in its sequence, and what is holding it ----
	if (Platforms.Num() > 0)
	{
		Ty += 4.f;
		PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
		PanelLabel(Canvas, Lx, Ty, TEXT("PLATFORMS"), PanelDim);
		Ty += Row;

		for (const FTUPlatform& P : Platforms)
		{
			const bool bReady = P.Process.IsReadyToDispatch();
			const ETUSegmentZone Kind = ZoneSpans.IsValidIndex(P.Zone)
				? ZoneSpans[P.Zone].Kind : ETUSegmentZone::None;

			PanelTile(Canvas, Lx, Ty + 3.f, 6.f, 6.f, bReady ? PanelGreen : PanelDim);
			PanelLabel(Canvas, Lx + 12.f, Ty,
				FString::Printf(TEXT("Z%d %s"), P.Zone, ZoneKindName(Kind)), PanelText);
			PanelLabel(Canvas, Lx + 130.f, Ty, PhaseName(P.Process.GetPhase()),
				bReady ? PanelGreen : PanelAmber);

			// WHAT IS HOLDING IT, which is the reason the station's gates are
			// modelled one at a time. "Not ready" is useless to somebody standing
			// on the platform; "restraints not locked" is somewhere to go and look.
			const FString Holding = UTF8_TO_TCHAR(P.Process.WhatIsHolding());
			if (!Holding.IsEmpty())
			{
				PanelLabel(Canvas, Lx + 226.f, Ty, Holding, PanelDim);
			}
			else if (P.Process.GetPhase() == EStationPhase::Departing)
			{
				// A DEPARTING TRAIN IS NOT A BLOCKED ONE, and the permissive text
				// below would call it one. The release is latched, so `bReady`
				// stays true while the train rolls off its mark — and the block it
				// is being asked about is the block it has just entered ITSELF, so
				// CanRelease says no and the row read "DEPARTING · DISPATCH — block
				// ahead". Both halves true, the pair nonsense.
				//
				// The reason column answers "why has it not gone". A train that has
				// gone has no answer to give, and the phase word already says so.
			}
			else if (bReady)
			{
				// Ready and still standing means the OTHER half of the AND is
				// saying no, which is the distinction an operator most wants and
				// can least otherwise see.
				const double AtS = ZoneSpans.IsValidIndex(P.Zone) ? ZoneSpans[P.Zone].StartS : 0.0;
				const bool bBlocked = !Signals->CanRelease(0, AtS);
				// PRE-LAUNCH gets its own words, because "waiting" for a block that
				// is occupied and "waiting" for a launch that has not armed are two
				// different waits and an operator needs to know which.
				const bool bNotArmed = !DeviceAheadIsReady(AtS);

				// ADVANCE OR DISPATCH, and a real console has them as separate
				// labelled buttons. They are not the same move: an advance shuffles
				// a train up to the next position on the same platform, still with
				// riders boarding behind it; a dispatch sends it onto the course
				// and is the last point anybody can change their mind. The model
				// can already tell them apart — it is whether the next block along
				// is another platform position or open track.
				const TCHAR* Verb = TEXT("DISPATCH");
				if (Signals->NumBlocks() > 0)
				{
					const std::size_t Here = Signals->BlockAt(AtS + 0.01);
					const std::size_t Next = (Here + 1) % Signals->NumBlocks();
					const double NextS = Signals->Boundaries()[Next];
					for (const FTUPlatform& Q : Platforms)
					{
						if (ZoneSpans.IsValidIndex(Q.Zone)
							&& FMath::Abs(ZoneSpans[Q.Zone].StartS - NextS) < 0.01)
						{
							Verb = TEXT("ADVANCE");
						}
					}
				}
				FString Why = Verb;
				if (bBlocked)      { Why = FString::Printf(TEXT("%s — block ahead"), Verb); }
				else if (bNotArmed) { Why = FString::Printf(TEXT("%s — PRE-LAUNCH"), Verb); }
				PanelLabel(Canvas, Lx + 226.f, Ty, Why,
					(bBlocked || bNotArmed) ? PanelAmber : PanelGreen);
			}
			Ty += Row;
		}

		// ---- THE CONSOLE, as lamps ------------------------------------------
		//
		// Straight off an operator's panel: CONTROL POWER, RESTRAINTS, GATES,
		// DISPATCH, E-STOP RESET, EMERGENCY STOP — each an illuminated control that
		// shows its own state. A screen cannot be pressed, so these are indicators
		// rather than buttons, but they are the same six facts an operator reads
		// off the panel in front of them, and every one is live.
		//
		// It shows ONE platform: whichever has a train and is furthest along, which
		// is the one an operator standing at a station console is working. Nothing
		// here is invented — a control this cannot honestly light is left off it,
		// which is why there is no FLOOR RAISE/LOWER lamp.
		const FTUPlatform* Console = nullptr;
		for (const FTUPlatform& P : Platforms)
		{
			if (P.Inputs.bTrainPresent
				&& (Console == nullptr || P.Zone > Console->Zone))
			{
				Console = &P;
			}
		}

		Ty += 4.f;
		PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
		PanelLabel(Canvas, Lx, Ty, Console != nullptr
			? FString::Printf(TEXT("CONSOLE · Z%d"), Console->Zone)
			: FString(TEXT("CONSOLE")), PanelDim);
		Ty += Row;

		auto Lamp = [&](float Lx2, const TCHAR* Label, bool bLit, const FLinearColor& Col)
		{
			PanelTile(Canvas, Lx2, Ty + 2.f, 7.f, 7.f, bLit ? Col : FLinearColor(0.16f, 0.18f, 0.20f, 1.f));
			PanelLabel(Canvas, Lx2 + 12.f, Ty, Label, bLit ? Col : PanelDim);
		};

		const bool bStop = Drives->IsEmergencyStopped();
		// LOCK HARNESS / GATES / DISPATCH READY, in the order and the words a real
		// console uses. "DISPATCH READY" is its own lamp on all three panels rather
		// than a property of the dispatch button, because the machine granting
		// permission and a person taking it are two different events.
		// HARNESS shows the bank's own sensors, not the switch position. Commanded
		// closed with bars still travelling reads amber with a count, because "told
		// to close" and "closed" are different facts and the gap between them is
		// exactly what a walk-round is looking for.
		if (Console != nullptr && Console->Crew.Restraints.IsCommandedClosed()
			&& !Console->Crew.Restraints.IsClosedAndLocked())
		{
			PanelTile(Canvas, Lx, Ty + 2.f, 7.f, 7.f, PanelAmber);
			PanelLabel(Canvas, Lx + 12.f, Ty,
				FString::Printf(TEXT("HARNESS %d/%d"),
					Console->Crew.Restraints.GroupsConfirmed(),
					Console->Crew.Restraints.Groups), PanelAmber);
		}
		else
		{
			Lamp(Lx, TEXT("HARNESS LOCKED"),
				Console != nullptr && Console->Inputs.bRestraintsLocked, PanelGreen);
		}
		// GATES read their own sensors too, so a jammed section shows as a count
		// rather than as a dark lamp indistinguishable from "not commanded yet".
		if (Console != nullptr && Console->Crew.Gates.IsCommandedClosed()
			&& !Console->Crew.Gates.IsClosedAndLocked())
		{
			PanelTile(Canvas, Lx + 116.f, Ty + 2.f, 7.f, 7.f, PanelAmber);
			PanelLabel(Canvas, Lx + 128.f, Ty,
				FString::Printf(TEXT("GATES %d/%d"),
					Console->Crew.Gates.GroupsConfirmed(),
					Console->Crew.Gates.Groups), PanelAmber);
		}
		else
		{
			Lamp(Lx + 116.f, TEXT("GATES"),
				Console != nullptr && Console->Crew.Gates.IsClosedAndLocked(), PanelGreen);
		}
		Lamp(Lx + 186.f, TEXT("DISPATCH READY"),
			Console != nullptr && Console->Process.IsReadyToDispatch(), PanelGreen);
		Lamp(Lx + 300.f, TEXT("E-STOP"), bStop, PanelRed);
		// RESET is only a live control once everything has been acknowledged, so it
		// lights differently from the thing it cannot yet do.
		Lamp(Lx + 372.f, bStop && Drives->AnyUnacknowledged() ? TEXT("ACK") : TEXT("RESET"),
			bStop, bStop && Drives->AnyUnacknowledged() ? PanelAmber : PanelCyan);
		Ty += Row;

		// ---- THE EVENT LOG ------------------------------------------------
		//
		// The difference between a status display and a RECORD. Everything above
		// says what is true now; this says what happened, which is the first thing
		// anybody asks after a ride stops. Newest first, and fed by the same places
		// that log, so there is one story rather than two that can disagree.
		if (EventLog.Num() > 0)
		{
			Ty += 4.f;
			PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
			PanelLabel(Canvas, Lx, Ty, TEXT("EVENTS"), PanelDim);
			Ty += Row;

			const int32 Show = FMath::Min(EventLog.Num(), 4);
			for (int32 e = 0; e < Show; ++e)
			{
				const FTURideEvent& Ev = EventLog[e];
				PanelLabel(Canvas, Lx, Ty,
					FString::Printf(TEXT("%6.1f s"), Ev.AtSeconds), PanelDim);
				PanelLabel(Canvas, Lx + 56.f, Ty, Ev.Text, Ev.bBad ? PanelRed : PanelDim);
				Ty += Row;
			}
		}
	}
}

// THE OPERATOR'S BUTTON IS THE ONE CATEGORY 1 STOP ON THE RIDE, and every
// automatic trip is Category 0. That split is a risk judgement rather than a
// preference, and IEC 60204-1 leaves it to exactly that:
//
//   A person pressing the button has decided the ride should stop. Nothing is
//   known to be broken, so a drive can be trusted to wind its own output down
//   before power goes — which is what stops a train being dropped mid-push.
//
//   A protective trip — signalling violation, detection disagreement, a counter
//   that has gone inconsistent, a faulted drive — means something IS broken, and
//   often that the thing being asked to perform a controlled stop is the thing
//   that failed. Power goes now.
//
// Both are safe because THE BRAKES ARE FAIL-SAFE: a zone commanded to zero
// bites, so removing power applies the brakes rather than merely ceasing to
// push. Cat 0 stops trains, it does not just stop driving them.
void ATUCoasterRide::PressEmergencyStop()
{
	bEmergencyStop = true;
	if (Drives && Drives->PressEmergencyStopButton("operator"))
	{
		UE_LOG(LogTemp, Error, TEXT("TrackUnlimited: EMERGENCY STOP — operator (Cat 1)."));
		LogEvent(TEXT("EMERGENCY STOP — operator (Cat 1)"));
	}
}

// Channel ranges. Allocated by hand and deliberately far apart: a collision here
// is silent and reads as a phantom transition on an unrelated signal.
namespace TUWatch
{
	constexpr std::size_t Blocks    = 0;
	constexpr std::size_t Platforms = 1000;
	constexpr std::size_t Drives    = 2000;
	constexpr std::size_t Console   = 3000;
	constexpr std::size_t Motion    = 4000;
}

// Which platform, if any, this train is standing at. A platform's bank is the
// only place its state exists, so a train has to be matched to one to sample it.
// Matched by SPAN rather than by a stored index, because the platform genuinely
// does not know which train is over it — the same honesty the stop marks keep.
void ATUCoasterRide::SampleRestraints()
{
	const int32 N = Trains.Num();
	if (TrainGroupClosed.Num() != N)
	{
		TrainGroupClosed.SetNum(N);
	}

	for (int32 t = 0; t < N; ++t)
	{
		const double S = Trains[t]->GetDistance();
		const FTUPlatform* At = nullptr;
		for (const FTUPlatform& P : Platforms)
		{
			if (!ZoneSpans.IsValidIndex(P.Zone)) { continue; }
			if (S >= ZoneSpans[P.Zone].StartS - 0.01 && S < ZoneSpans[P.Zone].EndS)
			{
				At = &P;
				break;
			}
		}
		// Away from a platform the last sample STANDS. The bars did not open
		// because the train left the room that was commanding them.
		if (At == nullptr) { continue; }

		const FCommandedBank& B = At->Crew.Restraints;
		const int32 G = FMath::Max(1, B.Groups);
		const int32 Base = t * G;
		if (TrainGroupState.Num() < Base + G)
		{
			TrainGroupState.SetNum(Base + G);
		}
		for (int32 g = 0; g < G; ++g)
		{
			TrainGroupState[Base + g] = static_cast<uint8>(B.GroupState(g));
		}
		TrainGroupClosed[t] = B.IsCommandedClosed();
	}
}

void ATUCoasterRide::BuildBlockMarks()
{
	BlockMarks.Reset();
	if (!Signals || Track.TotalLength() <= 0.0)
	{
		return;
	}

	// ONE WALK for every boundary. Boundaries are ascending, so AdvanceFrom can
	// carry the frame forward from one to the next — eleven posts cost one
	// integration of the track rather than eleven.
	const std::vector<double>& B = Signals->Boundaries();
	FTrackFrame Walk = Track.EvaluateAt(0.0);
	double At = 0.0;

	for (std::size_t i = 0; i < B.size(); ++i)
	{
		const double To = FMath::Clamp(B[i], 0.0, Track.TotalLength());
		if (To > At)
		{
			Walk = Track.AdvanceFrom(Walk, At, To);
			At = To;
		}
		FTUBlockMark M;
		M.World = ToWorld(Walk.Position);
		// Kept in WORLD space, converted once here rather than per frame, and
		// through the same mapping the rest of the drawing uses — a post that
		// leans the wrong way on a banked boundary would be its own small lie.
		//
		// NORMALISED, and it was not. Differencing two ToWorld calls a metre
		// apart gives a vector of length 100, because ToWorld converts metres to
		// centimetres — so this was a UNIT-PER-METRE vector wearing the name of a
		// direction, and every multiplier applied to it downstream came out 100x.
		// The gate meant to stand 4.2 m over the rails stood 420 m over them.
		M.Up = (ToWorld(Walk.Position + Walk.Up) - M.World).GetSafeNormal();
		M.Lateral = (ToWorld(Walk.Position + Walk.Lateral) - M.World).GetSafeNormal();
		BlockMarks.Add(M);
	}
}

void ATUCoasterRide::DrawBlockMarkers() const
{
	if (!bShowBlockMarkers || !Signals || BlockMarks.Num() == 0)
	{
		return;
	}

	for (int32 i = 0; i < BlockMarks.Num(); ++i)
	{
		const FTUBlockMark& M = BlockMarks[i];
		const std::size_t B = static_cast<std::size_t>(i);

		// A boundary is the START of block i, so it carries block i's colour —
		// THE SAME COLOURS THE PANEL'S BLOCK STRIP USES, because the strip and
		// the track are meant to be one object seen twice. Amber occupied, cyan
		// in its overlap, green clear.
		const EBlockState State = Signals->GetState(B);
		FColor Col = State == EBlockState::Occupied ? FColor(250, 158, 41)
			: (State == EBlockState::Buffer ? FColor(89, 189, 255) : FColor(89, 209, 115));

		// A disagreement between the two detection methods paints it RED, exactly
		// as it does on the panel — so a cross-check trip can be found on the
		// track rather than only read about.
		if (Counter && B < Counter->NumBlocks()
			&& (Counter->IsOccupied(B) != Signals->Occupies(B) || Counter->IsOverOccupied(B)))
		{
			Col = FColor(215, 60, 50);
		}

		// A gate across the track rather than a post beside it: the boundary is a
		// line the train crosses, and a marker off to one side reads as scenery.
		// CENTIMETRES, because M.Up and M.Lateral are unit vectors in UE space.
		// A gate 4.2 m over the rails and 1.6 m under them, 2.6 m to each side —
		// which is about a train and a half wide and reads as a gantry rather
		// than a wall.
		const FVector Top = M.World + M.Up * 420.f;
		const FVector Foot = M.World - M.Up * 160.f;
		const FVector Half = M.Lateral * 260.f;

		DrawDebugLine(GetWorld(), Foot, Top, Col, false, -1.f, 0, 4.f);
		DrawDebugLine(GetWorld(), Top - Half, Top + Half, Col, false, -1.f, 0, 4.f);
		DrawDebugLine(GetWorld(), Foot - Half, Foot + Half, Col, false, -1.f, 0, 2.f);
	}
}

void ATUCoasterRide::DrawRestraints() const
{
	if (!bShowRestraints || Trains.Num() == 0 || Platforms.Num() == 0)
	{
		return;
	}
	// Group count comes from the banks, which all carry the same authored number.
	const int32 G = FMath::Max(1, Platforms[0].Crew.Restraints.Groups);
	const double Len = FMath::Max(1.0, static_cast<double>(TrainLengthM));

	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		const int32 Base = t * G;
		if (!TrainGroupState.IsValidIndex(Base + G - 1)) { continue; }
		const bool bClosed = TrainGroupClosed.IsValidIndex(t) && TrainGroupClosed[t];

		for (int32 g = 0; g < G; ++g)
		{
			// Group g spans its share of the train, measured from the rear. The
			// nine sample points are a physics discretisation and are NOT cars;
			// the groups are the things with a sensor on them.
			const double Offset = -Len * 0.5 + (static_cast<double>(g) + 0.5) * (Len / G);
			const FTrackFrame& F = Trains[t]->GetFrameAt(Offset);

			const auto State =
				static_cast<FCommandedBank::EGroupState>(TrainGroupState[Base + g]);

			// STUCK IS RED WHATEVER IT WAS DOING. A bar that will not close is a
			// train that cannot go; a bar that will not OPEN is a rider who cannot
			// get out, which is a different emergency and just as much a fault.
			FColor Col = FColor(90, 95, 100);                       // open, at rest
			if (State == FCommandedBank::EGroupState::Stuck)        { Col = FColor(215, 60, 50); }
			else if (State == FCommandedBank::EGroupState::Travelling) { Col = FColor(235, 160, 30); }
			else if (bClosed)                                       { Col = FColor(70, 200, 110); }

			// Drawn ABOVE the rails at the heartline, where a bar actually is, and
			// as a box rather than a tint so it reads from outside the train.
			const FVec3 P = F.Position + F.Up * 0.35;
			const float Half = static_cast<float>(Len / G) * 0.5f * 0.85f;
			DrawDebugBox(GetWorld(), ToWorld(P),
				FVector(Half * 100.f, 55.f, 30.f), ToWorldRotation(F), Col,
				false, -1.f, 0, State == FCommandedBank::EGroupState::Stuck ? 4.f : 2.f);
		}
	}
}

void ATUCoasterRide::PublishShowEvents()
{
	// TIER 3'S ONE INTERFACE. Events out, fixture commands out the other side, and
	// no path by which either can reach the ride — not because something checks,
	// but because there is nowhere to name a train.
	//
	// The same three sources the panel draws from, because a cue that fired on
	// something the operator could not see would be a fourth idea of the ride's
	// state. Blocks, sensors, platforms: sensor -> pyro, sensor -> camera,
	// station phase -> audio, block occupied -> scenery are all this one
	// subscription with a different fixture on the end.
	if (!Signals)
	{
		return;
	}

	ShowPublisher.BeginScan(ScanNumber);
	for (std::size_t b = 0; b < Signals->NumBlocks(); ++b)
	{
		ShowPublisher.Observe(ERideEventKind::BlockState, static_cast<int>(b),
			static_cast<int>(Signals->GetState(b)));
	}
	if (StopMarks)
	{
		for (std::size_t i = 0; i < StopMarks->Num(); ++i)
		{
			ShowPublisher.Observe(ERideEventKind::SensorTrip, static_cast<int>(i),
				StopMarks->IsBlocked(i) ? 1 : 0);
		}
	}
	for (int32 p = 0; p < Platforms.Num(); ++p)
	{
		ShowPublisher.Observe(ERideEventKind::StationPhase, p,
			static_cast<int>(Platforms[p].Process.GetPhase()));
	}

	// Nothing authors triggers yet, so this delivers into an empty list and the
	// return is dropped. That is the honest state of Tier 3 and it is deliberately
	// visible here rather than hidden behind an `if (bShowEnabled)` — the wiring
	// is what was missing, and a fixture layer plugs into this line.
	ShowBus.Deliver(ShowPublisher.Scanned());
}

void ATUCoasterRide::LogTransitions()
{
	if (!bLogStateTransitions || !Signals || !Drives)
	{
		return;
	}

	// ---- Blocks. Clear / occupied / buffer, per block.
	for (std::size_t b = 0; b < Signals->NumBlocks(); ++b)
	{
		const EBlockState S = Signals->GetState(b);
		const int Code = S == EBlockState::Occupied ? 1 : (S == EBlockState::Buffer ? 2 : 0);
		if (StateWatch.ChangedFrom(TUWatch::Blocks + b, Code))
		{
			static const TCHAR* Names[] = {TEXT("CLEAR"), TEXT("OCCUPIED"), TEXT("BUFFER")};
			LogEvent(FString::Printf(TEXT("block %d  %s -> %s"), static_cast<int32>(b),
				Names[StateWatch.Previous()], Names[Code]), false);
		}
	}

	// ---- Platforms. The sequence, which is what a dwell argument is settled by.
	for (int32 i = 0; i < Platforms.Num(); ++i)
	{
		const int Code = static_cast<int>(Platforms[i].Process.GetPhase());
		if (StateWatch.ChangedFrom(TUWatch::Platforms + static_cast<std::size_t>(i), Code))
		{
			LogEvent(FString::Printf(TEXT("Z%d  %s -> %s"), Platforms[i].Zone,
				PhaseName(static_cast<EStationPhase>(StateWatch.Previous())),
				PhaseName(static_cast<EStationPhase>(Code))), false);
		}
	}

	// ---- Drives. The four states the operator view shows, so the log and the
	// panel cannot tell different stories.
	for (std::size_t z = 0; z < Drives->Num(); ++z)
	{
		const FDriveReading& R = Drives->Read(z);
		int Code = 0;                                            // STOPPED
		if (Drives->IsFaulted(z))                { Code = 3; }   // FAULT
		else if (!Drives->IsReady(z))            { Code = 2; }   // RAMPING
		else if (FMath::Abs(R.Output) > 0.01)    { Code = 1; }   // RUNNING

		if (StateWatch.ChangedFrom(TUWatch::Drives + z, Code))
		{
			static const TCHAR* Names[] = {TEXT("STOPPED"), TEXT("RUNNING"),
										   TEXT("RAMPING"), TEXT("FAULT")};
			LogEvent(FString::Printf(TEXT("drive Z%d  %s -> %s  (cmd %.1f out %.1f)"),
				static_cast<int32>(z), Names[StateWatch.Previous()], Names[Code],
				R.Commanded, R.Output), Code == 3);
		}
	}

	// ---- TRAIN MOTION, which is what settles a throughput argument.
	//
	// A block releases when the REAR clears it, not when the train stops — that is
	// what OCCUPIED -> BUFFER already means. But without this you cannot tell from
	// a log whether the train ahead was still trucking to its mark while the next
	// one was released, or had already parked, and those are a ride that overlaps
	// its moves and a ride that queues them.
	//
	// CRAWLING is its own state rather than a slow RUNNING because it is a
	// different activity: the pad has stopped the train and the drive tyres are
	// conveying it to its stop mark, which on the mid-course brake is 67 m of
	// travel at walking pace. That stretch is exactly the window a busy operation
	// wants to overlap into.
	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		const double V = FMath::Abs(Trains[t]->GetSpeed());
		const int Code = V < 0.05 ? 0 : (V < 2.5 ? 1 : 2);
		if (StateWatch.ChangedFrom(TUWatch::Motion + static_cast<std::size_t>(t), Code))
		{
			static const TCHAR* Names[] = {TEXT("STOPPED"), TEXT("CRAWLING"), TEXT("RUNNING")};
			LogEvent(FString::Printf(TEXT("train %d  %s -> %s  at %.0f m"), t,
				Names[StateWatch.Previous()], Names[Code],
				Trains[t]->GetDistance()), false);
		}
	}

	// ---- THE CONSOLE CONTACTS, which are the ones a still frame cannot answer.
	//
	// "Is the harness lamp dark because the crew has already released, or because
	// it never lit?" is not a question a screenshot can settle, and it is exactly
	// the question worth asking about a securing sequence. Logged per platform
	// rather than for the one console the panel picks, or the answer depends on
	// which platform happened to be selected when you looked.
	for (int32 i = 0; i < Platforms.Num(); ++i)
	{
		const FTUPlatform& P = Platforms[i];
		const std::size_t Base = TUWatch::Console + static_cast<std::size_t>(i) * 8;

		// A BANK HAS THREE STATES, NOT A COUNT. `GroupsConfirmed` counts groups
		// that have reached their COMMANDED position, whichever direction that
		// is — so "4/4" means fully locked when closing and fully OPEN when
		// releasing. The first version of this logged it as "harness 4/4 locked"
		// either way, which read as authoritative and said the opposite of the
		// truth half the time. A lying instrument is worse than no instrument.
		//
		// So log what it MEANS, and keep the count only where it is the
		// interesting part: mid-travel, where "commanded closed but 3 of 4" is
		// the failure a walk-round exists to find.
		auto BankState = [](const FCommandedBank& B) -> int
		{
			if (B.IsClosedAndLocked()) { return 2; }
			if (B.IsFullyOpen())       { return 0; }
			return 1;                  // travelling, or stuck
		};
		auto BankWords = [](const FCommandedBank& B, int S) -> FString
		{
			if (S == 2) { return FString(TEXT("CLOSED AND LOCKED")); }
			if (S == 0) { return FString(TEXT("open")); }
			return FString::Printf(TEXT("%s — %d/%d"),
				B.IsCommandedClosed() ? TEXT("closing") : TEXT("releasing"),
				B.GroupsConfirmed(), B.Groups);
		};

		const int RState = BankState(P.Crew.Restraints);
		if (StateWatch.ChangedFrom(Base + 0, RState))
		{
			LogEvent(FString::Printf(TEXT("Z%d  restraints %s"), P.Zone,
				*BankWords(P.Crew.Restraints, RState)), false);
		}
		const int GState = BankState(P.Crew.Gates);
		if (StateWatch.ChangedFrom(Base + 1, GState))
		{
			LogEvent(FString::Printf(TEXT("Z%d  gates %s"), P.Zone,
				*BankWords(P.Crew.Gates, GState)), false);
		}
		if (StateWatch.ChangedFrom(Base + 3, P.Process.IsReadyToDispatch() ? 1 : 0))
		{
			LogEvent(FString::Printf(TEXT("Z%d  dispatch permission %s"), P.Zone,
				P.Process.IsReadyToDispatch() ? TEXT("GRANTED") : TEXT("withdrawn")), false);
		}
	}
}

void ATUCoasterRide::LogEvent(const FString& Text, bool bBad)
{
	// Newest first, capped. Oldest falls off the end rather than the ring wrapping
	// in place, because the panel wants "the last few" and a wrapped array has to
	// be unwrapped to give it.
	EventLog.Insert(FTURideEvent{RideClock, Text, bBad}, 0);
	// MIRRORED TO THE ENGINE LOG, WHICH IS WHERE IT PERSISTS. The in-memory ring
	// is what the panel reads while you are standing there; the file is what you
	// read afterwards, and "afterwards" is when every interesting question gets
	// asked. UE stamps it with a wall clock and a frame number for free, so the
	// two timebases sit side by side: RideClock says where in the ride, the file
	// says when in the session.
	//
	// Its own category rather than LogTemp, so `LogTUEvents` can be filtered to on
	// its own — with transitions on, this is the loudest thing in the file.
	UE_LOG(LogTUEvents, Log, TEXT("[%7.2f] %s"), RideClock, *Text);

	// Deeper when transitions are being recorded: eight is the right size for a
	// panel that shows four, and useless for a securing sequence that spends a
	// dozen lines getting a train out of the station.
	const int32 Keep = bLogStateTransitions ? 256 : 8;
	if (EventLog.Num() > Keep)
	{
		EventLog.SetNum(Keep);
	}
}

void ATUCoasterRide::AcknowledgeFaults()
{
	if (!Drives || !Drives->AnyUnacknowledged())
	{
		return;
	}
	for (std::size_t z = 0; z < Drives->Num(); ++z)
	{
		if (Drives->IsFaulted(z) && !Drives->IsAcknowledged(z))
		{
			Drives->AcknowledgeFault(z);
			LogEvent(FString::Printf(TEXT("zone %d fault acknowledged"),
				static_cast<int32>(z)), false);
			UE_LOG(LogTemp, Warning,
				TEXT("TrackUnlimited: drive %d fault ACKNOWLEDGED — seen, not fixed. "
					"[End] to reset once it is."),
				static_cast<int32>(z));
		}
	}
}

// The reset button as a CONTACT rather than an event. The drives layer decides
// when the 0-1-0 sequence is complete; this just reports the switch, which is all
// a real panel wire does.
//
// The acknowledgement check stays HERE, on the press, so an operator is told why
// nothing happened at the moment they press rather than at the moment they let
// go — and so the ordering rule keeps living in the layer that owns it.
// Turn the key. Refused with a readable reason rather than silently ignored:
// "the ride will not start" with no explanation is the commonest complaint about
// real ride control, and the machine always knows why.
void ATUCoasterRide::SetPlcMode(EPlcMode Wanted)
{
	if (Plc.RequestMode(Wanted))
	{
		static const TCHAR* Names[] = {TEXT("STOPPED"), TEXT("PROGRAM"), TEXT("RUN")};
		LogEvent(FString::Printf(TEXT("PLC mode -> %s"),
			Names[static_cast<int32>(Wanted)]), false);
		return;
	}
	const char* Why = Plc.WhyNotRun();
	UE_LOG(LogTemp, Warning, TEXT("TrackUnlimited: PLC refused RUN — %s."),
		Why ? UTF8_TO_TCHAR(Why) : TEXT("not powered"));
	LogEvent(FString::Printf(TEXT("PLC refused RUN — %s"),
		Why ? UTF8_TO_TCHAR(Why) : TEXT("not powered")));
}

// The operator's walkdown. A controller that came up knowing where trains are
// would be inventing occupancy it cannot possibly have watched.
void ATUCoasterRide::DeclareCourseClear()
{
	if (Plc.DeclareCourseClear())
	{
		LogEvent(TEXT("course declared clear"), false);
	}
}

void ATUCoasterRide::PressResetButton()
{
	if (Drives && Drives->AnyUnacknowledged())
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrackUnlimited: reset REFUSED — a drive fault has not been acknowledged. "
				"[Home] to acknowledge, then [End]."));
		return;
	}
	if (Drives) { Drives->ScanResetInput(true); }
}

void ATUCoasterRide::ReleaseResetButton()
{
	if (!Drives)
	{
		return;
	}
	// A fault raised BETWEEN the press and the release. Abandon the sequence
	// rather than complete it — otherwise the acknowledge-before-reset rule leaks
	// out through a gap in time, and a stop nobody has read clears itself.
	if (Drives->AnyUnacknowledged())
	{
		Drives->AbortReset();
		return;
	}
	if (Drives->ScanResetInput(false))
	{
		ResetEmergencyStop();
	}
}

void ATUCoasterRide::ResetEmergencyStop()
{
	// Cleared only here, never because the condition passed. Same reasoning as a
	// drive fault needing a reset: a stop nobody has looked at has not been dealt
	// with. Drive faults are cleared with it, because an operator resetting the
	// ride has been to look at what tripped it.
	// REFUSED WHILE ANYTHING IS UNACKNOWLEDGED. Every real console has ACKNOWLEDGE
	// and RESET as two separate controls, and the order between them is the point:
	// acknowledging says "I have seen this", resetting says "I have dealt with it".
	// A reset nobody had to read first clears faults nobody knows about.
	if (Drives && Drives->AnyUnacknowledged())
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrackUnlimited: reset REFUSED — a drive fault has not been acknowledged. "
				"[A] to acknowledge, then [End]."));
		return;
	}

	bEmergencyStop = false;
	if (Drives)
	{
		Drives->ResetEmergencyStop();
		for (std::size_t z = 0; z < Drives->Num(); ++z)
		{
			Drives->ResetFault(z);
		}
	}
	ReportedDriveFault.Reset();
	UE_LOG(LogTemp, Warning, TEXT("TrackUnlimited: emergency stop reset."));
	LogEvent(TEXT("emergency stop RESET"), false);
}

void ATUCoasterRide::ServeStations(float DeltaSeconds)
{
	// The station's inputs, from instruments, once per scan. Only two of the six
	// are real here and both are readings a control system genuinely has: a train
	// is in the zone, and it is stopped ON ITS MARK. The other four are the crew's,
	// which is the part that goes away when there are riders.
	//
	// "In position" is the stop mark AND a motor reading nothing, because a train
	// running through the platform covers the same switch. Two instruments, and the
	// pair means something neither of them does alone.
	TrainLoaded.SetNum(Trains.Num());

	for (FTUPlatform& P : Platforms)
	{
		const std::size_t Z = static_cast<std::size_t>(P.Zone);
		bool bPresent = false;
		int32 Who = 0;
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			if (Trains[t]->IsInZone(Z, Trains[t]->GetDistance()))
			{
				bPresent = true;
				Who = t;
			}
		}
		P.Inputs.bTrainPresent = bPresent;
		P.Inputs.bTrainInPosition = bPresent && StopMarks && Z < StopMarks->Num()
			&& StopMarks->IsBlocked(Z)
			&& Drives && FMath::Abs(Drives->Read(Z).Actual) < 1e-6;

		// One button, every platform. A real ride has one per position and an
		// operator standing at it; there is one keyboard here, so it presses them
		// all — which is right for the single-platform layouts this project has and
		// is the first thing a multi-position platform will have to split.
		P.Inputs.bDispatchRequest = bDispatchHeld;
		P.Process.SetMode(bManualDispatch ? EDispatchMode::Manual
										  : EDispatchMode::Automatic);

		// Already aboard, so there is nobody to board again. Only a LOAD position
		// passes through — a combined station's arriving train is full and genuinely
		// does need unloading, and an unload position's certainly does.
		const bool bAlready = bPresent && TrainLoaded.IsValidIndex(Who)
			&& P.Process.GetRole() == EStationRole::Load && TrainLoaded[Who];

		P.Process.Update(P.Inputs);
		P.Crew.Serve(P.Process, P.Inputs, DeltaSeconds, bAlready);

		// Boarded here, and it stays boarded. Set on readiness rather than on the
		// load contact so it survives being re-checked at the next position, which
		// is the whole reason the flag exists. Cleared when the train unloads.
		if (bPresent && TrainLoaded.IsValidIndex(Who))
		{
			if (P.Process.NeedsUnload() && P.Process.GetPhase() == EStationPhase::Unloading)
			{
				TrainLoaded[Who] = false;
			}
			if (P.Process.NeedsLoad() && P.Process.IsReadyToDispatch())
			{
				TrainLoaded[Who] = true;
			}
		}
	}
}

void ATUCoasterRide::ScanBlockSensors()
{
	if (!BlockSensors)
	{
		return;
	}
	BlockSensors->BeginScan();
	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		// Always circuit-wrapped: FBlockCounter is a counter over a RING, and this
		// only exists on a layout that is one.
		BlockSensors->Cover(Trains[t]->GetRearS(), Trains[t]->GetFrontS(),
			true, Track.TotalLength());
	}
	BlockSensors->EndScan();
}

void ATUCoasterRide::CrossCheckOccupancy()
{
	// TWO INDEPENDENT MEANS OF KNOWING WHERE THE TRAINS ARE, AND THEY MUST AGREE.
	// FRideSignals is handed each train's exact span every frame; the counter has
	// nothing but rising and falling edges at the block boundaries. They are
	// derived from different information, so a disagreement means one of them is
	// wrong and neither can say which — which is precisely why a real installation
	// pays for a second detection method rather than a better single one.
	//
	// Verified in test_twotrains.cpp: on both circuits, at every train count, the
	// two agree on every block on every frame. Breaking the counter's falling-edge
	// rule makes that assertion fail, so the check bites.
	if (!Counter || !BlockSensors || !Signals)
	{
		return;
	}
	ScanBlockSensors();
	Counter->Scan();

	for (std::size_t b = 0; b < Counter->NumBlocks(); ++b)
	{
		const TCHAR* Why = nullptr;
		if (Counter->IsOverOccupied(b))
		{
			// Two trains counted into a block nothing has counted out of. Derived
			// from switches alone, so it is a collision detected without anything
			// ever having known a position.
			Why = TEXT("two trains counted into one block");
		}
		else if (Counter->IsOccupied(b) != Signals->Occupies(b))
		{
			Why = TEXT("occupancy disagrees with the train counter");
		}
		if (Why != nullptr && Drives && Drives->TripEmergencyStop("detection disagreement"))
		{
			bEmergencyStop = true;
			LogEvent(FString::Printf(TEXT("block %d: %s"), static_cast<int32>(b), Why));
			UE_LOG(LogTemp, Error,
				TEXT("TrackUnlimited: EMERGENCY STOP — block %d: %s. The interlocking says "
					"%s and the counter says %d train(s)."),
				static_cast<int32>(b), Why,
				Signals->Occupies(b) ? TEXT("occupied") : TEXT("clear"),
				Counter->TrainsIn(b));
		}
	}

	// Below zero is not a collision, it is a LIE: the counter has been told a train
	// left somewhere it was never told one arrived. Seeding is wrong or a trip was
	// missed, and either way it can no longer be trusted to detect anything.
	if (Counter->IsInconsistent() && Drives
		&& Drives->TripEmergencyStop("train counter inconsistent"))
	{
		bEmergencyStop = true;
		UE_LOG(LogTemp, Error,
			TEXT("TrackUnlimited: EMERGENCY STOP — a block counted below zero. A trip was "
				"missed or the counters were seeded wrong; sweep the ride and reset."));
	}
}

bool ATUCoasterRide::DeviceAheadIsReady(double AtS) const
{
	if (!Signals || !Drives || Signals->NumBlocks() == 0)
	{
		return true;
	}
	const std::size_t Next = (Signals->BlockAt(AtS) + 1) % Signals->NumBlocks();
	const double NextS = Signals->Boundaries()[Next];

	// A zone's start IS a block boundary by construction - blocks and zones fall
	// out of the same walk - so the device in the next block is the zone that
	// begins there. No zone there means plain track, which takes a train perfectly
	// well and is always ready.
	for (int32 z = 0; z < ZoneSpans.Num(); ++z)
	{
		if (FMath::Abs(ZoneSpans[z].StartS - NextS) < 0.01)
		{
			return Drives->IsReady(static_cast<std::size_t>(z));
		}
	}
	return true;
}

bool ATUCoasterRide::StationSaysGo(std::size_t Zone) const
{
	for (const FTUPlatform& P : Platforms)
	{
		if (static_cast<std::size_t>(P.Zone) == Zone)
		{
			return P.Process.IsReadyToDispatch();
		}
	}
	return true;   // not a platform; nothing to ask
}

FColor ATUCoasterRide::RailColourAt(double S) const
{
	// A debug view, and the reason it earns its keep is that the DEVICES are the
	// part of a layout you cannot see. Geometry is visible — a hill is a hill —
	// but which stretch of track can hold a train, which can only slow one, and
	// which can only push one all look identical in wireframe, and confusing them
	// is the single most expensive authoring mistake this model allows.
	//
	// Linear scan over a handful of zones, once per half metre, once at BeginPlay.
	for (const FTUZoneSpan& Z : ZoneSpans)
	{
		if (S < Z.StartS || S > Z.EndS)
		{
			continue;
		}
		switch (Z.Kind)
		{
		case ETUSegmentZone::Station:
		case ETUSegmentZone::StationLoad: return FColor(35, 70, 165);   // dark blue
		// Distinguishable from the load platform without ceasing to read as a
		// station, because on a split operation they are different rooms and
		// telling them apart at a glance is the point of the view.
		case ETUSegmentZone::StationUnload: return FColor(70, 110, 200);
		case ETUSegmentZone::Lift:
		case ETUSegmentZone::Launch:     return FColor(70, 210, 95);    // green
		case ETUSegmentZone::Brake:
		case ETUSegmentZone::BlockBrake: return FColor(230, 60, 50);    // red
		default:                         break;
		}
	}
	return FColor(235, 235, 235);   // plain track
}

// One buffer, ported. Everything geometric already happened in TrackMesh.h; this
// converts units and handedness and nothing else, which is the whole job of this
// file.
void ATUCoasterRide::PushMeshSection(UProceduralMeshComponent* Target, const FMeshBuffer& M) const
{
	if (Target == nullptr)
	{
		return;
	}
	Target->ClearAllMeshSections();
	if (M.NumTriangles() == 0)
	{
		return;
	}

	TArray<FVector> Pos;
	TArray<FVector> Nrm;
	TArray<FVector2D> UV;
	TArray<int32> Tri;
	Pos.Reserve(static_cast<int32>(M.NumVertices()));
	Nrm.Reserve(static_cast<int32>(M.NumVertices()));
	UV.Reserve(static_cast<int32>(M.NumVertices()));
	Tri.Reserve(static_cast<int32>(M.Index.size()));

	for (std::size_t v = 0; v < M.NumVertices(); ++v)
	{
		Pos.Add(ToLocal(M.Position[v]));
		Nrm.Add(ToLocalDirection(M.Normal[v]));
		UV.Add(FVector2D(M.UV[v].U, M.UV[v].V));
	}

	// THE PART THAT IS NOT UNIT CONVERSION, and the one neither CLAUDE.md nor
	// PHASE0_FINDINGS said until now: M(x,y,z) = (x,-y,z) is a REFLECTION with
	// determinant -1, so it reverses triangle orientation. Mirror the positions
	// and normals and change nothing else and every surface on this ride is
	// inside out — invisible under backface culling, black under a light, with
	// every vertex position perfectly correct.
	//
	// So two indices of every triangle swap. Asserted as a property in
	// test_trackmesh.cpp, where it can be, rather than trusted here where it
	// cannot.
	for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
	{
		Tri.Add(static_cast<int32>(M.Index[t]));
		Tri.Add(static_cast<int32>(M.Index[t + 2]));
		Tri.Add(static_cast<int32>(M.Index[t + 1]));
	}

	Target->CreateMeshSection_LinearColor(0, Pos, Tri, Nrm, UV,
		TArray<FLinearColor>(), TArray<FProcMeshTangent>(), /*bCreateCollision*/ false);
}

FTUTrackStyle ATUCoasterRide::StylePreset(ETUTrackStyleName Which)
{
	FTUTrackStyle S;
	switch (Which)
	{
	case ETUTrackStyleName::SteelClassic:
		S.Name = TEXT("Steel - classic tubular");
		S.RailColour = FLinearColor(0.55f, 0.56f, 0.58f);
		S.SpineColour = FLinearColor(0.16f, 0.35f, 0.55f);
		S.TieColour = FLinearColor(0.22f, 0.24f, 0.26f);
		S.GaugeM = 1.22f;              // the wide end of the real range
		S.RailDiameterM = 0.127f;      // and the fat end of the rail range
		S.SpineDropM = 0.38f;
		S.SpineDiameterM = 0.36f;
		S.TieSpacingM = 0.75f;
		S.TieDiameterM = 0.06f;
		break;
	case ETUTrackStyleName::SteelCompact:
		S.Name = TEXT("Steel - compact / family");
		S.RailColour = FLinearColor(0.70f, 0.72f, 0.75f);
		S.SpineColour = FLinearColor(0.20f, 0.60f, 0.42f);
		S.TieColour = FLinearColor(0.34f, 0.36f, 0.38f);
		// NARROW GAUGE FOR THE SMALL VEHICLES the small-batch preset already
		// ships. 0.76 m is the bottom of the real range and it is where a
		// six-metre car belongs.
		S.GaugeM = 0.78f;
		S.RailDiameterM = 0.09f;
		S.SpineDropM = 0.30f;
		S.SpineDiameterM = 0.22f;
		S.TieSpacingM = 0.60f;
		S.TieDiameterM = 0.04f;
		break;
	default:
		break;                          // the struct's own defaults are Modern
	}
	return S;
}

FTUTrackStyle ATUCoasterRide::ActiveStyle() const
{
	return bUseCustomStyle ? CustomStyle : StylePreset(TrackStyle);
}

void ATUCoasterRide::ApplyTrackStyle()
{
	// NO AUTHORED ASSET. Dynamic instances over an engine material referenced by
	// path, which is the same trick the cars already use for their cube. A style
	// is therefore data in this file rather than a .uasset somebody has to make
	// before anything looks like anything.
	if (!BaseMaterial)
	{
		BaseMaterial = LoadObject<UMaterialInterface>(nullptr,
			TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	}
	if (!BaseMaterial)
	{
		// REPORTED, NOT FAKED. Without a base the sections keep the engine's
		// default grey, which is honest — a track that silently rendered
		// untextured with no explanation would be read as the mesher failing.
		UE_LOG(LogTemp, Warning,
			TEXT("TrackUnlimited: no base material; the track will draw untextured."));
		return;
	}

	const FTUTrackStyle S = ActiveStyle();
	auto Paint = [this](TObjectPtr<UMaterialInstanceDynamic>& Mid,
		UProceduralMeshComponent* Mesh, const FLinearColor& C)
	{
		if (!Mesh) { return; }
		if (!Mid) { Mid = UMaterialInstanceDynamic::Create(BaseMaterial, this); }
		if (!Mid) { return; }
		// The engine base exposes a "Color" parameter. Setting one that does not
		// exist is a no-op rather than an error, so this degrades to plain grey
		// rather than failing if the asset ever changes.
		Mid->SetVectorParameterValue(TEXT("Color"), C);
		Mesh->SetMaterial(0, Mid);
	};

	// THREE MATERIALS FOR THREE BUFFERS, which is why the mesher emits three:
	// running rail is polished where the wheels touch it, spine is painted
	// structure, ties are usually neither.
	Paint(RailMaterial, RailMesh, S.RailColour);
	Paint(SpineMaterial, SpineMesh, S.SpineColour);
	Paint(TieMaterial, TieMesh, S.TieColour);
}

void ATUCoasterRide::RebuildTrackMesh()
{
	if (!bBuildTrackMesh || Track.TotalLength() <= 0.0)
	{
		if (RailMesh) { RailMesh->ClearAllMeshSections(); }
		if (SpineMesh) { SpineMesh->ClearAllMeshSections(); }
		if (TieMesh) { TieMesh->ClearAllMeshSections(); }
		return;
	}

	FMeshSettings Settings;
	Settings.SampleSpacing = MeshSampleSpacingM;
	Settings.Sides = MeshSides;

	// CAP THE OPEN ENDS, AND ONLY IF THERE ARE ANY. On a circuit the first ring
	// and the last are the same ring, so a cap is a disc buried in the seam —
	// invisible, and coplanar with the geometry either side of it, which is what
	// z-fighting is made of.
	//
	// bTrackIsCircuit is MEASURED at rebuild rather than authored, so this
	// follows the geometry rather than somebody's intent. Ties are capped either
	// way: a strut has two free ends whatever the layout does.
	Settings.bCapEnds = !bTrackIsCircuit;

	// The walk is the ONLY thing that touches the track, and it walks with
	// AdvanceFrom. The sweep below has no FTrack at all, which is what makes the
	// O(n^2) trap unreachable rather than merely discouraged — see TrackMesh.h.
	std::vector<FMeshFinding> Findings;
	const FTrackMesh Mesh = BuildTrackMesh(WalkTrack(Track, Settings.SampleSpacing),
		Track.GetHeartlineHeight(), Profile, Settings, &Findings);

	PushMeshSection(RailMesh, Mesh.Rails);
	PushMeshSection(SpineMesh, Mesh.Spine);
	PushMeshSection(TieMesh, Mesh.Ties);

	// AFTER the sections exist, because SetMaterial on a component with no
	// section is a colour assigned to nothing.
	ApplyTrackStyle();

	// REPORTED, NEVER REPAIRED. A curve tighter than half the gauge folds the
	// inner rail through its own axis, and the fix is a wider curve — geometry
	// quietly straightened to fit would read as a layout that works.
	for (const FMeshFinding& F : Findings)
	{
		UE_LOG(LogTUEvents, Warning,
			TEXT("track mesh at %.1f m: %s (curvature %.4f 1/m, inner rail radius %.3f m)"),
			F.S, UTF8_TO_TCHAR(F.What.c_str()), F.Curvature, F.MinRadius);
	}
	UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: track mesh %d vertices, %d triangles at %.2f m / %d sides"),
		static_cast<int32>(Mesh.NumVertices()), static_cast<int32>(Mesh.NumTriangles()),
		MeshSampleSpacingM, MeshSides);
}

void ATUCoasterRide::DrawTrack() const
{
	// THE WIREFRAME SURVIVED PHASE 4, and is not redundant beside the mesh.
	// It draws two things the swept geometry structurally cannot: the HEARTLINE,
	// which is not a physical part of the track at all, and the DEVICE COLOURS,
	// which say which block and which brake each stretch belongs to. A solid
	// track hides both — so this stays on by default and the mesh grows over it.
	//
	// The rails and spine drawn here are inside their own tubes and will be
	// hidden. Left in rather than trimmed, because turning the mesh off has to
	// give back the view every screenshot before Phase 4 was taken with.
	//
	// Everything comes off the frame the walk already has. The previous version
	// called Track.RailCentreAt(S) in here, which re-runs EvaluateAt — O(track
	// length) per call, so the loop was quadratic: about 118 million integrator
	// steps on this 543 m layout, all of it at BeginPlay.
	const double Total = Track.TotalLength();
	const double Step = 0.5;
	const double Heartline = Track.GetHeartlineHeight();

	FTrackFrame Walk = Track.EvaluateAt(0.0);
	FTrackCrossSection Section = CrossSectionAt(Walk, Heartline, Profile);
	double SinceTie = 0.0;

	for (double S = 0.0; S < Total; S += Step)
	{
		const double Next = FMath::Min(S + Step, Total);
		const FTrackFrame NextFrame = Track.AdvanceFrom(Walk, S, Next);
		const FTrackCrossSection NextSection = CrossSectionAt(NextFrame, Heartline, Profile);

		// The RAILS carry the device colour and nothing else does. Heartline, spine
		// and ties keep their own so the geometry stays readable underneath it —
		// a view where everything changes colour at once says less, not more.
		const FColor Rail = RailColourAt(S);

		DrawDebugLine(GetWorld(), ToWorld(Walk.Position), ToWorld(NextFrame.Position),
			FColor(90, 190, 255), true, -1.f, 0, 2.f);                 // heartline
		DrawDebugLine(GetWorld(), ToWorld(Section.LeftRail), ToWorld(NextSection.LeftRail),
			Rail, true, -1.f, 0, 3.f);                                 // running rails
		DrawDebugLine(GetWorld(), ToWorld(Section.RightRail), ToWorld(NextSection.RightRail),
			Rail, true, -1.f, 0, 3.f);
		DrawDebugLine(GetWorld(), ToWorld(Section.SpineCentre), ToWorld(NextSection.SpineCentre),
			FColor(150, 150, 160), true, -1.f, 0, 4.f);                // spine

		SinceTie += Next - S;
		if (SinceTie >= Profile.TieSpacing)
		{
			SinceTie = 0.0;
			DrawDebugLine(GetWorld(), ToWorld(Section.LeftRail), ToWorld(Section.RightRail),
				FColor(200, 160, 90), true, -1.f, 0, 2.f);             // tie across the rails
			DrawDebugLine(GetWorld(), ToWorld(Section.RailCentre), ToWorld(Section.SpineCentre),
				FColor(200, 160, 90), true, -1.f, 0, 2.f);             // down to the spine
		}

		Walk = NextFrame;
		Section = NextSection;
	}
}

void ATUCoasterRide::DrawRideProfile() const
{
	// Each channel is a curve offset from the heartline along the rider's UP,
	// so the track itself is the zero line and the offset is in the rider's own
	// frame rather than the world's. Through an inversion the trace goes with
	// the rider, which is the honest reading — "how hard, which way, for the
	// person in the seat".
	//
	// Per-channel units chosen so the four are comparable at one scale: 1 metre
	// per G, per 10 km/h, per 30 deg/s.
	struct FChannel
	{
		bool bEnabled;
		FColor Colour;
		double PerUnit;
		double FRideSample::*Field;
	};
	// Okabe-Ito, the colourblind-safe qualitative palette — see Docs/UI_CONVENTIONS.md.
	// The previous green/orange pair was the common deuteranopia collision. Blue #0072B2 is
	// deliberately unused: ~3:1 against the background, too weak for a 2 px line.
	const FChannel Channels[] = {
		{bGraphVerticalG, FColor(86, 180, 233), 1.0, &FRideSample::VerticalG},          // sky blue
		{bGraphLateralG, FColor(230, 159, 0), 1.0, &FRideSample::LateralG},             // orange
		{bGraphSpeed, FColor(0, 158, 115), 1.0 / (10.0 / 3.6), &FRideSample::Speed},    // bluish green
		{bGraphRollRate, FColor(204, 121, 167), 1.0 / 30.0, &FRideSample::RollRateDegPerSec}, // reddish purple
	};

	if (Profile_.Samples.size() < 2)
	{
		return;
	}

	// One walk for every channel rather than one per channel: the frames are
	// the expensive part and they do not depend on which trace is being drawn.
	FTrackFrame Walk = Track.EvaluateAt(0.0);
	double PrevS = Profile_.Samples[0].S;
	for (std::size_t i = 1; i < Profile_.Samples.size(); ++i)
	{
		const FRideSample& A = Profile_.Samples[i - 1];
		const FRideSample& B = Profile_.Samples[i];
		const FTrackFrame FrameA = Walk;
		const FTrackFrame FrameB = Track.AdvanceFrom(Walk, PrevS, B.S);

		for (const FChannel& C : Channels)
		{
			if (!C.bEnabled)
			{
				continue;
			}
			const double VA = A.*(C.Field) * C.PerUnit * GraphScale;
			const double VB = B.*(C.Field) * C.PerUnit * GraphScale;
			DrawDebugLine(GetWorld(), ToWorld(FrameA.Position + FrameA.Up * VA),
				ToWorld(FrameB.Position + FrameB.Up * VB), C.Colour, true, -1.f, 0, 2.f);
		}

		Walk = FrameB;
		PrevS = B.S;
	}

	// Where the train gave up, if it did. A marker beats a log line nobody reads.
	if (!Profile_.bCompleted)
	{
		const FTrackFrame Stall = Track.EvaluateAt(Profile_.StalledAtS);
		DrawDebugSphere(GetWorld(), ToWorld(Stall.Position), 200.f, 12, FColor::Red, true, -1.f, 0,
			4.f);
	}
}

void ATUCoasterRide::ServeHolds(std::size_t TrainIndex)
{
	// THE DISPATCHER, entire. Everything else in this file is geometry, physics or
	// drawing; this is the ride control system, and it is four lines because the
	// two layers underneath it already know what they are doing.
	//
	// It asks about the train's CENTRE, because that is what FTrain::Step tests a
	// zone against, and because zones and blocks fall out of the same walk — so a
	// zone never straddles a block boundary and "the block my centre is in" is
	// unambiguous.
	//
	// A device with nobody standing at it is left where it was, which is closed:
	// brakes-on is the resting state, and a device that opens because nobody is
	// asking is a device that fails open.
	// It writes to a DRIVE, not to the track. A command is a request; how fast the
	// drive gets there, and whether it manages to, is the drive's business and the
	// panel's story. This is the whole of the PLC's authority over the ride.
	if (!Signals || !Drives || TrainIndex >= static_cast<std::size_t>(Trains.Num()))
	{
		return;
	}
	FTrain& T = *Trains[TrainIndex];
	const int Z = T.FindHoldZoneAt(T.GetDistance());
	if (Z < 0)
	{
		return;   // not standing at a holding device; nothing to command
	}
	const std::size_t Zi = static_cast<std::size_t>(Z);

	// THE PERMISSIVE IS AN AND, and the interlocking is only one term of it. A real
	// dispatch needs the blocks clear AND the riders aboard AND the restraints
	// locked AND the platform confirmed AND the device about to take the train
	// ready — and on a working ride the block is usually the term that went green
	// first, while an operator was still walking the train. Before the station
	// process existed a train left the instant the track ahead was free, which is a
	// ride with nobody in it.
	//
	// The last term is PRE-LAUNCH, and it is a genuinely different question from
	// the first: CLEAR IS NOT READY. A block with a launch in it can be perfectly
	// empty and still refuse a train, because the launch has not armed.
	if (Signals->CanRelease(TrainIndex, T.GetDistance()) && StationSaysGo(Zi)
		&& DeviceAheadIsReady(T.GetDistance()))
	{
		Drives->Command(Zi, ZoneReleaseSpeed[Z]);
		return;
	}

	// HELD, and the hardware does this in TWO STAGES rather than one glide.
	//
	// A real block brake is two devices sharing a stretch of track. A sensor sits
	// just before the pad; the brake trips as the train ENTERS and clamps a fin
	// under the car, stopping it as hard as it is allowed to — and the limit is
	// RIDER COMFORT, not distance, because the alternative is whiplash. So the
	// train stops wherever that lands. Only then do rubber tyres engage and convey
	// it forward into an acceptable holding position.
	//
	// Commanding a crawl speed says both stages in one number, because a zone
	// closes the gap to its target using its full authority: from 26 m/s the brake
	// bites at everything it has, and from rest the tyres push. The sequence falls
	// out — hard stop, creep to the mark, held.
	//
	// The conveying stage is also what keeps the train INSIDE ITS OWN BLOCK. Brake
	// alone and it stops ~0.3 m past the zone start; the station's start is the
	// circuit's seam, so that leaves its back half in the LAST block, a dwelling
	// train holds two, and three trains deadlock — each denied by the tail of the
	// one in front. Real rides reposition for exactly the same reason.
	// WHERE it parks: the nose HoldNoseClearanceM short of the far end of the
	// block. Measured from the end and applied to the nose, because the thing
	// being prevented is a train protruding into the next zone through a defect —
	// the margin is the whole point, so it is expressed as the margin.
	//
	// Not the middle. Mid-device was the minimum fix for the seam straddle and is
	// arbitrary everywhere else: on the 130 m mid-course brake it parked a train
	// 65 m in with 65 m of empty brake ahead of it, where a real one holds near
	// the exit.
	//
	// AND A SWITCH IS WHAT SAYS SO, not a sum. The rule is "truck forward until the
	// stop mark trips" — no train length, no arc length, no zone extent. A PLC has
	// none of those three and does not need them: the margin was surveyed into the
	// track when the switch was bolted down, and from then on the ride enforces it
	// by geometry rather than by arithmetic.
	//
	// The mark trips on the NOSE, because a span covers a point the moment its
	// front reaches it — the same asymmetry the block counter runs on, and the
	// reason a stop mark measures the thing it is named after.
	//
	// A mark can be tripped by ANY train, since a switch has no idea which. It is
	// unambiguous only because the interlocking guarantees the train standing at
	// this device is the only one that can be in this block. That mutual support is
	// the whole sensor layer's premise; read a mark for a block that is not yours
	// and you are reading somebody else's train.
	//
	// MEASURED on the two-train circuit: the station mark sits at 25.00 m, the nose
	// parks at 25.18 m, and 0.82 m of the metre survives the crawl overshoot.
	// ponytail: 1.5 m/s of crawl, a maintenance-pace guess.
	const double Convey = FMath::Min(ZoneReleaseSpeed[Z], 1.5);

	// A MISSING SWITCH IS NOT PERMISSION TO MOVE. No sensor list, or one that
	// disagrees with the zone list about how many devices there are, means the mark
	// this train is trucking towards cannot be read — and a device that cannot be
	// told when to stop must not be told to go. Same direction as every other rule
	// in this system: fail closed.
	const bool bAtMark = !StopMarks || Zi >= StopMarks->Num() || StopMarks->IsBlocked(Zi);
	Drives->Command(Zi, bAtMark ? 0.0 : Convey);
}

// ONE SCAN, AT A FIXED PERIOD. Called from Tick as many times as the elapsed
// frame is worth, never once per rendered frame — see Tick below for why.
void ATUCoasterRide::SimStep(double DeltaSeconds)
{
	// THE SCAN CYCLE, and it is the whole shape of this function. IEC 61131-3 runs
	// a PLC as read inputs -> execute the program -> write outputs, and each step
	// below is one of those:
	//
	//   1. read every switch on the track, once
	//   2. run the dispatcher for every train against that ONE snapshot
	//   3. let the drives ramp toward what they were just commanded
	//   4. write those outputs to the track
	//   5. step the world, and tell the signalling where everything ended up
	//   6. read the motors back
	//
	// Interleaving 2 with 5 — serving train 1 after train 0 has already moved — is
	// what a game does and what a PLC cannot: a program that re-reads its inputs
	// mid-scan acts on two different worlds in one pass.
	//
	// Overlaps live on blocks rather than on trains, so Signals->Tick is ONCE PER
	// FRAME: ticking per train would age a 5 s overlap in 5/N seconds and nothing
	// anywhere would say so. Same contract for Drives->Tick, which ramps at a rate.
	//
	// Holding is done by GATING THE ZONE, not by declining to integrate. The old
	// version held the train at the station by simply not stepping it, which
	// worked for exactly one train in exactly one place; a zone commanded to zero
	// holds a train anywhere there is a device to hold it, and the station stops
	// being a special case.
	if (StopMarks)
	{
		StopMarks->BeginScan();
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			StopMarks->Cover(Trains[t]->GetRearS(), Trains[t]->GetFrontS(),
				bTrackIsCircuit, Track.TotalLength());
		}
		StopMarks->EndScan();
	}
	// Session time, for the event log. Not the wall clock: what matters is how long
	// before the stop something happened, and a relative figure survives a paused
	// PIE session where a wall clock does not.
	RideClock += DeltaSeconds;

	// AFTER the scan and the physics below would be wrong: this reads the state
	// the frame ENDED in, and reading it at the top means every transition is
	// reported one frame after it happened. Placed here, against the same values
	// the panel is about to draw, so the log and the screen cannot disagree.
	SampleRestraints();
	LogTransitions();

	// THE SCAN'S FINGERPRINT. Same fields, same order, as the prototype suite's
	// digest — two sessions on the same preset, left alone, must agree at the same
	// scan number. Physics AND control state, because either can drift alone: a
	// train in the right place with the wrong block state is still a different run.
	++ScanNumber;
	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		SimFingerprint.Add(Trains[t]->GetDistance());
		SimFingerprint.Add(Trains[t]->GetSpeed());
	}
	if (Signals)
	{
		for (std::size_t k = 0; k < Signals->NumBlocks(); ++k)
		{
			SimFingerprint.Add(static_cast<int>(Signals->GetState(k)));
		}
	}
	if (Drives)
	{
		for (std::size_t z = 0; z < Drives->Num(); ++z)
		{
			SimFingerprint.Add(Drives->Output(z));
		}
	}

	// TIER 3, LAST, AFTER THE FINGERPRINT IS TAKEN. That placement is the whole
	// claim: everything hashed above is the ride, and the show layer is a
	// subscriber to it that cannot reach back. Asserted rather than asserted-to —
	// test_twotrains.cpp runs the same circuit with and without this and the two
	// digests are equal to the bit.
	PublishShowEvents();

	// The Details-panel checkbox, so the stop can be tripped without playing. Read
	// here rather than in a PostEditChangeProperty because it has to work in PIE.
	if (bEmergencyStop && Drives && !Drives->IsEmergencyStopped())
	{
		Drives->PressEmergencyStopButton("operator");
	}
	// THE CONTROLLER SCANS, and its watchdog is the overrun the accumulator
	// already detects one layer up. Reporting that as a note rather than a trip
	// was the machine having a symptom with nowhere to put it.
	// THE SCAN OVERRUN IS NOT A WATCHDOG EVENT, and wiring it to one was a
	// mistake that killed the ride.
	//
	// Two different things were conflated. A real PLC watchdog means THE
	// PROGRAM'S OWN LOGIC took longer than its scan period — a controller fault,
	// and rightly latching. The accumulator's overrun means THE HOST CANNOT
	// RENDER FAST ENOUGH to give the simulation its share of wall-clock time.
	// The controller is executing every scan perfectly; there are simply fewer of
	// them per second.
	//
	// Measured, which is how the difference became obvious: the editor dropped to
	// ~3 fps, so every 333 ms frame could only afford 100 ms of simulation and
	// dropped 233 ms. That is correct graceful degradation — the ride runs at 30%
	// speed and remains bit-identical, because the step is fixed. Faulting on it
	// meant a machine that renders slowly is a machine whose ride will not run,
	// which is nonsense.
	//
	// AND THE BAR WAS THIS LOW: the 3 fps was the print-screen overlay taking
	// priority while somebody took a SCREENSHOT of the panel. Observing the ride
	// killed it, permanently, and any hitch would have done — an alt-tab, a
	// shader compile, something else waking up. Not an edge case worth a note; a
	// thing that would have happened to everybody, constantly.
	//
	// So the overrun stays what it was: a reported performance note. The PLC's
	// watchdog stays for a genuine one, which in this model would be the scan's
	// own logic exceeding its period — straight-line code, so never yet.
	const bool bWasFaulted = Plc.IsFaulted();
	Plc.Scan(DeltaSeconds, /*bOverran*/ false);
	bScanOverranThisFrame = false;
	// A CONTROLLER FAULTING IS THE LOUDEST THING THAT CAN HAPPEN TO A RIDE, and
	// it was silent. The watchdog tripped, the ride stopped being commanded, and
	// the only trace was three violations a second and a half later with nothing
	// connecting them to a cause.
	if (Plc.IsFaulted() && !bWasFaulted)
	{
		LogEvent(FString::Printf(TEXT("PLC FAULT — %s"),
			UTF8_TO_TCHAR(Plc.FaultReason())));
	}

	ServeStations(DeltaSeconds);

	// THE PROGRAM RUNS ONLY IF THE MACHINE IS RUNNING IT. Not a stop — a
	// controller with no permission simply commands nothing, and a device with
	// no command falls to its safe state exactly as it would with the cabinet
	// unplugged. The stop does not come from here, and the E-stop below is
	// untouched by it: that separation is constraint 7 and it is asserted in
	// test_plcunit.cpp against the real drive layer.
	if (Plc.OutputsEnabled())
	{
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			ServeHolds(static_cast<std::size_t>(t));
		}
	}
	else if (Drives)
	{
		// A CONTROLLER THAT IS NOT COMMANDING MUST NOT LEAVE EVERYTHING RUNNING,
		// and the first version of this did exactly that.
		//
		// Skipping ServeHolds does not mean "no command" — it means the LAST
		// command stands. On a ride that has just opened that is each zone's
		// PRESET, so every brake sat at its release speed and all three trains
		// left the moment the watchdog tripped. Three signalling violations, 1.47
		// seconds in, from a controller that had faulted and was doing nothing.
		//
		// The header of PlcUnit.h claimed a device with no command falls to its
		// safe state "exactly as it would with the cabinet unplugged". That was
		// aspirational: nothing made it true. This does.
		//
		// In a real cabinet the PLC's output card DE-ENERGISES when it faults —
		// the drives lose their enable and the brakes, being spring-applied,
		// bite. Commanding zero is that, expressed in the one authority this
		// layer has.
		for (std::size_t z = 0; z < Drives->Num(); ++z)
		{
			Drives->Command(z, 0.0);
		}
	}

	// One drive, every train's copy of the zone. Zones live on each FTrain, so
	// before drives existed every train carried its own private idea of what every
	// brake on the ride was doing; a real brake is ONE device acting on whatever is
	// in it, and this is what makes them agree.
	if (Drives)
	{
		Drives->Tick(DeltaSeconds);
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			for (std::size_t z = 0; z < Drives->Num(); ++z)
			{
				Trains[t]->SetZoneTargetSpeed(z, Drives->Output(z));

				// THE PAD TRACKS THE SAME COMMAND, and bites at its own rate.
				//
				// Not a second command from the PLC, and that is deliberate: a
				// block brake is commanded to a SPEED, and the pad and the tyres
				// are two ways of getting there. Above the commanded speed the
				// pad is what removes the energy, at whatever the hardware was
				// specified at; below it the tyres push. The four-state sequence
				// ServeHolds describes — bite, stop, release, convey — then falls
				// out of one number instead of needing a state machine, because
				// "release" is just the train no longer being above the limit.
				//
				// A ZONE WITH NO PAD KEEPS ITS NEGATIVE LIMIT and is untouched,
				// which is every zone on every preset until somebody authors one.
				if (z < static_cast<std::size_t>(ZoneBrakeDecel.Num())
					&& ZoneBrakeDecel[static_cast<int32>(z)] > 0.0)
				{
					Trains[t]->SetZoneBrakeLimit(z, Drives->Output(z));
				}
				// And how much of that the hardware is actually producing, for the
				// same reason and by the same route: one device, one number, every
				// train's copy hearing the same thing about it. 1.0 unless
				// something has injected a failure, so this is free until it is not.
				Trains[t]->SetZoneHealth(z, Drives->DeliveredFraction(z));
			}
		}
	}

	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		Trains[t]->Step(DeltaSeconds);

		// The return is the only record a signalling violation leaves besides the
		// counter, so it is read rather than discarded. With two trains it is also
		// the collision report.
		if (Signals && !Signals->Update(static_cast<std::size_t>(t),
			Trains[t]->GetRearS(), Trains[t]->GetFrontS()))
		{
			UE_LOG(LogTemp, Error,
				TEXT("TrackUnlimited: SIGNALLING VIOLATION — train %d entered a block that "
					"was not clear, at %.1f m."),
				t, Trains[t]->GetDistance());

			// AND IT STOPS THE RIDE. Until the E-stop existed this was a log line
			// and nothing else: the interlocking detected the one thing it is for
			// and then let the ride carry on into it. A violation is the definition
			// of an E-stop condition on real hardware.
			LogEvent(FString::Printf(TEXT("SIGNALLING VIOLATION — train %d at %.0f m"),
				t, Trains[t]->GetDistance()));
			if (Drives && Drives->TripEmergencyStop("signalling violation"))
			{
				bEmergencyStop = true;
			}
		}
	}
	if (Signals)
	{
		Signals->Tick(DeltaSeconds);
	}
	CrossCheckOccupancy();

	// The motors report back: what each is actually turning at, and how much of its
	// authority that is taking. A drive with no train on it goes unreported, which
	// is how EndFeedback learns it is free-running rather than slipping against a
	// stale reading from the last train through.
	if (Drives)
	{
		Drives->BeginFeedback();
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			const double At = Trains[t]->GetDistance();
			for (std::size_t z = 0; z < Drives->Num(); ++z)
			{
				if (Trains[t]->IsInZone(z, At))
				{
					Drives->ReportFeedback(z, Trains[t]->GetSpeed(), Trains[t]->GetZoneLoad(z));
				}
			}
		}
		Drives->EndFeedback();

		// A drive that trips is a ride an operator has to go and look at, so it says
		// so once rather than every frame. Reported, never acted on: what a ride does
		// about a failed drive is an E-stop policy and the PLC's decision, not a
		// property of the motor.
		for (std::size_t z = 0; z < Drives->Num(); ++z)
		{
			if (Drives->IsFaulted(z) && !ReportedDriveFault.Contains(static_cast<int32>(z)))
			{
				ReportedDriveFault.Add(static_cast<int32>(z));
				UE_LOG(LogTemp, Error,
					TEXT("TrackUnlimited: DRIVE FAULT — zone %d commanded %.1f m/s, output "
						"%.1f, motor reading %.1f, at full torque and not gaining."),
					static_cast<int32>(z), Drives->Read(z).Commanded, Drives->Read(z).Output,
					Drives->Read(z).Actual);

				// A motor at full torque going nowhere is a stalled lift, a failed
				// launch or a brake that is not biting, and every one of those is a
				// ride an operator has to walk out to. The DRIVE still only reports
				// — deciding what a ride does about a failed motor is the PLC's job,
				// and this is the PLC doing it.
				LogEvent(FString::Printf(TEXT("DRIVE FAULT — zone %d, full torque, not gaining"),
					static_cast<int32>(z)));
				if (Drives->TripEmergencyStop("drive fault"))
				{
					bEmergencyStop = true;
				}
			}
		}
	}
}

// THE NEAR PLANE IS ONE GLOBAL, and that is UE's answer rather than a limitation
// of this code: UCameraComponent carries an ortho near plane and no perspective
// one, because FMinimalViewInfo::PerspectiveNearClipPlane defaults negative
// meaning "use GNearClippingPlane". SetNearClipPlaneGlobals is what the engine's
// own r.SetNearClipPlane console command calls, and it moves the render thread's
// copy as well — writing GNearClippingPlane directly moves only half of it.
//
// One global is not a compromise here. There is one camera.
//
// GUARDED, because this is called every frame and the setter flushes rendering
// commands. A near plane that has not moved must not cost a flush.
static void SetNearPlaneMetres(double Metres)
{
	// The engine clamps its own console command to 1 cm, so match it rather than
	// discovering the floor by rendering through it.
	const float Cm = FMath::Max(1.f, static_cast<float>(Metres * MetresToUU));

	// A PROPORTIONAL GUARD, not an absolute one. The near plane is
	// distance * 0.002, so it moves every frame the camera does — and an absolute
	// 0.05 cm tolerance meant a render command enqueued on every one of them.
	// It is only an enqueue rather than a flush, so this was not the performance
	// problem it looked like, but a near plane exists to be the right ORDER OF
	// MAGNITUDE and updating it by half a percent buys exactly nothing.
	if (Cm < GNearClippingPlane * 0.8f || Cm > GNearClippingPlane * 1.25f)
	{
		SetNearClipPlaneGlobals(Cm);
	}
}

// [F2] — EVERY OVERLAY OFF, AND BACK EXACTLY AS IT WAS.
//
// A MASTER GATE RATHER THAN A SNAPSHOT. Saving the eight toggles, clearing them
// and putting them back is the obvious version and it is wrong: anything that
// touches one while hidden gets overwritten on restore, and the symptom is a
// panel coming back that somebody had deliberately turned off. One bool
// consulted at draw time restores the previous state for free, because nothing
// was ever changed.
//
// The wireframe and the ride-profile trace are PERSISTENT debug lines drawn once
// at BeginPlay rather than every frame, so hiding them means flushing and
// unhiding means redrawing. Everything else is a per-frame gate.
void ATUCoasterRide::ToggleOverlays()
{
	bHideOverlays = !bHideOverlays;

	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);
	}
	if (!bHideOverlays)
	{
		// bDrawTrack is still consulted, so [F2] does not turn on a wireframe
		// somebody had switched off in the Details panel.
		if (bDrawTrack) { DrawTrack(); }
		DrawRideProfile();
	}

	// SAID ONCE, IN THE LOG, because the banner that would normally carry a hint
	// is one of the things just hidden — and a screenshot mode that printed a
	// caption over the screenshot would defeat itself.
	UE_LOG(LogTUEvents, Log, TEXT("overlays %s  [F2]"),
		bHideOverlays ? TEXT("hidden") : TEXT("shown"));
}

void ATUCoasterRide::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (Trains.Num() == 0 || !Trains[0].IsValid())
	{
		return;
	}
	FTrain* const Train = Trains[0].Get();   // the rider's train: camera, readout

	// ===================== A FIXED SCAN PERIOD =====================
	//
	// The scan used to run ONCE PER RENDERED FRAME, which is wrong twice over.
	//
	// IT IS UNFAITHFUL. A PLC scans on a fixed period; it does not scan faster
	// because the graphics card is idle. Running it at frame rate meant the ride's
	// control system executed at 144 Hz on one machine and 40 Hz on another, and
	// every rate in it moved with that — edge detection, restraint travel, overlap
	// ageing, drive ramps. The already-documented "a train can cross a sensor
	// inside one scan" limitation was quietly WORSE on a slow machine, which is
	// exactly backwards from how a limitation should behave.
	//
	// AND IT IS NOT REPRODUCIBLE. Two runs of the same session diverge on the
	// first frame, so nothing can be recorded and replayed — and a fault-injection
	// scenario that reproduces differently every run cannot prove anything to
	// anybody. Everything else about replay is downstream of this one change.
	//
	// It also brings the actor into line with the prototype suites, which have
	// always stepped at a fixed 1/240 s. The canonical figures were never affected
	// because they come from those; it was only the thing you actually played that
	// varied.
	const double Step = 1.0 / static_cast<double>(FMath::Max(1, SimHz));

	// THE FIRST FRAME AFTER A LOAD IS NOT AN OVERRUN. It carries the whole level
	// load — hundreds of milliseconds of it — and reporting that as a missed scan
	// deadline is the panel's first lie of the session: nothing was late, because
	// nothing had started. A controller powers up and begins scanning; it does not
	// log the time before power-on as lost.
	//
	// Swallowed rather than clamped, because there is no correct amount of that
	// frame to simulate. The ride begins now.
	// WAIT FOR A PLAUSIBLE FRAME BEFORE THE RIDE STARTS, rather than swallowing
	// exactly one. Swallowing one was not enough: PIE takes several frames to
	// settle — shaders, streaming, the first draw — and the SECOND frame still
	// carried 300 ms of it, overran, and tripped the PLC watchdog. None of that
	// is ride time, and a controller coming online is not one missing a deadline.
	//
	// Self-calibrating rather than a frame count, because how many frames a load
	// takes is a property of the machine and the threshold is a property of what
	// a frame plausibly is.
	if (!bScanStarted)
	{
		if (DeltaSeconds > 0.1)
		{
			SimAccumulator = 0.0;
			return;
		}
		bScanStarted = true;
		SimAccumulator = 0.0;
		return;
	}

	// PAUSED, SCALED, OR STEPPED — and all three change how much TIME arrives,
	// never the step itself. The scan period stays 1/SimHz, so a quarter-speed
	// run is the same sequence of scans delivered more slowly and the digest
	// still matches a real-time run. Scaling the step instead would change every
	// rate inside the controller and make slow motion a different ride.
	if (bSimPaused)
	{
		// A single step is COUNTED rather than timed, so one press is exactly one
		// scan whatever the frame rate — which is the whole reason to want it.
		if (StepsOwed > 0)
		{
			SimStep(Step);
			--StepsOwed;
		}
		SimAccumulator = 0.0;
		return;
	}
	SimAccumulator += DeltaSeconds * static_cast<double>(TimeScale);

	int32 Ran = 0;
	while (SimAccumulator >= Step && Ran < MaxStepsPerFrame)
	{
		SimStep(Step);
		SimAccumulator -= Step;
		++Ran;
	}

	// SCAN OVERRUN. A frame so long that the backlog exceeds the cap — a hitch, a
	// breakpoint, a level load. THE BACKLOG IS DROPPED rather than worked off,
	// because catching up means running the ride faster than real time, and a ride
	// that fast-forwards through a hitch can skip a train past a block boundary:
	// the one failure this whole layer exists to prevent.
	//
	// Real controllers treat a missed scan deadline as a fault to report rather
	// than time to make up, which is the same call for the same reason.
	if (SimAccumulator >= Step)
	{
		++ScanOverruns;
		bScanOverranThisFrame = true;   // the PLC's watchdog reads this next scan

		// THE TOTAL IS THE NUMBER THAT MATTERS, not the count. One overrun is a
		// hitch; a count says how many times without saying how much ride went
		// missing, and 545 of them reads as "a bit stuttery" when it can be a
		// minute of simulation that never happened.
		//
		// This is what decides whether a run can be JUDGED. Dropped time is not
		// slow motion — the scans never ran, so a train really did move further
		// between two scans than it should have, and any behaviour somebody
		// watched across a drop is behaviour of a ride the model did not compute.
		ScanTimeDroppedS += SimAccumulator;
		WorstOverrunS = FMath::Max(WorstOverrunS, SimAccumulator);

		LogEvent(FString::Printf(
			TEXT("SCAN OVERRUN — dropped %.0f ms (overrun %d, %.2f s dropped in total)"),
			SimAccumulator * 1000.0, ScanOverruns, ScanTimeDroppedS));
		SimAccumulator = 0.0;
	}

	// ---- Everything below is DRAWING, and runs once per rendered frame. It reads
	// the state the last scan left and never advances anything.
	if (!bHideOverlays)
	{
		DrawRestraints();
		DrawBlockMarkers();
	}

	// Where the rider is sitting, which is a real choice now that cars differ.
	const double SeatOffset = RiderPosition * TrainLengthM * 0.5;
	const FTrackFrame& Frame = Train->GetFrameAt(SeatOffset);
	const FQuat Rotation = ToWorldRotation(Frame);

	// THE BODY OF THE TRAIN, ONE BOX PER SAMPLE POINT — and the boxes ABUT into
	// one continuous body rather than standing apart as vehicles.
	//
	// A SAMPLE POINT IS NOT A CAR, and treating it as one is what the first
	// version did. SampleCount() is 9 because that is what the mean-height
	// gravity integration needs; it has nothing to do with how many vehicles a
	// train has, and FTrainConfig has no car count at all — it models a train as
	// a LENGTH. Dividing that length into nine and leaving a gap between each
	// piece made a 15 m train look accidentally plausible (nine 1.5 m boxes) and
	// the 6 m small-batch vehicle look like nine playing cards standing on edge.
	// Same code, and only one of the two presets exposed it.
	//
	// So the boxes are the sample SPACING long and touch end to end. What is
	// drawn is then exactly what the physics knows: a body of TrainLength,
	// following the track, with no invented vehicle count.
	//
	// Frames come from the train rather than from Track.EvaluateAt, which is
	// O(track length) a call and would be nine of those per train every frame.
	{
		const int32 Points = Train->NumSamplePoints();
		const int32 Total = Points * Trains.Num();
		// Nose-to-tail over Points samples is Points-1 gaps.
		const double Spacing = (Points > 1 && TrainLengthM > 0.f)
			? static_cast<double>(TrainLengthM) / (Points - 1) : 2.4;
		if (Cars->GetInstanceCount() != Total)
		{
			Cars->ClearInstances();
			for (int32 i = 0; i < Total; ++i)
			{
				Cars->AddInstance(FTransform::Identity, true);
			}
		}

		// Generic, per constraint 5: a shade under a metre and a half across, and
		// CHEST HEIGHT ON A SEATED RIDER rather than head height.
		//
		// 0.9 m, and the number is load-bearing: the heartline is 1.1 m above the
		// rail centreline, so a body any taller than that swallows the point the
		// ride camera sits at and the rider spends the lap inside a box. A real
		// car body comes up to about the chest for the same reason — you have to
		// be able to see out of it.
		const double BodyWidth = 1.4;
		const double BodyHeight = 0.9;
		const FVector CarScale(Spacing, BodyWidth, BodyHeight);

		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			for (int32 i = 0; i < Points; ++i)
			{
				const int32 Slot = t * Points + i;
				const FTrackFrame& CarFrame = Trains[t]->GetSamplePoint(i);

				// SITS ON THE RAILS, WHICH IS NOT THE SAME AS CENTRED ON THEM.
				// The first version put the box's CENTRE at the rail plane, so
				// half of it was buried in the ties and the spine — which is
				// exactly what the screenshot showed. A cube's origin is its
				// middle, so the body has to be lifted by half its height.
				//
				// The rider still sits at the heartline, which is above this.
				// That distinction is the entire reason the heartline model
				// exists, so the slice shows it rather than putting both in one
				// place.
				const FVec3 OnRails = CarFrame.Position
					- CarFrame.Up * (Track.GetHeartlineHeight() - BodyHeight * 0.5);
				Cars->UpdateInstanceTransform(Slot,
					FTransform(ToWorldRotation(CarFrame), ToWorld(OnRails), CarScale), true,
					Slot == Total - 1, true);
			}
		}
	}

	// EACH MODE KEEPS ITS OWN CAMERA, so Build to Ride to Build returns you where
	// you were. Cheap, obvious, and never added later because it is never the most
	// urgent bug — but it is the difference between a viewport somebody works in
	// and one they fight.
	//
	// BEFORE the chain below, not inside it: the swap has to happen whichever mode
	// is being entered, and putting it between two branches would run Free and
	// then fall into the rest of the chain as well.
	if (CameraMode != LastCameraMode)
	{
		CameraRigs.For(static_cast<int>(LastCameraMode)) = Orbit;
		Orbit = CameraRigs.For(static_cast<int>(CameraMode));
		LastCameraMode = CameraMode;
	}

	// LOOKING IS A DRAG NOW, because the cursor is free.
	//
	// A camera that turned whenever the mouse moved is fine when the mouse has
	// nothing else to do, and impossible once there are rows to click: every
	// click becomes a gamble on where the panel went while you were reaching for
	// it. Holding the right button is what every editor with both a viewport and
	// a panel does, and it costs nothing to somebody who only wants to look.
	//
	// ZEROED RATHER THAN SKIPPED, so a drag starts from where the mouse is
	// instead of applying everything it travelled while the button was up.
	ApplyCursorMode();
	PollMovementKeys();
	if (!bDraggingLook && Session.Mode() != EAppMode::Ride)
	{
		LookYaw = 0.f;
		LookPitch = 0.f;
	}

	if (CameraMode == ETUCameraMode::Free)
	{
		// Flown by hand while the ride carries on without you. Seeded from
		// wherever the camera already was, so switching in does not teleport.
		if (!bFreeInitialised)
		{
			FreeLocation = Camera->GetComponentLocation();
			FreeRotation = Camera->GetComponentRotation();
			FreeRotation.Roll = 0.f;
			bFreeInitialised = true;
		}

		// FREE-FLY IS HEAD-TURNING: mouse down looks down, uninverted.
		const float PitchSign = bInvertLookY ? -1.f : 1.f;
		FreeRotation.Yaw += LookYaw * 2.2f;
		FreeRotation.Pitch =
			FMath::Clamp(FreeRotation.Pitch + LookPitch * 2.2f * PitchSign, -87.f, 87.f);
		FreeRotation.Roll = 0.f; // a free camera that rolls is a lost camera

		const FVector Forward = FreeRotation.Vector();
		const FVector Right = FRotationMatrix(FreeRotation).GetScaledAxis(EAxis::Y);
		const float Speed = FreeCameraSpeedMs * MetresToUU * (bBoost ? 5.f : 1.f) * DeltaSeconds;
		FreeLocation += (Forward * MoveForward + Right * MoveRight) * Speed
			+ FVector(0.f, 0.f, MoveUp * Speed);

		Camera->SetWorldLocationAndRotation(FreeLocation, FreeRotation.Quaternion());
	}
	else if (CameraMode == ETUCameraMode::Orbit)
	{
		// Around a point, which is what editing wants. Free-fly is for going
		// somewhere; orbit is for looking at what you are already at.
		//
		// The state and the arithmetic are in Prototypes/Shell/CameraRig.h and
		// are tested there — this converts at the same boundary as everything
		// else, so the framing maths never learns which way round Unreal's Y is.
		if (!bOrbitFramed) { FrameWholeTrack(); }

		Orbit.AddYaw(LookYaw * 2.2);
		// ORBIT IS SUBJECT-DRAGGING, SO PITCH IS NEGATED. You are pulling the
		// thing you are looking at, not turning your head: mouse down swings the
		// camera up and over the top of the subject, which is what every DCC tool
		// does and what the raw axis did backwards.
		Orbit.AddPitch(-LookPitch * 2.2 * (bInvertLookY ? -1.0 : 1.0));   // clamped, not wrapped
		Orbit.Pan(MoveRight * DeltaSeconds * 120.0, MoveUp * DeltaSeconds * 120.0);

		const FCamVec P = Orbit.Position();
		const FVector World = ToLocal(FVec3{P.X, P.Y, P.Z}) + GetActorLocation();
		const FVector Focus = ToLocal(FVec3{Orbit.Focus.X, Orbit.Focus.Y, Orbit.Focus.Z})
			+ GetActorLocation();
		Camera->SetWorldLocationAndRotation(World, (Focus - World).Rotation().Quaternion());

		// THE NEAR PLANE FOLLOWS THE CAMERA. A 90 m lift hill and a 2 cm bolt
		// cannot share a fixed one: set it for the bolt and depth precision at the
		// top of the hill is gone; set it for the hill and the restraint in front
		// of a rider is clipped away.
		const FDepthRange D = DepthRangeFor(Orbit.Distance, Orbit.Distance);
		SetNearPlaneMetres(D.Near);
	}
	else if (CameraMode == ETUCameraMode::Rider)
	{
		// THE HEARTLINE IS THE RIDER'S HEART, NOT THEIR EYES, and the camera wants
		// the second one. Sitting exactly on it puts the view at chest height,
		// inside the car body — which is where it was.
		//
		// ADDED HERE AND NOWHERE ELSE. The heartline is what FeltG is computed
		// about and what the banking is built around; moving it to suit a camera
		// would change every G number on the ride to fix a framing problem. This
		// is a cosmetic offset on the view alone and touches no physics.
		Camera->SetWorldLocationAndRotation(
			ToWorld(Frame.Position + Frame.Up * RiderEyeAboveHeartlineM), Rotation);
		// Centimetres in a seat, or the restraint in front of the rider is clipped
		// off — the other end of the same trade the orbit camera makes. 2 cm is
		// twice the engine's own 1 cm floor, so it survives the clamp.
		SetNearPlaneMetres(0.02);
	}
	else
	{
		// Behind and above, held LEVEL with the world. The old non-ride view
		// took the rider's rotation and simply stepped back from it, which meant
		// it turned upside down through the loop — disorienting rather than
		// dramatic, and it made the inversion hard to actually look at.
		//
		// Direction of travel flattened into the horizontal plane. Through a
		// vertical section there is no horizontal component to speak of, so the
		// last good one is held rather than letting the camera spin.
		const FVector Target = ToWorld(Train->GetFrame().Position);
		FVector Forward = ToWorldRotation(Train->GetFrame()).GetForwardVector();
		Forward.Z = 0.f;
		if (Forward.SizeSquared() > 0.05f)
		{
			LastChaseForward = Forward.GetSafeNormal();
		}

		const FVector Desired = Target - LastChaseForward * (ChaseDistanceM * MetresToUU)
			+ FVector(0.f, 0.f, ChaseHeightM * MetresToUU);

		// Critically-damped-ish follow. A rigid offset reads as a camera bolted
		// to the train; lagging it is what makes the speed legible.
		//
		// ponytail: one exponential smooth, no spring, no collision. Add a
		// spring arm the day the camera needs to avoid terrain.
		if (!bChaseInitialised)
		{
			ChaseLocation = Desired;
			bChaseInitialised = true;
		}
		const float Alpha = 1.f - FMath::Exp(-6.f * DeltaSeconds);
		ChaseLocation = FMath::Lerp(ChaseLocation, Desired, Alpha);

		Camera->SetWorldLocationAndRotation(ChaseLocation,
			(Target - ChaseLocation).ToOrientationQuat());
	}

	if (bShowTelemetry && !bHideOverlays && GEngine)
	{
		// What the RIDER feels, at whichever row they are sitting in — not the
		// train's centre. That is the whole point of choosing a seat.
		const FGForces G = Train->GetForcesAt(SeatOffset);
		const double S = Train->GetDistance();
		GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::White,
			FString::Printf(TEXT("%6.1f km/h    %5.1f m along %.0f m    height %5.1f m"),
				Train->GetSpeed() * 3.6, S, Track.TotalLength(), Frame.Position.Z));
		GEngine->AddOnScreenDebugMessage(2, 0.f,
			G.Vertical > 4.5 || G.Vertical < -1.0 ? FColor::Red : FColor::Green,
			FString::Printf(TEXT("vertical %+5.2f G    lateral %+5.2f G    fore-aft %+5.2f G"),
				G.Vertical, G.Lateral, Train->GetTangentialG()));

		// Front and back, when there is a train to have ends. Every car shares
		// one speed, so the spread is purely which curvature each one is on —
		// and it is the whole reason people queue for the back row.
		if (TrainLengthM > 0.f)
		{
			const double Half = TrainLengthM * 0.5;
			GEngine->AddOnScreenDebugMessage(6, 0.f, FColor(200, 200, 120),
				FString::Printf(TEXT("%.0f m train:  front %+5.2f G    back %+5.2f G"),
					TrainLengthM, Train->GetForcesAt(+Half).Vertical,
					Train->GetForcesAt(-Half).Vertical));
		}
		GEngine->AddOnScreenDebugMessage(3, 0.f, FColor::Silver,
			S >= BrakeStartS ? TEXT("BRAKE RUN") : TEXT("on course"));

		// The block row. Making the causal chain VISIBLE is the pillar, not
		// decoration — a block holding its overlap is why the train in the
		// station is not moving, and that should be readable off the screen
		// rather than inferred.
		if (Signals)
		{
			FString Row;
			for (std::size_t b = 0; b < Signals->NumBlocks(); ++b)
			{
				const TCHAR* Tag = TEXT("CLEAR");
				if (Signals->GetState(b) == EBlockState::Occupied) { Tag = TEXT("OCCUPIED"); }
				else if (Signals->GetState(b) == EBlockState::Buffer) { Tag = TEXT("BUFFER"); }

				Row += FString::Printf(TEXT("[%d %s"), static_cast<int32>(b), Tag);
				if (Signals->GetState(b) == EBlockState::Buffer)
				{
					Row += FString::Printf(TEXT(" %.1fs"), Signals->GetBufferRemaining(b));
				}
				Row += TEXT("]  ");
			}
			// "Held" is now a property of a train standing at a device, not a flag:
			// a train is held when it is at a holding place and its permissive is
			// refusing. Reported per train, because with two of them "the ride is
			// held" is not a fact about the ride.
			FString HeldRow;
			for (int32 t = 0; t < Trains.Num(); ++t)
			{
				const double At = Trains[t]->GetDistance();
				if (Trains[t]->FindHoldZoneAt(At) >= 0
					&& !Signals->CanRelease(static_cast<std::size_t>(t), At))
				{
					HeldRow += FString::Printf(TEXT("train %d HELD at %.0f m   "), t, At);
				}
			}
			GEngine->AddOnScreenDebugMessage(8, 0.f,
				HeldRow.IsEmpty() ? FColor(120, 200, 140) : FColor(255, 176, 32), Row);

			if (!HeldRow.IsEmpty())
			{
				GEngine->AddOnScreenDebugMessage(9, 0.f, FColor(255, 176, 32),
					HeldRow + TEXT("— dispatch permissive not satisfied"));
			}
			if (Signals->Violations() > 0)
			{
				GEngine->AddOnScreenDebugMessage(10, 0.f, FColor::Red,
					FString::Printf(TEXT("%d SIGNALLING VIOLATION(S)"),
						static_cast<int32>(Signals->Violations())));
			}

			// A caught train is a THIRD outcome, and it needs its own words. The
			// device did its job and the ride still failed: nobody is in danger and
			// the layout is still wrong. Reporting it as "held" would read as the
			// signalling working; reporting nothing would read as the ride working.
			FString CaughtRow;
			for (int32 t = 0; t < Trains.Num(); ++t)
			{
				if (Trains[t]->GetRollbacksCaught() > 0)
				{
					CaughtRow += FString::Printf(TEXT("train %d %s at %.0f m   "), t,
						Trains[t]->IsHeldByCatch() ? TEXT("HELD BY ANTI-ROLLBACK")
												   : TEXT("was caught"),
						Trains[t]->GetDistance());
				}
			}
			if (!CaughtRow.IsEmpty())
			{
				GEngine->AddOnScreenDebugMessage(11, 0.f, FColor(255, 90, 60),
					CaughtRow + TEXT("— the catch worked, the layout did not"));
			}

			// WHAT IS HOLDING THE DISPATCH, named. This is the whole reason the
			// station's gates are modelled one at a time instead of as a single
			// "platform ready" flag: "the station is not ready" is useless to
			// somebody standing on it, and "restraints not locked" is somewhere to
			// go and look. It is also the first piece of the control panel that
			// exists, and it is a view over data the PLC layer already had.
			FString StationRow;
			for (const FTUPlatform& P : Platforms)
			{
				static const TCHAR* PhaseName[] = {
					TEXT("empty"), TEXT("arriving"), TEXT("unloading"), TEXT("loading"),
					TEXT("securing"), TEXT("READY"), TEXT("departing")};
				const int32 Ph = static_cast<int32>(P.Process.GetPhase());
				StationRow += FString::Printf(TEXT("zone %d %s"), P.Zone,
					Ph >= 0 && Ph < UE_ARRAY_COUNT(PhaseName) ? PhaseName[Ph] : TEXT("?"));

				const FString Holding = UTF8_TO_TCHAR(P.Process.WhatIsHolding());
				if (!Holding.IsEmpty())
				{
					StationRow += FString::Printf(TEXT(" — %s"), *Holding);
				}
				else if (P.Process.IsReadyToDispatch())
				{
					// Ready but still standing means the OTHER half of the AND is
					// the one saying no, which is exactly the distinction an
					// operator wants and cannot otherwise see.
					StationRow += Signals && !Signals->CanRelease(0, ZoneSpans[P.Zone].StartS)
						? TEXT(" — waiting on the block ahead") : TEXT("");
				}
				StationRow += TEXT("   ");
			}
			if (!StationRow.IsEmpty())
			{
				GEngine->AddOnScreenDebugMessage(12, 0.f, FColor(120, 170, 255),
					StationRow + (bManualDispatch
						? TEXT("   [Space] dispatch") : TEXT("   auto")));
			}

			// Loudest thing on screen, and it stays until somebody resets it. A
			// stop nobody has looked at has not been dealt with.
			if (Drives && Drives->IsEmergencyStopped())
			{
				GEngine->AddOnScreenDebugMessage(13, 0.f, FColor::Red,
					FString::Printf(
						TEXT("*** EMERGENCY STOP — %s ***   power is cut to every drive; ")
						TEXT("trains run to the next brake and hold.   %s"),
						UTF8_TO_TCHAR(Drives->EmergencyStopReason()),
						Drives->AnyUnacknowledged()
							? TEXT("[Home] acknowledge, then [End] to reset")
							: TEXT("[End] to reset")));
			}
			else
			{
				GEngine->AddOnScreenDebugMessage(13, 0.f, FColor(120, 120, 120),
					TEXT("[Backspace] emergency stop"));
			}
		}

		// Discoverable, because a keybinding nobody knows about does not exist.
		//
		// Each mode names its OWN controls rather than sharing one line. A hint
		// listing keys that do nothing in the mode you are actually in is worse
		// than no hint, because it is the same amount of reading and then wrong.
		FString CamLine;
		switch (CameraMode)
		{
		case ETUCameraMode::Rider:
			CamLine = TEXT("[C] camera: rider");
			break;
		case ETUCameraMode::Chase:
			CamLine = TEXT("[C] camera: chase");
			break;
		case ETUCameraMode::Free:
			CamLine = TEXT("[C] camera: free     WASD / Q E / mouse, Shift to hurry");
			break;
		default:
			CamLine = FString::Printf(
				TEXT("[C] camera: orbit    mouse to turn, wheel to zoom, [F] frames all  (%.0f m)"),
				Orbit.Distance);
			break;
		}
		GEngine->AddOnScreenDebugMessage(7, 0.f, FColor(120, 170, 200), CamLine);

		// The ride's own worst case, alongside the current reading, so a number
		// on screen means something without having to remember the whole lap.
		// Roll rate is here rather than in the G line because it belongs to a
		// different question: G is what presses on you, roll rate is what spins
		// you, and no amount of looking at the first will show you the second.
		GEngine->AddOnScreenDebugMessage(4, 0.f, FColor(150, 150, 150),
			FString::Printf(
				TEXT("this ride: %.0f km/h max, %+.2f..%+.2f vertical, %.2f lateral, ")
				TEXT("%.0f deg/s roll"),
				Profile_.TopSpeed * 3.6, Profile_.MinVerticalG, Profile_.MaxVerticalG,
				Profile_.MaxAbsLateralG, Profile_.MaxAbsRollRate));
		if (!Profile_.bCompleted)
		{
			GEngine->AddOnScreenDebugMessage(5, 0.f, FColor::Red,
				FString::Printf(TEXT("TRAIN DOES NOT GET ROUND — stalls at %.0f m"),
					Profile_.StalledAtS));
		}
	}

	// ONLY THE MOUSE IS RESET HERE NOW. A bound axis ACCUMULATES over the frame
	// and has to be cleared after it is read; a polled key is read fresh at the
	// top of the frame, and clearing it here as well would be harmless today and
	// wrong the first time anything reads movement after this line.
	LookYaw = LookPitch = 0.f;

	// Send it round again once it has settled at the END of the track. The return
	// to the station is a teleport, and the range diff handles it with no special
	// case: the last block exits and arms its overlap, the station block enters.
	//
	// ONLY FOR LAYOUTS THAT DO NOT CLOSE. On a measured circuit FTrain wraps, so
	// a train drives into the station under its own power and none of this runs —
	// IsAtEnd is false for ever and the last block is just another block. This is
	// the fallback for the three presets whose two ends are hundreds of metres
	// apart, where a wrap would be inventing continuity that is not there.
	//
	// TWO GUARDS, both of which only matter with more than one train: it must be
	// stopped in the LAST block (a train held at a block brake is also stopped,
	// and teleporting that one would take it out of the queue it is waiting in),
	// and the station must be empty (or two trains land on each other, which the
	// interlocking would then correctly and uselessly report as a violation).
	if (Signals && !bTrackIsCircuit)
	{
		const std::size_t Last = Signals->NumBlocks() - 1;
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			// "Arrived" is stopped OR at the end of the track, and the second half
			// is not redundant: the last block is now a block brake that HOLDS its
			// target, so a train released from it is still doing its release speed
			// when it runs out of track. Testing speed alone would leave it
			// pressed against the end at 2 m/s for ever and the ride would hang.
			const bool bSettled = Signals->BlockAt(Trains[t]->GetDistance()) == Last
				&& (Trains[t]->GetSpeed() <= 0.0 || Trains[t]->IsAtEnd());
			if (!bSettled)
			{
				StoppedForS[t] = 0.f;
				continue;
			}
			StoppedForS[t] += DeltaSeconds;
			if (StoppedForS[t] >= RestartDelaySeconds && !Signals->Occupies(0))
			{
				Trains[t]->Place(0.0, 0.0);
				Signals->Update(static_cast<std::size_t>(t), Trains[t]->GetRearS(),
					Trains[t]->GetFrontS());
				StoppedForS[t] = 0.f;
			}
		}
	}
}
