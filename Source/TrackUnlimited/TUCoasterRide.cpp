#include "TUCoasterRide.h"

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

	// Seeded so a freshly placed actor has a ride in it. Everything about that
	// ride is data in the Details panel rather than code, which is the whole
	// point — Preset + bLoadPreset puts a known-good one back if an edit goes
	// wrong, or swaps in a different worked example to take apart.
	Segments = PresetLayout(Preset);
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
	for (const FTrackDiagnostic& D : ValidateTrack(BuildSegments(Doc)))
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

		auto Close = [this, &Zones, &Open, &OpenS, &OpenSpeed, &StopMarkS](double EndS)
		{
			if (Open == ETUSegmentZone::None || !(EndS > OpenS))
			{
				return;
			}
			// ponytail: grip fixed at 6 m/s^2 of tractive authority for every
			// zone. It is the one number a real VFD panel would expose per
			// drive; give it a field when something needs a weak chain or a
			// hard launch, not before.
			const double Grip = 6.0;

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
				Zones.Add(MakeLift(OpenS, EndS, Speed, Grip));
				break;
			case ETUSegmentZone::Launch:
				Zones.Add(MakeLaunch(OpenS, EndS, Speed, Grip));
				break;
			case ETUSegmentZone::Brake:
				Zones.Add(MakeBrake(OpenS, EndS, Speed, Grip));
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
				Zones.Add(MakeLift(OpenS, EndS, Speed, Grip));
				break;
			default:
				return;
			}
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
			const bool bSpeedChanged = Open != ETUSegmentZone::None
				&& !FMath::IsNearlyEqual(Segments[i].ZoneSpeed, OpenSpeed);
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
			}
			AccS += SegLength;
		}
		Close(AccS);
		if (bCatchOpen)
		{
			CatchSpans.Add(TPair<double, double>(CatchStartS, AccS));
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

	// Rebuild and redraw happen in OnConstruction, so a typed number gets an
	// answer immediately — the drawn track, plus total length, continuity, where
	// it ends up and whether it hits itself. The viewport stays a read-only
	// preview: this is feedback, not manipulation.
	Super::PostEditChangeProperty(Event);
}
#endif

FVector ATUCoasterRide::ToWorld(const FVec3& V) const
{
	// Mirror Y: the prototype frame is right-handed, Unreal is left-handed.
	// Lift by GroundOffsetM: the heartline origin is RIDER height, not track
	// height, so z = 0 in track space is about 1.7 m above the bottom of the
	// spine. See RebuildFromSegments for why this is computed rather than typed.
	return GetActorLocation() + FVector(V.X, -V.Y, V.Z + GroundOffsetM) * MetresToUU;
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
	PlayerInputComponent->BindKey(EKeys::BackSpace, IE_Pressed, this,
		&ATUCoasterRide::PressEmergencyStop);
	PlayerInputComponent->BindKey(EKeys::End, IE_Pressed, this,
		&ATUCoasterRide::ResetEmergencyStop);
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

	PlayerInputComponent->BindAxisKey(EKeys::W, this, &ATUCoasterRide::AxisForward);
	PlayerInputComponent->BindAxisKey(EKeys::S, this, &ATUCoasterRide::AxisBack);
	PlayerInputComponent->BindAxisKey(EKeys::D, this, &ATUCoasterRide::AxisRight);
	PlayerInputComponent->BindAxisKey(EKeys::A, this, &ATUCoasterRide::AxisLeft);
	PlayerInputComponent->BindAxisKey(EKeys::E, this, &ATUCoasterRide::AxisUp);
	PlayerInputComponent->BindAxisKey(EKeys::Q, this, &ATUCoasterRide::AxisDown);
	PlayerInputComponent->BindAxisKey(EKeys::MouseX, this, &ATUCoasterRide::AxisLookYaw);
	PlayerInputComponent->BindAxisKey(EKeys::MouseY, this, &ATUCoasterRide::AxisLookPitch);

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
	default: CameraMode = ETUCameraMode::Rider; break;
	}
	// Re-seed on the way in, so the free camera starts from wherever you were
	// just looking rather than teleporting you somewhere unrecognisable.
	bFreeInitialised = false;
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

void ATUCoasterRide::DrawControlPanel(UCanvas* Canvas, APlayerController* /*PC*/)
{
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
		+ (bMaint ? 1 : 0)                         // DETECTION summary
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
		FString StopText(TEXT("RUNNING"));
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
			bStopped ? PanelRed : PanelGreen);
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

void ATUCoasterRide::LogEvent(const FString& Text, bool bBad)
{
	// Newest first, capped. Oldest falls off the end rather than the ring wrapping
	// in place, because the panel wants "the last few" and a wrapped array has to
	// be unwrapped to give it.
	EventLog.Insert(FTURideEvent{RideClock, Text, bBad}, 0);
	const int32 Keep = 8;
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

void ATUCoasterRide::DrawTrack() const
{
	// No track mesh yet — that is Phase 4. This draws the actual cross-section
	// as wireframe instead: two running rails at gauge, the spine below them,
	// and cross-ties. Enough to model a track style against, and enough to see
	// that the heartline and the rails really are different curves.
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

void ATUCoasterRide::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (Trains.Num() == 0 || !Trains[0].IsValid())
	{
		return;
	}
	FTrain* const Train = Trains[0].Get();   // the rider's train: camera, readout

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

	// The Details-panel checkbox, so the stop can be tripped without playing. Read
	// here rather than in a PostEditChangeProperty because it has to work in PIE.
	if (bEmergencyStop && Drives && !Drives->IsEmergencyStopped())
	{
		Drives->PressEmergencyStopButton("operator");
	}
	ServeStations(DeltaSeconds);

	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		ServeHolds(static_cast<std::size_t>(t));
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

	// Where the rider is sitting, which is a real choice now that cars differ.
	const double SeatOffset = RiderPosition * TrainLengthM * 0.5;
	const FTrackFrame& Frame = Train->GetFrameAt(SeatOffset);
	const FQuat Rotation = ToWorldRotation(Frame);

	// One car per sample point. The cars sit on the RAILS; the rider sits at
	// the heartline. That distinction is the entire reason the heartline model
	// exists, so the slice shows it rather than putting both in one place.
	//
	// Frames come from the train rather than from Track.EvaluateAt, which is
	// O(track length) a call and would be six of those every frame.
	{
		const int32 CarCount = Train->NumSamplePoints();
		const int32 Total = CarCount * Trains.Num();
		const double CarLength =
			TrainLengthM > 0.f ? TrainLengthM / CarCount : 2.4;
		if (Cars->GetInstanceCount() != Total)
		{
			Cars->ClearInstances();
			for (int32 i = 0; i < Total; ++i)
			{
				Cars->AddInstance(FTransform::Identity, true);
			}
		}
		const FVector CarScale(CarLength * 0.9, 1.4, 1.0);
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			for (int32 i = 0; i < CarCount; ++i)
			{
				const int32 Slot = t * CarCount + i;
				const FTrackFrame& CarFrame = Trains[t]->GetSamplePoint(i);
				const FVec3 OnRails = CarFrame.Position - CarFrame.Up * Track.GetHeartlineHeight();
				Cars->UpdateInstanceTransform(Slot,
					FTransform(ToWorldRotation(CarFrame), ToWorld(OnRails), CarScale), true,
					Slot == Total - 1, true);
			}
		}
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

		FreeRotation.Yaw += LookYaw * 2.2f;
		FreeRotation.Pitch = FMath::Clamp(FreeRotation.Pitch + LookPitch * 2.2f, -87.f, 87.f);
		FreeRotation.Roll = 0.f; // a free camera that rolls is a lost camera

		const FVector Forward = FreeRotation.Vector();
		const FVector Right = FRotationMatrix(FreeRotation).GetScaledAxis(EAxis::Y);
		const float Speed = FreeCameraSpeedMs * MetresToUU * (bBoost ? 5.f : 1.f) * DeltaSeconds;
		FreeLocation += (Forward * MoveForward + Right * MoveRight) * Speed
			+ FVector(0.f, 0.f, MoveUp * Speed);

		Camera->SetWorldLocationAndRotation(FreeLocation, FreeRotation.Quaternion());
	}
	else if (CameraMode == ETUCameraMode::Rider)
	{
		Camera->SetWorldLocationAndRotation(ToWorld(Frame.Position), Rotation);
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

	if (bShowTelemetry && GEngine)
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
		const TCHAR* ModeName =
			CameraMode == ETUCameraMode::Rider ? TEXT("rider")
			: (CameraMode == ETUCameraMode::Chase ? TEXT("chase") : TEXT("free"));
		GEngine->AddOnScreenDebugMessage(7, 0.f, FColor(120, 170, 200),
			CameraMode == ETUCameraMode::Free
				? FString::Printf(TEXT("[C] camera: free    WASD / Q E / mouse, Shift to hurry"))
				: FString::Printf(TEXT("[C] camera: %s"), ModeName));

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

	MoveForward = MoveRight = MoveUp = LookYaw = LookPitch = 0.f;

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
