#include "TUCoasterRide.h"

#include "TrackSpline/TrackClose.h"
#include "TrackSpline/TrackValidate.h"

#include "Camera/CameraComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
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
	default:                               return ReferenceLayout();
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
	AddStraight(Out, 20.0, ETUSegmentZone::Lift, 4.f);         // station
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
	AddStraight(Out, 26.0, ETUSegmentZone::Lift, 1.5f);       // 1 STATION, drive tyres
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

	Straight(20.0, ETUSegmentZone::Lift, 4.f);         // station
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
		return;
	}

	// Built once and handed to every train, rather than walked per train: the
	// zones ARE the track, and a second train that derived its own could disagree
	// with the first about where the brakes are.
	TArray<FTrackZone> Zones;
	ZoneReleaseSpeed.Reset();

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

		auto Close = [this, &Zones, &Open, &OpenS, &OpenSpeed](double EndS)
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
				// Brakes AND drive tyres, so identical in shape to a lift chain.
				// The separate enumerator exists for the block boundary and for
				// the Details panel, not for the physics — what makes it a block
				// brake is having BOTH authorities, which is exactly what
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
			if (Segments[i].Zone != Open)
			{
				Close(AccS);
				AddBoundary(AccS);
				Open = Segments[i].Zone;
				OpenS = AccS;
				OpenSpeed = Segments[i].ZoneSpeed;
			}
			else if (Open != ETUSegmentZone::None
				&& !FMath::IsNearlyEqual(Segments[i].ZoneSpeed, OpenSpeed))
			{
				// A run is defined by its KIND, so this segment joins the one
				// already open and its own ZoneSpeed is DISCARDED. Measured: a
				// brake run at 6 m/s followed immediately by one at 2 m/s becomes
				// a single zone targeting 6, and the 2 never existed.
				//
				// That is a typed number the build throws away, which is exactly
				// what this project validates against elsewhere. Reported, not
				// repaired: guessing which speed was meant would be worse, and the
				// fix an author actually wants is usually a short powered section
				// between the two — that both splits the run and gives a stopped
				// train something to move it, since a friction brake can hold a
				// train but cannot start one.
				UE_LOG(LogTemp, Warning,
					TEXT("TrackUnlimited: segment %d continues the run that began at %.1f m, "
						"so its zone speed %.1f m/s is IGNORED — the run keeps %.1f m/s. "
						"Separate them with a different zone kind if two devices were meant."),
					i, OpenS, Segments[i].ZoneSpeed, OpenSpeed);
			}
			AccS += SegLength;
		}
		Close(AccS);
		if (bCatchOpen)
		{
			CatchSpans.Add(TPair<double, double>(CatchStartS, AccS));
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

		Signals = MakeUnique<FRideSignals>(BlockStarts, BlockBufferSeconds,
			static_cast<std::size_t>(FMath::Max(1, DispatchLookahead)),
			static_cast<std::size_t>(Running), bTrackIsCircuit);

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
	Trains[0]->Place(0.0, 0.0);

	// NOW shut them, once the profile has been taken. Brakes-on is the resting
	// state of real ride control, and the alternative — open until a dispatcher
	// notices — is open for exactly one frame every time, which is one frame of a
	// train being pushed through a red.
	//
	// And NOW close the circuit, for the same reason and in the same breath: the
	// profile walks arc length forwards and stops at the end, so on a lapping
	// train it would never stop and its samples would overwrite each other. The
	// ride profile is one lap, measured open; the ride itself laps.
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

		DrawDebugLine(GetWorld(), ToWorld(Walk.Position), ToWorld(NextFrame.Position),
			FColor(90, 190, 255), true, -1.f, 0, 2.f);                 // heartline
		DrawDebugLine(GetWorld(), ToWorld(Section.LeftRail), ToWorld(NextSection.LeftRail),
			FColor(235, 235, 235), true, -1.f, 0, 3.f);                // running rails
		DrawDebugLine(GetWorld(), ToWorld(Section.RightRail), ToWorld(NextSection.RightRail),
			FColor(235, 235, 235), true, -1.f, 0, 3.f);
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
	const FChannel Channels[] = {
		{bGraphVerticalG, FColor(120, 235, 130), 1.0, &FRideSample::VerticalG},
		{bGraphLateralG, FColor(250, 175, 80), 1.0, &FRideSample::LateralG},
		{bGraphSpeed, FColor(110, 205, 255), 1.0 / (10.0 / 3.6), &FRideSample::Speed},
		{bGraphRollRate, FColor(230, 130, 235), 1.0 / 30.0, &FRideSample::RollRateDegPerSec},
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
	if (!Signals || TrainIndex >= static_cast<std::size_t>(Trains.Num()))
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
	if (Signals->CanRelease(TrainIndex, T.GetDistance()))
	{
		T.SetZoneTargetSpeed(Zi, ZoneReleaseSpeed[Z]);
		return;
	}

	// Held — and it brakes to a POSITION, not to zero wherever it happens to be.
	// sqrt(2*a*d) is the fastest a train may travel and still stop in d, so
	// commanding that as the target eases it down and parks it mid-device.
	//
	// Why that matters, measured: commanded to plain zero a train stops within
	// ~0.3 m of where the ZONE starts, because a zone says "reach this speed" and
	// zero is reachable immediately. The station's start IS the seam of the
	// circuit, so that leaves the back half of the train in the LAST block — a
	// dwelling train holds two blocks, and three trains then DEADLOCK, each denied
	// by the tail of the one in front, silently and with no violation.
	const FTrackZone Zone = T.GetZone(Zi);
	const double StopS = 0.5 * (Zone.StartS + Zone.EndS);
	const double Remaining = StopS - T.GetDistance();
	// The device's OWN authority, not a repeat of the grip constant — if a zone is
	// ever given a weaker brake, the curve has to weaken with it or the dispatcher
	// would be promising a stop the hardware cannot make.
	const double Curve = Remaining > 0.0 && Zone.MaxDecel > 0.0
		? FMath::Sqrt(2.0 * Zone.MaxDecel * Remaining) : 0.0;
	// ponytail: mid-device, which is right for a station and arbitrary for a long
	// block brake. Give a holding zone an authored stop offset when somebody wants
	// a platform marked somewhere other than the middle.
	T.SetZoneTargetSpeed(Zi, FMath::Min(ZoneReleaseSpeed[Z], Curve));
}

void ATUCoasterRide::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (Trains.Num() == 0 || !Trains[0].IsValid())
	{
		return;
	}
	FTrain* const Train = Trains[0].Get();   // the rider's train: camera, readout

	// Every train, then ONE tick. Overlaps live on blocks rather than on trains,
	// so ticking per train would age a 5 s overlap in 5/N seconds and nothing
	// anywhere would say so.
	//
	// Holding is done by GATING THE ZONE, not by declining to integrate. The old
	// version held the train at the station by simply not stepping it, which
	// worked for exactly one train in exactly one place; a zone commanded to zero
	// holds a train anywhere there is a device to hold it, and the station stops
	// being a special case.
	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		ServeHolds(static_cast<std::size_t>(t));
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
		}
	}
	if (Signals)
	{
		Signals->Tick(DeltaSeconds);
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
