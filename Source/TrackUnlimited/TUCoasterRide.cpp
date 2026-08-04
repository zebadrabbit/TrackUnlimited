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
	// because that is the only place a device exists that can hold a train. Every
	// other preset has two powered runs and therefore three blocks, and capacity
	// is blocks/(1 + lookahead) — so three blocks is one train, always. This has
	// eight, which is four trains at lookahead 1 and two at lookahead 2.
	//
	// The approach is TWO brake blocks with drive tyres between, not one long
	// brake, and both halves of that matter. Two adjacent Brake runs MERGE into a
	// single block, so "one long brake" holds exactly one train — and worse, the
	// second run's authored ZoneSpeed is silently discarded (measured: 6 m/s then
	// 2 m/s becomes one zone at 6). And a friction brake can hold a train but
	// cannot start one, also measured, so moving a stopped train from the outer
	// brake to the inner one needs powered track either way.
	//
	// MEASURED, and every figure re-derived independently before it was written
	// here: 25 segments, 1072.5 m, 8 blocks, ends -0.0000 m, C2 to 1e-9, closest
	// self-approach 7.75 m, top 136.8 km/h, vertical -0.35..+3.57 g, lateral
	// 0.54 g, no block shorter than the 15 m train, zero curvature-step
	// diagnostics. The train comes to rest at 1044.4 m — inside the inner
	// pre-station brake, which is where a train waits for the station and is the
	// correct outcome rather than a stall.
	const double Up = Deg(28.0);
	const double Dn = Deg(-30.0);

	// Solved, not eyeballed: the climb that brings the circuit back to station
	// height. It is the right lever because more climb ends higher monotonically,
	// where the drop is not — even a minimum-length drop overshoots downward,
	// since the eases and the return carry most of the descent.
	const double Climb = 41.7685;

	// A brake block must hold a whole train with room to stop, not just to sit.
	const double BrakeLen = 37.5;      // 2.5 train lengths
	const double TransferLen = 27.0;   // 1.8 train lengths

	TArray<FTUTrackSegment> Out;

	AddStraight(Out, 26.0, ETUSegmentZone::Lift, 1.5f);       // 1 STATION, drive tyres
	AddStraight(Out, 150.0, ETUSegmentZone::Launch, 38.f);    // 2 LAUNCH

	// Launch length matters more than the target: at the fixed 6 m/s^2 grip a
	// launch caps at sqrt(2*grip*length) whatever is asked for, so 70 m could
	// never exceed 29 m/s and three different targets all produced the same top
	// speed. 150 m is what makes 38 m/s actually reachable.

	// 3 COURSE: up, over, down. NO INVERSION deliberately — a planar loop's two
	// legs pass 0.19 m apart, which is the reference layout's known defect, and
	// this preset exists to demonstrate blocks rather than to show off.
	AddEasedPitch(Out, Up, 0.0195);       // the +Gz peak lives HERE, not in the
	AddStraight(Out, Climb);              // valley: it is the highest-v^2 curvature
	AddEasedPitch(Out, Dn - Up, 0.0300);  // on the track. 0.030 -> 0.0195 took the
	AddStraight(Out, 34.0);               // peak from +5.00 g to +3.57.
	AddEasedPitch(Out, -Dn, 0.012);

	// Turn 1, banked for the 31.5 m/s the train ACTUALLY carries rather than a
	// guessed 24. In a banked turn the felt magnitude is sqrt(1 + (v^2/gR)^2)
	// however well it is banked — bank only splits that between vertical and
	// lateral — so RADIUS is the only lever that lowers both, and 30 -> 48 m is
	// what took lateral from 1.34 g to 0.54. Widening it also opened the return
	// leg, which is where the clearance went from 3.33 m to 7.75.
	AddBankedTurn(Out, 48.0, Pi * 48.0, 34.0, BankDegreesFor(31.5, 48.0));

	AddStraight(Out, 45.0, ETUSegmentZone::Brake, 20.f);      // 4 MID-COURSE BLOCK BRAKE

	AddEasedPitch(Out, Deg(12.0), 0.010);                     // 5 COURSE back
	AddEasedPitch(Out, Deg(-12.0), 0.010);
	AddBankedTurn(Out, 30.0, Pi * 30.0, 24.0, BankDegreesFor(10.6, 30.0));
	AddStraight(Out, 24.0);

	// 6/7/8 THE APPROACH. The outer brake is where a second train waits while the
	// first is in the station; the inner one holds short of the station and is
	// what clears the station-entry signal once its train has stopped.
	//
	// ponytail: the two pre-station brakes trim to 6 and 2 m/s rather than to a
	// dead stop, because a zone target cannot be changed at runtime yet — so a
	// brake commanded to zero would hold its train there forever with nothing
	// able to release it. Once FTrain can retarget a zone, both become 0 and the
	// permissive releases them; the geometry does not change.
	AddStraight(Out, BrakeLen, ETUSegmentZone::Brake, 6.f);       // outer
	AddStraight(Out, TransferLen, ETUSegmentZone::Lift, 4.f);     // transfer tyres
	AddStraight(Out, BrakeLen, ETUSegmentZone::Brake, 2.f);       // inner
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
		return;
	}

	FTrainConfig TrainConfig;
	TrainConfig.TrainLength = TrainLengthM;
	TrainConfig.bAllowRollback = bAllowRollback;
	Train = MakeUnique<FTrain>(Track, TrainConfig);

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

		auto Close = [this, &Open, &OpenS, &OpenSpeed](double EndS)
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
			switch (Open)
			{
			case ETUSegmentZone::Lift:
				Train->AddZone(MakeLift(OpenS, EndS, OpenSpeed, Grip));
				break;
			case ETUSegmentZone::Launch:
				Train->AddZone(MakeLaunch(OpenS, EndS, OpenSpeed, Grip));
				break;
			case ETUSegmentZone::Brake:
				Train->AddZone(MakeBrake(OpenS, EndS, OpenSpeed, Grip));
				break;
			default:
				break;
			}
		};

		for (int32 i = 0; i < Segments.Num(); ++i)
		{
			const double SegLength =
				BuildSegment(Doc.Segments[static_cast<std::size_t>(i)]).Length;
			if (!(SegLength > 0.0))
			{
				continue; // AddSegment refused it, so it occupies no arc length
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

		// Reported before it is used, not repaired silently — FRideSignals will
		// repair it either way, but a walk that produced something malformed is a
		// bug upstream and should say so rather than be absorbed.
		if (!FRideSignals::IsWellFormed(BlockStarts))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("TrackUnlimited: derived block boundaries are malformed; repairing."));
		}
		Signals = MakeUnique<FRideSignals>(BlockStarts, BlockBufferSeconds,
			static_cast<std::size_t>(FMath::Max(1, DispatchLookahead)));

		{
			FString Where;
			for (const double S : Signals->Boundaries())
			{
				Where += FString::Printf(TEXT("%.1f  "), S);
			}
			UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: %d blocks, boundaries at %sm; "
				"lookahead %d, overlap %.1f s"),
				static_cast<int32>(Signals->NumBlocks()), *Where,
				static_cast<int32>(Signals->Lookahead()), BlockBufferSeconds);
		}

		BrakeStartS = AccS;
		for (int32 i = Segments.Num() - 1; i >= 0; --i)
		{
			if (Segments[i].Zone != ETUSegmentZone::Brake)
			{
				break;
			}
			BrakeStartS -= BuildSegment(Doc.Segments[static_cast<std::size_t>(i)]).Length;
		}
	}
	Train->Place(0.0, 0.0);

	// Seed occupancy from where the train actually is, before anything asks a
	// permissive. Without this the station block reads CLEAR with a train sitting
	// in it, and the first dispatch is granted against a lie.
	if (Signals)
	{
		Signals->Update(Train->GetRearS(), Train->GetFrontS());
	}

	// Only hold if there is something to grant the release. Without this, a
	// layout that somehow produced no signalling would sit in the station
	// forever waiting on a permissive nothing can ever answer — the ride would
	// simply never start, and nothing on screen would say why.
	bAwaitingDispatch = Signals.IsValid();

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
	Profile_ = RunRideProfile(*Train, Track, 1.0);
	Train->Place(0.0, 0.0);

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

void ATUCoasterRide::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!Train.IsValid())
	{
		return;
	}

	// Overlaps age whether or not the train is moving — that is the entire point
	// of a time-based overlap, and a held train is exactly when it matters.
	if (Signals)
	{
		Signals->Tick(DeltaSeconds);
	}

	if (bAwaitingDispatch && Signals)
	{
		// Held. The train is deliberately NOT stepped, because the station is a
		// Lift zone at 4 m/s and would otherwise pull it straight out from under
		// the interlock. Holding it by not integrating is honest for a station;
		// a launch would want the zone itself gated instead.
		// NumBlocks is never zero — the constructor repairs an empty boundary
		// list into a single block — so the modulo is safe without a guard.
		const std::size_t Here = Signals->BlockAt(Train->GetDistance());
		const std::size_t Next = (Here + 1) % Signals->NumBlocks();
		if (Signals->CanDispatchInto(Next))
		{
			bAwaitingDispatch = false;
		}
	}
	else
	{
		Train->Step(DeltaSeconds);

		// The return is the only record a signalling violation leaves besides the
		// counter, so it is read rather than discarded. On a single-train circuit
		// this should never fire; if it does, the permissive let something
		// through and that is worth seeing immediately.
		if (Signals && !Signals->Update(Train->GetRearS(), Train->GetFrontS()))
		{
			UE_LOG(LogTemp, Error,
				TEXT("TrackUnlimited: SIGNALLING VIOLATION at %.1f m — entered a block "
					"that was not clear."),
				Train->GetDistance());
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
		const double CarLength =
			TrainLengthM > 0.f ? TrainLengthM / CarCount : 2.4;
		if (Cars->GetInstanceCount() != CarCount)
		{
			Cars->ClearInstances();
			for (int32 i = 0; i < CarCount; ++i)
			{
				Cars->AddInstance(FTransform::Identity, true);
			}
		}
		const FVector CarScale(CarLength * 0.9, 1.4, 1.0);
		for (int32 i = 0; i < CarCount; ++i)
		{
			const FTrackFrame& CarFrame = Train->GetSamplePoint(i);
			const FVec3 OnRails = CarFrame.Position - CarFrame.Up * Track.GetHeartlineHeight();
			Cars->UpdateInstanceTransform(i,
				FTransform(ToWorldRotation(CarFrame), ToWorld(OnRails), CarScale), true,
				i == CarCount - 1, true);
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
			GEngine->AddOnScreenDebugMessage(8, 0.f,
				bAwaitingDispatch ? FColor(255, 176, 32) : FColor(120, 200, 140), Row);

			if (bAwaitingDispatch)
			{
				GEngine->AddOnScreenDebugMessage(9, 0.f, FColor(255, 176, 32),
					TEXT("HELD — dispatch permissive not satisfied"));
			}
			if (Signals->Violations() > 0)
			{
				GEngine->AddOnScreenDebugMessage(10, 0.f, FColor::Red,
					FString::Printf(TEXT("%d SIGNALLING VIOLATION(S)"),
						static_cast<int32>(Signals->Violations())));
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

	// Send it round again once it has settled in the brakes. The return to the
	// station is a teleport, and the range diff handles it with no special case:
	// the brake block exits and arms its overlap, the station block enters. That
	// overlap is then usually what holds the next dispatch.
	if (!bAwaitingDispatch && Train->GetSpeed() <= 0.0)
	{
		StoppedFor += DeltaSeconds;
		if (StoppedFor >= RestartDelaySeconds)
		{
			Train->Place(0.0, 0.0);
			if (Signals)
			{
				Signals->Update(Train->GetRearS(), Train->GetFrontS());
			}
			bAwaitingDispatch = true;
			StoppedFor = 0.f;
		}
	}
	else
	{
		StoppedFor = 0.f;
	}
}
