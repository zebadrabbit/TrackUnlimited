#include "TUCoasterRide.h"

#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	// The prototypes work in metres; Unreal works in centimetres.
	constexpr double MetresToUU = 100.0;

	constexpr double Pi = 3.14159265358979323846;
	double Deg(double D) { return D * Pi / 180.0; }

	// A vertical curve with no curvature step at either end: pitch curvature
	// ramps 0 -> peak -> 0, so both joints stay continuous. This is the whole
	// point of the representation, and it is why the layout below reports
	// IsCurvatureContinuous() == true.
	void EasedPitch(FTrack& T, double PitchDelta, double PeakCurvature)
	{
		const double K = PitchDelta >= 0.0 ? PeakCurvature : -PeakCurvature;
		const double L = FMath::Abs(PitchDelta) / PeakCurvature;

		FTrackSegment In;
		In.Length = L;
		In.PitchCurvatureEnd = K;
		T.AddSegment(In);

		FTrackSegment Out;
		Out.Length = L;
		Out.PitchCurvatureStart = K;
		T.AddSegment(Out);
	}
}

ATUCoasterRide::ATUCoasterRide()
{
	PrimaryActorTick.bCanEverTick = true;
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	Cart = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Cart"));
	Cart->SetupAttachment(Root);
	Cart->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Cart->SetStaticMesh(CubeMesh.Object);
		// Roughly one car: 2.4 m long, 1.4 m wide, 1 m tall. The train is still
		// a POINT in the simulation — this is a stand-in, not a length model.
		Cart->SetRelativeScale3D(FVector(2.4f, 1.4f, 1.0f));
	}

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Root);
}

void ATUCoasterRide::BuildTrack()
{
	// Authored numerically, exactly as the project intends track to be authored
	// — an ordered list of typed segment parameters, with no viewport dragging
	// anywhere in sight. Tuned in the standalone harness first; these numbers
	// give a 43.4 m lift, 101 km/h, +0.70..+4.25 vertical G, 0.36 peak lateral,
	// and +1.25 G over the loop apex.
	const double Lift = Deg(25.0);
	const double Drop = Deg(-34.0);
	const double LoopRadius = 9.0;
	const double LoopEase = 54.0;
	const double TurnRadius = 32.0;
	const double Bank = FMath::Atan((26.5 * 26.5) / (GravityMs2 * TurnRadius));

	Track = FTrack();

	// Solved, not eyeballed. At 55.0 m this layout ended 8.498 m BELOW its own
	// station and ran the whole back half underground — the rail centreline
	// bottomed out at -9.60 m, which is what "dropping through the floor after
	// the loop" actually was. TrackClose.h's HeightTarget with the lift climb
	// freed closes that to +0.0007 m at 75.11 m.
	//
	// It costs nothing in ride feel, and that is not luck: the chain hauls the
	// train to the crest at 4 m/s whatever the crest's height, and the drop
	// geometry below it never moved. Speed, both G peaks, lateral and the loop
	// apex all come out bit-identical to the 55 m version. Only the height moved.
	const double LiftClimb = 75.11;

	Track.AddSegment(MakeStraight(20.0));            // station
	EasedPitch(Track, Lift, 0.03);                   // into the climb
	// One constant, used twice on purpose: the chain zone's extent is derived
	// from where the climb ends, so a second literal here could drift out of
	// step with the segment and strand the train short of the crest.
	const double LiftTopS = Track.TotalLength() + LiftClimb;
	Track.AddSegment(MakeStraight(LiftClimb));       // lift climb
	EasedPitch(Track, Drop - Lift, 0.05);            // crest
	Track.AddSegment(MakeStraight(12.0));            // drop
	EasedPitch(Track, -Drop, 0.012);                 // pull-out

	// Teardrop loop: curvature eases in and out rather than stepping, so the
	// radius is large where the train is fastest. A circular loop at this speed
	// would pull over 7 G at the bottom, which is why real loops are not circles.
	{
		FTrackSegment EaseIn;
		EaseIn.Length = LoopEase;
		EaseIn.PitchCurvatureEnd = 1.0 / LoopRadius;
		Track.AddSegment(EaseIn);

		FTrackSegment Crown;
		Crown.Length = 2.0 * Pi * LoopRadius - LoopEase;
		Crown.PitchCurvatureStart = Crown.PitchCurvatureEnd = 1.0 / LoopRadius;
		Track.AddSegment(Crown);

		FTrackSegment EaseOut;
		EaseOut.Length = LoopEase;
		EaseOut.PitchCurvatureStart = 1.0 / LoopRadius;
		Track.AddSegment(EaseOut);
	}

	// Banked turn, clothoid in and out so neither curvature nor roll steps.
	Track.AddSegment(MakeClothoid(26.0, 0.0, 1.0 / TurnRadius, 0.0, Bank));
	Track.AddSegment(MakeArc(55.0, TurnRadius, Bank));
	Track.AddSegment(MakeClothoid(26.0, 1.0 / TurnRadius, 0.0, Bank, 0.0));

	BrakeStartS = Track.TotalLength();
	Track.AddSegment(MakeStraight(70.0));            // brake run

	Train = MakeUnique<FTrain>(Track);
	// The chain has to run over the crest: the climb tops out before the track
	// stops rising, and releasing at the top of the straight strands the train.
	Train->AddZone(MakeLift(0.0, LiftTopS + 24.0, 4.0, 6.0));
	Train->AddZone(MakeBrake(BrakeStartS, Track.TotalLength(), 0.0, 6.0));
	Train->Place(0.0, 0.0);

	UE_LOG(LogTemp, Log,
		TEXT("TrackUnlimited: %d segments, %.1f m, curvature-continuous=%s"),
		static_cast<int32>(Track.NumSegments()), Track.TotalLength(),
		Track.IsCurvatureContinuous() ? TEXT("yes") : TEXT("NO"));
}

FVector ATUCoasterRide::ToWorld(const FVec3& V) const
{
	// Mirror Y: the prototype frame is right-handed, Unreal is left-handed.
	return GetActorLocation() + FVector(V.X, -V.Y, V.Z) * MetresToUU;
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

void ATUCoasterRide::BeginPlay()
{
	Super::BeginPlay();
	BuildTrack();

	if (bDrawTrack)
	{
		DrawTrack();
	}
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

void ATUCoasterRide::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!Train.IsValid())
	{
		return;
	}

	Train->Step(DeltaSeconds);

	const FTrackFrame& Frame = Train->GetFrame();
	const FQuat Rotation = ToWorldRotation(Frame);

	// The cart sits on the rails; the rider sits at the heartline. That
	// distinction is the entire reason the heartline model exists, so the slice
	// should show it rather than putting both in the same place.
	Cart->SetWorldLocationAndRotation(ToWorld(Track.RailCentreAt(Train->GetDistance())), Rotation);
	Camera->SetWorldLocationAndRotation(ToWorld(Frame.Position), Rotation);

	if (!bRideCamera)
	{
		Camera->SetWorldLocation(ToWorld(Frame.Position) - Rotation.GetForwardVector() * 1400.f
			+ FVector(0.f, 0.f, 450.f));
	}

	if (bShowTelemetry && GEngine)
	{
		const FGForces G = Train->GetForces();
		const double S = Train->GetDistance();
		GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::White,
			FString::Printf(TEXT("%6.1f km/h    %5.1f m along %.0f m    height %5.1f m"),
				Train->GetSpeed() * 3.6, S, Track.TotalLength(), Frame.Position.Z));
		GEngine->AddOnScreenDebugMessage(2, 0.f,
			G.Vertical > 4.5 || G.Vertical < -1.0 ? FColor::Red : FColor::Green,
			FString::Printf(TEXT("vertical %+5.2f G    lateral %+5.2f G    fore-aft %+5.2f G"),
				G.Vertical, G.Lateral, Train->GetTangentialG()));
		GEngine->AddOnScreenDebugMessage(3, 0.f, FColor::Silver,
			S >= BrakeStartS ? TEXT("BRAKE RUN") : TEXT("on course"));
	}

	// Send it round again once it has settled in the brakes.
	if (Train->GetSpeed() <= 0.0)
	{
		StoppedFor += DeltaSeconds;
		if (StoppedFor >= RestartDelaySeconds)
		{
			Train->Place(0.0, 0.0);
			StoppedFor = 0.f;
		}
	}
	else
	{
		StoppedFor = 0.f;
	}
}
