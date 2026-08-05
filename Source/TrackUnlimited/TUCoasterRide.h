// Phase 0 vertical slice: a hand-authored track, a train that follows it with
// real physics, and a ride camera at the heartline.
//
// This is a thin shell. All the maths lives in the engine-free prototype
// headers under Prototypes/ — this class converts units and handedness at the
// boundary, and does nothing else of consequence. Keep it that way: anything
// that belongs to the simulation belongs in the prototypes, where it can be
// tested in a second without launching an editor.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "TrainPhysics/TrainPhysics.h"
#include "TrainPhysics/RideProfile.h"
#include "TrackSpline/TrackProfile.h"
#include "BlockSignal/RideSignals.h"
#include "BlockSignal/TrackDrives.h"
#include "BlockSignal/TrackSensors.h"
#include "TUTrackSegment.h"

#include "TUCoasterRide.generated.h"

class UCameraComponent;
class UInstancedStaticMeshComponent;

UCLASS()
class TRACKUNLIMITED_API ATUCoasterRide : public APawn
{
	GENERATED_BODY()

public:
	ATUCoasterRide();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Runs on place, load, move and property change — so the preview is live without pressing play. */
	virtual void OnConstruction(const FTransform& Transform) override;

	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

#if WITH_EDITOR
	/** Rebuild and re-check the moment a number changes, so editing has feedback. */
	virtual void PostEditChangeProperty(FPropertyChangedEvent& Event) override;
#endif

	/**
	 * The track, as an ordered list of typed segment parameters. This IS the
	 * editing surface — the viewport is a read-only preview and always will be.
	 *
	 * Metres and degrees. Seeded in the constructor with the reference layout,
	 * so there is something to ride and something to take apart.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Track")
	TArray<FTUTrackSegment> Segments;

	/** Which starter layout the tick-box below loads. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Track")
	ETUPresetLayout Preset = ETUPresetLayout::Reference;

	/** Load the preset above, DISCARDING every edit in the segment list. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Track")
	bool bLoadPreset = false;

	/** The starter layouts, as authored segment lists. */
	static TArray<FTUTrackSegment> PresetLayout(ETUPresetLayout Which);

	/** The reference ride: station, eased 25 degree lift, drop, teardrop loop, banked turn, brakes. */
	static TArray<FTUTrackSegment> ReferenceLayout();

	/** Launch, coast, brake. Three straights — the smallest complete ride there is. */
	static TArray<FTUTrackSegment> FlatRigLayout();

	/** Lift, drop, asymmetric airtime hill, banked turnaround, brakes. No inversion. */
	static TArray<FTUTrackSegment> OutAndBackLayout();

	/** Launched, eight blocks. The only preset that can hold more than one train. */
	static TArray<FTUTrackSegment> TwoTrainCircuitLayout();

	/** Where to watch from. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Camera")
	ETUCameraMode CameraMode = ETUCameraMode::Rider;

	/** Chase only: how far behind the train to sit, in metres. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Camera",
		meta = (ClampMin = "2.0", UIMax = "60.0",
		EditCondition = "CameraMode == ETUCameraMode::Chase", EditConditionHides))
	float ChaseDistanceM = 18.f;

	/** Free camera: metres per second, before the boost modifier. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Camera",
		meta = (ClampMin = "1.0", UIMax = "200.0",
		EditCondition = "CameraMode == ETUCameraMode::Free", EditConditionHides))
	float FreeCameraSpeedMs = 30.f;

	/** Chase only: how far above it, in metres. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Camera",
		meta = (ClampMin = "0.0", UIMax = "40.0",
		EditCondition = "CameraMode == ETUCameraMode::Chase", EditConditionHides))
	float ChaseHeightM = 6.f;

	/** Draw the heartline and rail centreline, since there is no track mesh yet. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	bool bDrawTrack = true;

	/** Speed, G-forces and block state on screen. The point of the slice. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	bool bShowTelemetry = true;

	// ---- Ride profile: the whole ride measured at EDIT time, not ride time.
	//
	// Each channel draws as a curve offset from the track, so the track itself
	// is the zero line and a spike is visible exactly where it happens. Roll
	// rate is here because no G trace can ever show it — felt G models the rider
	// as a point, so spinning one costs nothing.
	//
	// ponytail: booleans and one scale, not a UI. The panel with per-channel
	// visibility, units and readouts is a UI/UX design question and wants
	// designing rather than accreting — this is the data plumbing under it.

	/** Speed along the track. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Ride profile")
	bool bGraphSpeed = false;

	/** Vertical G. The airtime-and-compression axis. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Ride profile")
	bool bGraphVerticalG = true;

	/** Lateral G. Near zero through a correctly banked turn. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Ride profile")
	bool bGraphLateralG = true;

	/** Roll rate. The one channel felt G structurally cannot contain. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Ride profile")
	bool bGraphRollRate = false;

	/** Metres of offset per unit. Each channel has its own sensible unit under this. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Ride profile",
		meta = (ClampMin = "0.1", UIMax = "10.0"))
	float GraphScale = 2.f;

	/** Restart the ride this many seconds after the train comes to rest. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	float RestartDelaySeconds = 3.f;

	// ---- Signalling. Blocks are derived, not authored: a boundary falls wherever
	// a powered run starts or ends, because that is the only place there is a
	// device capable of holding a train. On the reference layout that gives three
	// blocks — lift, course, brake.

	/**
	 * Seconds a block withholds CLEAR after the train's tail leaves it — the
	 * real-railway "overlap".
	 *
	 * Zero clears the instant a tail crosses the boundary, which is a toy:
	 * nothing then separates a following train from the one ahead. Turn it up and
	 * the station visibly holds the train longer between laps.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Signalling",
		meta = (ClampMin = "0.0", UIMax = "30.0"))
	float BlockBufferSeconds = 5.f;

	/**
	 * EXTRA headway, in whole blocks, on top of the braking requirement.
	 *
	 * It used to BE the braking requirement — "clear N blocks ahead" as a stand-in
	 * for a stopping distance nothing computed. That is now derived instead: the
	 * signalling is told which blocks hold a device that can stop a train, and a
	 * permissive clears all the way to the next one, because a train let into a
	 * block with nothing in it is committed until it reaches somewhere it can stop.
	 *
	 * So 1 is the right default now, and larger values buy separation rather than
	 * safety. MEASURED on the two-train circuit: at 1 it carries four trains clean;
	 * at 2 the same four spend their time held, because the extra block demands the
	 * stopping margin twice on a ring this tight.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Signalling",
		meta = (ClampMin = "1", UIMax = "6"))
	int32 DispatchLookahead = 1;

	/**
	 * Where a held train parks: its NOSE this far short of the far end of the
	 * block it is held in.
	 *
	 * About a metre is typical, and the margin IS the number's reason for
	 * existing — it is what stops a train ever protruding into the next zone, a
	 * lift or a launch or open course, through a defect or a mistake. So it is
	 * measured from the END of the block and applied to the NOSE, rather than
	 * being an offset from the start applied to the centre.
	 *
	 * The brake does not put the train here. It stops it well short, wherever the
	 * pad lands it, and the drive tyres then truck it the rest of the way — which
	 * is why a holding device needs both.
	 *
	 * Clamped so a device barely longer than the train still parks it wholly
	 * inside rather than solving to a position behind its own entrance.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Signalling",
		meta = (ClampMin = "0.0", UIMax = "5.0"))
	float HoldNoseClearanceM = 1.f;

	/**
	 * Metres, nose to tail. Zero is a point mass at the heartline.
	 *
	 * A train's speed is governed by the height of its centre of mass, so a
	 * long one softens sharp features — it does not pay the full height of a
	 * crest it is straddling. Measured on this layout, point mass against 15 m:
	 * peak vertical +4.44 -> +4.20 g, and the loop apex FIRMS +1.11 -> +1.13,
	 * because straddling the top of a loop means more speed at the top.
	 *
	 * Note how small that second number is. Length softens the sharp peak by a
	 * quarter of a G and moves the apex by two hundredths — the direction is the
	 * interesting part, not the magnitude, and an earlier version of this comment
	 * quoted a much larger apex effect from before the rolling-resistance
	 * correction and the re-tune that followed it.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited", meta = (ClampMin = "0.0", UIMax = "30.0"))
	float TrainLengthM = 15.f;

	/**
	 * How many trains to run.
	 *
	 * CLAMPED BY THE LAYOUT, not by this number. A train needs somewhere to stand
	 * that can both stop it and start it again — drive tyres and block brakes,
	 * never trim brakes — and ONE of those has to stay free, or every train is
	 * parked exactly where the train behind it needs to go and the ride gridlocks
	 * without a single violation to show for it. So the ceiling is
	 * `hold-capable zones - 1`, measured on the closed circuit: it has five, four
	 * trains run clean, and five never move at all.
	 *
	 * Ask for more and the extras are refused with a log line, because the
	 * alternative is a train materialising on open course with nothing able to
	 * hold it.
	 *
	 * Only the two-train preset can exceed one. The other three have a station and
	 * a trim brake, which is one place to stand and therefore one train.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited", meta = (ClampMin = "1", UIMax = "4"))
	int32 TrainCount = 1;

	/**
	 * Let a train that runs out of energy roll BACK down the hill instead of
	 * stopping dead where it ran out.
	 *
	 * Off by default. Rolling back is the more honest physics, but a valley
	 * stall is a design error to surface, and a train that rolls back
	 * oscillates and settles — which can look like the ride working. The ride
	 * profile reports it either way, and says which of the two happened.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	bool bAllowRollback = false;

	/**
	 * Which row the rider sits in: -1 is the back, 0 the middle, +1 the front.
	 *
	 * Now that the train has length this is a real choice rather than a camera
	 * offset. Every car shares one speed, so what differs between them is the
	 * curvature each is sitting on at any instant — and on an asymmetric crest
	 * that is worth most of a G. The back row is not folklore.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited",
		meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float RiderPosition = 0.f;

private:
	void RebuildFromSegments();
	void DrawTrack() const;

	// What colour the running rails are at this arc length: the authored device,
	// not the geometry. Station dark blue, lift and launch green, brakes red,
	// plain track white.
	FColor RailColourAt(double S) const;
	void DrawRideProfile() const;

	/** The whole ride, sampled by arc length. Recomputed on every rebuild. */
	FRideProfile Profile_;

	/** Prototype metres/right-handed -> Unreal centimetres/left-handed. */
	FVector ToWorld(const FVec3& V) const;
	FQuat ToWorldRotation(const FTrackFrame& Frame) const;

	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited")
	TObjectPtr<USceneComponent> Root;

	// One instance per sample point along the train, so the thing on the track
	// looks like a train rather than a cube. Instanced because the count is
	// whatever the length implies and they all share one mesh.
	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited")
	TObjectPtr<UInstancedStaticMeshComponent> Cars;

	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited")
	TObjectPtr<UCameraComponent> Camera;

	// Not UPROPERTYs: plain C++ with no reflection needed, and deliberately so.
	FTrack Track;

	// Index 0 is the RIDER'S train — the one the camera rides and the readout
	// describes. Every other train is simulated identically and drawn, but nothing
	// on screen reports it.
	TArray<TUniquePtr<FTrain>> Trains;

	// Rebuilt wholesale alongside the trains, for the same reason: block
	// boundaries come off the segment list, so an edit to the track is an edit to
	// the blocks. Null only if the track failed to build at all.
	TUniquePtr<FRideSignals> Signals;

	// The AUTHORED target of each zone, by zone index, kept because a holding
	// device's live target is overwritten to zero whenever its signal is red — so
	// this is the only remaining record of what it should release at.
	TArray<double> ZoneReleaseSpeed;

	// Where each zone is and WHAT KIND it was authored as. FTrackZone deliberately
	// drops the kind — a station, a block brake and a lift chain are the same
	// physics, which is the point — so this keeps it for the things that need to
	// tell them apart. Today that is the debug view colouring the rails; the
	// control panel will want the same list.
	struct FTUZoneSpan
	{
		double StartS = 0.0;
		double EndS = 0.0;
		ETUSegmentZone Kind = ETUSegmentZone::None;
	};
	TArray<FTUZoneSpan> ZoneSpans;

	// The STOP MARK of each zone: a physical switch bolted to the track that tells
	// the PLC a trucking train has come far enough. One per zone, so the index is
	// the zone's own; a zone with no drive tyres can never be commanded to creep,
	// so its mark is simply never read.
	//
	// A SENSOR RATHER THAN A SUM, and that is the entire point of it. Where you
	// place a switch is something an installer knows at survey time, with a tape
	// measure and a train in front of them. Where a train's nose is right now is
	// not something a control system knows at all. HoldNoseClearanceM and
	// TrainLengthM are therefore consumed HERE, once, at build time — and the
	// dispatcher never reads a length or a position again.
	//
	// Null until the track builds, exactly like Signals.
	TUniquePtr<FTrackSensors> StopMarks;

	// ONE DRIVE PER ZONE — the motor at each lift, launch and brake run, and the
	// only thing the dispatcher is allowed to write to. It takes a speed command,
	// ramps its output toward it, and reports back what the motor is really doing
	// and how much torque that is taking; those readings can DISAGREE with the
	// command, which is the entire reason a control panel exists.
	//
	// It also settles something that used to be wrong by construction: zones live
	// on each FTrain, so before this every train carried its own private idea of
	// what every brake on the ride was doing. A drive is ONE device, and its output
	// is written to every train's copy each frame, so they cannot disagree.
	TUniquePtr<FTrackDrives> Drives;

	// Which drive faults have already been logged. A fault is a standing condition
	// until an operator resets it, so without this it would print every frame for
	// the rest of the session and bury everything else in the log.
	TSet<int32> ReportedDriveFault;

	// Which of those zones can HOLD a train — both push and hold, so drive tyres
	// and block brakes, never a trim brake or a launch. Also the list of places a
	// train may be parked, which is what caps how many trains the layout runs.
	TArray<int32> HoldZoneIndices;

	// Seconds each train has been at rest in the last block, for the teleport back
	// to the station. Per train, because they arrive at different times. Unused on
	// a circuit, where trains drive into the station instead.
	TArray<float> StoppedForS;

	// MEASURED on every rebuild, never authored: does the end of the track meet
	// the start in position, heading AND roll? Trains lap when it does and are
	// teleported back to the station when it does not, and neither behaviour is a
	// preference — wrapping an open layout would invent continuity across whatever
	// gap is there.
	bool bTrackIsCircuit = false;

	// Generic track cross-section: gauge, rail and spine dimensions, tie
	// spacing. Defaults sit mid-range for real steel coaster track. Model a
	// track style against these rather than against numbers baked into a mesh.
	FTrackProfile Profile;

	// Landmarks along the track, in metres, filled in by RebuildFromSegments.
	double BrakeStartS = 0.0;

	// How far the ride's lowest structural point sits BELOW the heartline
	// origin, in metres. Applied by ToWorld so that an actor at z = 0 puts the
	// track on the ground instead of half through it. Computed on rebuild,
	// because it depends on the whole layout.
	double GroundOffsetM = 0.0;

	// Commands every holding device under a train: open to its authored release
	// speed while the permissive grants, closed to zero otherwise. Brakes-on is
	// the resting state, so a device that nobody serves stays shut.
	void ServeHolds(std::size_t TrainIndex);

	// Chase camera state. Smoothed in world space rather than recomputed from
	// the track, so it lags the way a following camera should and needs no
	// EvaluateAt — which is O(track length) and has no business in a tick.
	FVector ChaseLocation = FVector::ZeroVector;
	// Held over when the track goes vertical and there is no horizontal
	// direction of travel to derive one from.
	FVector LastChaseForward = FVector(1.f, 0.f, 0.f);
	bool bChaseInitialised = false;

	// Free camera. Seeded from wherever the previous mode had the camera, so
	// switching to it does not teleport you somewhere unrecognisable.
	FVector FreeLocation = FVector::ZeroVector;
	FRotator FreeRotation = FRotator::ZeroRotator;
	bool bFreeInitialised = false;

	// Axis values written by the input bindings and consumed by Tick.
	float MoveForward = 0.f;
	float MoveRight = 0.f;
	float MoveUp = 0.f;
	float LookYaw = 0.f;
	float LookPitch = 0.f;
	bool bBoost = false;

	void CycleCameraMode();
	void BoostOn() { bBoost = true; }
	void BoostOff() { bBoost = false; }

	// One per key rather than one per axis: BindAxisKey reports 1.0 while a key
	// is held, so the sign has to come from which key was bound.
	void AxisForward(float V) { MoveForward += V; }
	void AxisBack(float V) { MoveForward -= V; }
	void AxisRight(float V) { MoveRight += V; }
	void AxisLeft(float V) { MoveRight -= V; }
	void AxisUp(float V) { MoveUp += V; }
	void AxisDown(float V) { MoveUp -= V; }
	void AxisLookYaw(float V) { LookYaw += V; }
	void AxisLookPitch(float V) { LookPitch += V; }
};
