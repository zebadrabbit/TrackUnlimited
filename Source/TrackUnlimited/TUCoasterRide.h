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
// FTrackDiagnostic is a MEMBER type below, not just something the .cpp uses.
// The .cpp included this and the header did not, so the member declared with it
// was ill-formed and MSVC recovered into a bare `std::vector` -- which then
// misreported as "no default constructor" in generated code and as a broken
// range-for hundreds of lines away from the actual cause.
#include "TrackSpline/TrackValidate.h"
#include "BlockSignal/RideSignals.h"
#include "BlockSignal/DeviceAudit.h"
#include "BlockSignal/SignalWatch.h"
#include "BlockSignal/Evacuation.h"
#include "BlockSignal/PlcUnit.h"
#include "TrackMesh/TrackMesh.h"
#include "Shell/CameraRig.h"
#include "Shell/GraphAxis.h"
#include "Shell/DiagnosticsModel.h"
#include "Shell/FirstRun.h"
#include "Shell/SessionState.h"
#include "Shell/TrackBrowser.h"
#include "TrackMesh/TrackSupports.h"
#include "BlockSignal/ShowBus.h"
#include "BlockSignal/SimDigest.h"
#include "BlockSignal/StationProcess.h"
#include "BlockSignal/TrackDrives.h"
#include "BlockSignal/TrackSensors.h"
#include "TUTrackSegment.h"

#include "TUCoasterRide.generated.h"

// The ride's event stream, so it can be filtered on its own.
DECLARE_LOG_CATEGORY_EXTERN(LogTUEvents, Log, All);

class UCameraComponent;
class UInstancedStaticMeshComponent;

UCLASS()
class TRACKUNLIMITED_API ATUCoasterRide : public APawn
{
	GENERATED_BODY()

public:
	ATUCoasterRide();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
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

	/**
	 * Evacuation catwalks, PLACED BY A PERSON. Start, stop, side.
	 *
	 * Deliberately not derived and deliberately not suggested. Where a walkway
	 * *should* go by the logic of "a train can stop here" may be somewhere
	 * architecturally impossible to reach, structurally unsupportable, or
	 * dangerous to stand — none of which this model knows anything about. The
	 * layout can say where one would HELP; only a person says where one goes.
	 *
	 * Empty by default, including on every preset, and that is honest rather than
	 * unfinished: a shipped layout with catwalks nobody surveyed would be the
	 * project asserting something it cannot know.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Track")
	TArray<FTUWalkway> Walkways;

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

	/** The same oval, re-zoned for small vehicles: unload plus three load positions. */
	static TArray<FTUTrackSegment> SmallBatchLayout();

	/** The train that goes with a preset — a small-batch ride has small vehicles. */
	void ApplyPresetTrainSetup(ETUPresetLayout Which);

	/** Where to watch from. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Camera")
	ETUCameraMode CameraMode = ETUCameraMode::Rider;

	/**
	 * Flip mouse Y.
	 *
	 * ORBIT AND FREE-FLY ALREADY DISAGREE, and that is a real convention rather
	 * than an inconsistency to iron out: in orbit you are dragging the SUBJECT,
	 * so pulling the mouse down swings the camera up over the top of it; in
	 * free-fly you are turning your own HEAD, so pulling down looks down. Every
	 * DCC tool and every flight sim respectively does it that way round.
	 *
	 * So the shipped default is not "no inversion anywhere" — it is each camera's
	 * own convention, and this knob flips BOTH from there. One setting for the
	 * whole application, because somebody who wants Y inverted wants it inverted.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Camera")
	bool bInvertLookY = false;

	/**
	 * Rider eye height above the heartline, metres.
	 *
	 * THE HEARTLINE IS THE HEART. It is where felt G is computed and what the
	 * banking is built around — a rider's chest, roughly — and a camera sitting
	 * exactly on it is a view from inside the car body.
	 *
	 * A KNOB RATHER THAN A CONSTANT, because seat geometry is a property of the
	 * vehicle somebody designed and 0.25 m is a seated adult's eye-above-heart.
	 * It is COSMETIC: it moves the view and nothing else, and must stay that way.
	 * Adding it to the heartline instead would change every G figure on the ride
	 * to fix a framing problem.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Camera",
		meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float RiderEyeAboveHeartlineM = 0.25f;

	/**
	 * THE CURSOR IS VISIBLE WHERE THERE IS SOMETHING TO CLICK, and looking moves
	 * to a HELD RIGHT BUTTON.
	 *
	 * Every panel that takes a click — the main menu, the segment list, the
	 * diagnostics rows — lives in Menu, Build and Operate, and a captured mouse
	 * made all three keyboard-only. Ride has nothing to click and a cursor over
	 * an on-ride camera is only in the way, so it keeps the captured mouse and
	 * free look.
	 *
	 * The two halves are one change: showing the cursor without moving look onto
	 * a drag gives a camera that spins every time somebody reaches for a row.
	 */
	bool bDraggingLook = false;
	bool bCursorShown = false;    // what was last applied, so this is not set every frame

	void BeginLookDrag() { bDraggingLook = true; }
	void EndLookDrag() { bDraggingLook = false; }
	void ApplyCursorMode();

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

	/** The heartline and the device colours, as wireframe. Not made redundant by
	 *  the Phase 4 mesh: the heartline is not a physical part of the track, and
	 *  which block a stretch belongs to is invisible on a solid rail. */
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

	/** Profile samples that could not be drawn because a channel was NaN or
	 *  infinite. Counted rather than clamped, and reported after the walk —
	 *  a trace drawn over broken data is worse than a trace with a hole in it. */
	mutable int32 NonFiniteTraceSamples = 0;

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
	 * How long the platform takes, in seconds: riders off, riders on, then
	 * restraints closed and the operators' walk-round.
	 *
	 * THESE ARE NOT WHAT GATES THE DISPATCH. Every step of a station sequence is a
	 * physical contact — a restraint lock sensor, an airgate switch, an operator's
	 * all-clear — and the process waits on those, not on a clock. There are no
	 * riders in this simulation, so something has to assert the contacts a person
	 * would, and these are that stand-in. They go away when riders arrive.
	 *
	 * Treat them as throughput targets, which is what they are on a real
	 * operation: the whole business of running a coaster is making them smaller,
	 * and a rider who needs longer to board is a load figure, not a fault.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Station",
		meta = (ClampMin = "0.0", UIMax = "60.0"))
	float UnloadSeconds = 6.f;

	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Station",
		meta = (ClampMin = "0.0", UIMax = "120.0"))
	float LoadSeconds = 12.f;

	/** Restraints closed, then checked. The all-clear comes at the end of it. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Station",
		meta = (ClampMin = "0.0", UIMax = "60.0"))
	float RestraintCheckSeconds = 4.f;

	/**
	 * How long the restraint bars take to travel once commanded, and how many
	 * separately-sensed groups they are in — a car, a row, a platform segment.
	 *
	 * UNLIKE THE DWELL FIGURES ABOVE, THIS IS HARDWARE. A bar closes in the same
	 * time on a quiet Tuesday as on a busy Saturday, and it is what the sequence
	 * actually waits on: the lock contact comes from the bank's own sensors rather
	 * than from a clock, so a bar that will not travel holds the dispatch for ever
	 * instead of being quietly declared shut.
	 *
	 * The group count is the resolution at which a failure can be reported. Real
	 * HMIs unlock "Seats Segment 1" and lock all of them, which is why it is worth
	 * more than one.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Station",
		meta = (ClampMin = "0.0", UIMax = "15.0"))
	float RestraintTravelSeconds = 2.f;

	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Station",
		meta = (ClampMin = "1", UIMax = "12"))
	int32 RestraintGroups = 4;

	/**
	 * Who decides the timing. In manual, a station holds its train until the
	 * dispatch button is pressed — [Space] — and the button must be RELEASED
	 * between trains, so a wedged control dispatches nothing.
	 *
	 * The safety interlocks apply identically in both modes. Manual changes who
	 * decides *when*, never whether the permissives can be bypassed.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Station")
	bool bManualDispatch = false;

	/**
	 * Trip the ride's emergency stop from the Details panel. [Backspace] does the
	 * same in play, and [End] resets it.
	 *
	 * IT DOES NOT STOP TRAINS, IT STOPS THE RIDE. Power is cut to every drive, so
	 * a train in a brake run stops at once and a train on open course coasts to the
	 * next brake and is held there — which is what a real E-stop does, and why a
	 * ride is built out of block brakes in the first place.
	 *
	 * Also tripped automatically by the three conditions this project already
	 * detects and, until now, only logged: a signalling violation, a drive fault,
	 * and a train counter reading a block as occupied twice.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Signalling")
	bool bEmergencyStop = false;

	/**
	 * The generated control panel: an indicator per block, a VFD module per
	 * powered run, a sequence readout per platform. [P] cycles operator ->
	 * maintenance -> off.
	 *
	 * Two views rather than one because a real installation has two, and they are
	 * not the same screen with a detail level: an operator dispatches trains and a
	 * maintainer diagnoses machines, and motor current belongs to exactly one of
	 * those. See ETUPanelView.
	 *
	 * Play only. It draws on the debug canvas, which the editor viewport does not
	 * run — and a control room is a thing you look at while the ride is running,
	 * not while you are drawing track.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Signalling")
	ETUPanelView PanelView = ETUPanelView::Operator;

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

	/**
	 * WHICH TRAIN YOU ARE ON. [T] cycles it.
	 *
	 * `Trains[0]` was hardcoded, with a comment calling it "the rider's train" —
	 * which was true when there was only ever one. The circuit carries four and
	 * the small-batch preset seven, and every one of them has its own lap, its
	 * own dwell and its own reason for waiting at a signal. Watching the ride
	 * from a different train is most of what an operator's view is FOR.
	 *
	 * The ride profile deliberately still measures train 0, because that is a
	 * property of the LAYOUT rather than of a train: every train on a circuit
	 * runs the same geometry, and a profile that changed with the camera would
	 * be a conformance verdict that moved when you looked at it differently.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Camera", meta = (ClampMin = "0"))
	int32 RiderTrain = 0;

	/** The train the camera and the readouts follow, clamped to what exists.
	 *  Clamped rather than trusted because TrainCount is editable and shrinking
	 *  it must not leave the camera pointed at a train that is gone. */
	int32 ActiveTrainIndex() const
	{
		return Trains.Num() > 0 ? FMath::Clamp(RiderTrain, 0, Trains.Num() - 1) : 0;
	}

	void NextRiderTrain();

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
	// The same conversion without the actor's own location, for anything that
	// lives in a COMPONENT rather than being drawn into the world: mesh vertices
	// are actor-local and get the actor transform applied for free.
	FVector ToLocal(const FVec3& V) const;
	// Direction only: mirrored, not scaled and not offset. A normal converted
	// with ToLocal would be a point near the origin rather than a direction.
	FVector ToLocalDirection(const FVec3& V) const;
	FQuat ToWorldRotation(const FTrackFrame& Frame) const;

	/**
	 * PHASE 4: THE TRACK, AS SOMETHING YOU CAN SEE.
	 *
	 * Three components because rails, spine and ties are three materials, and
	 * because a track style may want to replace one of them without touching the
	 * others.
	 *
	 * The geometry comes from `Prototypes/TrackMesh/`, which is engine-free and
	 * assert-tested — this is only the port. That means unit conversion,
	 * mirroring Y, AND REVERSING TRIANGLE WINDING, which is the part neither
	 * CLAUDE.md nor PHASE0_FINDINGS previously said: M(x,y,z) = (x,-y,z) is a
	 * REFLECTION with determinant -1, so it flips triangle orientation. Mirror
	 * the positions and normals alone and every surface on the ride is inside
	 * out — invisible under backface culling, with every vertex position
	 * correct. Asserted as a property in test_trackmesh.cpp.
	 */
	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited|Mesh")
	TObjectPtr<class UProceduralMeshComponent> RailMesh;

	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited|Mesh")
	TObjectPtr<class UProceduralMeshComponent> SpineMesh;

	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited|Mesh")
	TObjectPtr<class UProceduralMeshComponent> TieMesh;

	/**
	 * Off leaves the wireframe as the only view, which is what every screenshot
	 * before Phase 4 was taken with. Measured: a 1288 m circuit at these defaults
	 * is 14 ms to sweep, so a rebuild is one hitch rather than something needing
	 * a button.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Mesh")
	bool bBuildTrackMesh = true;

	/**
	 * THE VISUAL STYLE, so a coaster looks like a structure rather than a spline
	 * in space.
	 *
	 * NO AUTHORED ASSET. The three mesh sections take dynamic material instances
	 * over an ENGINE material referenced by path — the same trick the cars
	 * already use for their cube — so a style is data in this file rather than a
	 * `.uasset` somebody has to make before anything looks like anything.
	 *
	 * That is a real limit as well as a convenience: an engine base material has
	 * a colour and nothing else, so there is no roughness, no metal and no normal
	 * map here. It is a FIRST style, and the card asked for one complete style
	 * rather than a good one.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Mesh")
	ETUTrackStyleName TrackStyle = ETUTrackStyleName::SteelModern;

	/** Override the preset's numbers. Empty name means "use the preset". */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Mesh")
	FTUTrackStyle CustomStyle;

	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Mesh")
	bool bUseCustomStyle = false;

	static FTUTrackStyle StylePreset(ETUTrackStyleName Which);
	FTUTrackStyle ActiveStyle() const;
	void ApplyTrackStyle();

	UPROPERTY(Transient)
	TObjectPtr<class UMaterialInterface> BaseMaterial;

	UPROPERTY(Transient)
	TObjectPtr<class UMaterialInstanceDynamic> RailMaterial;
	UPROPERTY(Transient)
	TObjectPtr<class UMaterialInstanceDynamic> SpineMaterial;
	UPROPERTY(Transient)
	TObjectPtr<class UMaterialInstanceDynamic> TieMaterial;

	/** Metres between rings. THE quality/cost knob, and a distance rather than a
	 *  count so a long track does not come out coarser than a short one. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Mesh",
		meta = (ClampMin = "0.1", ClampMax = "5.0", EditCondition = "bBuildTrackMesh"))
	float MeshSampleSpacingM = 0.5f;

	/** Segments around a tube. Eight is a visible octagon close up; twelve is the
	 *  sensible shipping figure. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Mesh",
		meta = (ClampMin = "3", ClampMax = "24", EditCondition = "bBuildTrackMesh"))
	int32 MeshSides = 12;

	void RebuildTrackMesh();
	void PushMeshSection(class UProceduralMeshComponent* Target, const FMeshBuffer& M) const;

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

	/** The friction pad's rate for each zone, or 0 where there is none. Parallel
	 *  to ZoneReleaseSpeed and built in the same walk, for the same reason: two
	 *  lists derived together cannot slide against each other. */
	TArray<double> ZoneBrakeDecel;

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

	// One PLATFORM POSITION, with everything it needs to run its own sequence.
	// Per position rather than per platform, because a high-throughput ride holds
	// several trains on one platform and dispatches them individually — a rider
	// who needs longer to board must not hold up the two trains in front. Three
	// positions would be three of these; every preset here has one.
	struct FTUPlatform
	{
		int32 Zone = INDEX_NONE;
		FStationProcess Process{EStationRole::Combined};
		FAutoStationCrew Crew;
		FStationInputs Inputs;
	};
	TArray<FTUPlatform> Platforms;

	// Has this train got its riders yet — per TRAIN, because that is what it is a
	// property of. On a multi-position platform every position after the first sees
	// an already-loaded train and must not board it again: riders get on once, at
	// whichever position their train was standing at, and it then advances full.
	//
	// Kept here rather than in the station because a platform cannot know which
	// train is over it — a switch has no idea. This is the sort of per-vehicle
	// state bit a real PLC keeps for exactly the same reason.
	TArray<bool> TrainLoaded;

	/**
	 * WHAT EACH RESTRAINT GROUP ON THIS TRAIN IS DOING, drawn on the train itself.
	 *
	 * A count cannot say WHICH bar is open, and which one is the only thing worth
	 * knowing — going to look at it is the entire purpose of the number. The panel
	 * says `HARNESS 3/4`; this says which of the four, on the car it belongs to.
	 *
	 * Kept PER TRAIN rather than read live off the platform, for the same reason
	 * TrainLoaded is: the bank belongs to the platform, so a departing train would
	 * lose its state the moment it left and its cars would go grey — reading as
	 * though the bars had opened. That is a lying instrument, which this project
	 * has now been bitten by once. Snapshotted while at a platform and held.
	 *
	 * This is also the readout a fault-injection scenario needs. `StuckGroup` is
	 * already the hook; "simulate a stuck harness" is unobservable without
	 * somewhere for the answer to appear.
	 */
	TArray<uint8> TrainGroupState;   // FCommandedBank::EGroupState per group, per train
	TArray<bool> TrainGroupClosed;   // was the bank COMMANDED closed when sampled

	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Signalling")
	bool bShowRestraints = true;

	void SampleRestraints();
	void DrawRestraints() const;

	/** The authored list turned into what the evacuation check reads. Rebuild only. */
	std::vector<FWalkwaySpan> WalkwaySpans;

	/** Where the block boundaries fall, in world space. Only changes on a rebuild. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Signalling")
	bool bShowBlockMarkers = true;

	struct FTUBlockMark
	{
		/** UE world space, centimetres. */
		FVector World = FVector::ZeroVector;

		/**
		 * UNIT vectors in UE space, and the word is load-bearing.
		 *
		 * They are built by differencing two ToWorld calls a metre apart, which
		 * yields a vector of length 100 rather than 1 — ToWorld converts metres
		 * to centimetres. Left un-normalised that is a unit-per-METRE vector
		 * wearing the name of a direction, and every multiplier downstream comes
		 * out 100x: the gate meant to stand 4.2 m over the rails stood 420 m over
		 * them, which is what "markers flying off into the sky" was.
		 */
		FVector Up = FVector::UpVector;
		FVector Lateral = FVector::RightVector;
	};
	TArray<FTUBlockMark> BlockMarks;

	/**
	 * Walked ONCE per rebuild, not per frame.
	 *
	 * `EvaluateAt` is O(track length) a call, so eleven boundaries every frame
	 * would be eleven full integrations to draw eleven posts. The boundaries only
	 * move when the track does, so they are walked with a single AdvanceFrom pass
	 * and cached — the same mistake DrawTrack's comment already records having
	 * made once.
	 */
	void BuildBlockMarks();
	void DrawBlockMarkers() const;

	/**
	 * One scan of the control system and one step of the physics, at a FIXED
	 * period. Tick runs however many of these the elapsed frame is worth.
	 *
	 * A PLC scans on a fixed period; it does not scan faster because the graphics
	 * card is idle. Running this once per rendered frame — which it did until now
	 * — put every rate in the control system at the mercy of the frame rate, and
	 * made two runs of the same session diverge on the first frame.
	 */
	void SimStep(double DeltaSeconds);

	/**
	 * Scans per second. 240 to match the prototype suites exactly, which have
	 * always stepped at 1/240 s — so the thing you play and the thing that is
	 * asserted now run the same way.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Physics",
		meta = (ClampMin = "30", ClampMax = "1000"))
	int32 SimHz = 240;

	/**
	 * The most scans one rendered frame may run before the backlog is DROPPED.
	 *
	 * Not a performance knob — a safety one. Working off a backlog means running
	 * the ride faster than real time, and a ride that fast-forwards through a
	 * hitch can skip a train past a block boundary, which is the single failure
	 * this whole layer exists to prevent. 24 is a tenth of a second at 240 Hz.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Physics",
		meta = (ClampMin = "1", ClampMax = "240"))
	int32 MaxStepsPerFrame = 24;

	double SimAccumulator = 0.0;
	int32 ScanOverruns = 0;

	/** False until the first tick after BeginPlay has been discarded. That frame's
	 *  delta is level load and the actor's own build, not ride time — a controller
	 *  does not owe scans for the time before it was powered on. */
	bool bSimClockStarted = false;

	/**
	 * HOW MUCH RIDE WENT MISSING, which the count alone does not say.
	 *
	 * Dropped time is not slow motion. The scans never ran, so between two
	 * consecutive scans a train really did travel further than the model would
	 * ever have moved it — which is precisely the failure the fixed scan period
	 * exists to prevent, arriving by the back door when the host cannot keep up.
	 *
	 * So this is the number that decides whether a run can be JUDGED at all.
	 * Behaviour somebody watched across a drop is behaviour of a ride nothing
	 * computed, and no amount of staring at it will settle what it did.
	 */
	double ScanTimeDroppedS = 0.0;
	double WorstOverrunS = 0.0;
	// The first tick after a load carries the load itself. Not a missed deadline.
	bool bScanStarted = false;
	// Raised by the accumulator, consumed by the PLC's watchdog on the next scan.
	bool bScanOverranThisFrame = false;

	/**
	 * A running fingerprint of every scan, and the scan number it is up to.
	 *
	 * Two sessions that loaded the same preset and were left alone must show the
	 * same digest at the same scan number. That is a check anybody can run by eye
	 * and it is the property a recorded scenario rests on — the prototype suite
	 * asserts it, this is where you watch it hold on the real thing.
	 *
	 * The scan number is shown WITH it because a digest on its own is not
	 * comparable: it is a running hash, so two rides agree only at the same point.
	 */
	FSimDigest SimFingerprint;
	int64 ScanNumber = 0;

	/**
	 * THE CONTROLLER, as a machine in the cabinet rather than an implication.
	 *
	 * `FRideSignals` plus `ServeHolds` is the PLC *program*; this is the PLC. It
	 * owns the key switch, the watchdog, the loaded program's identity and the
	 * power state — four real operational conditions the model could not express.
	 *
	 * THE STANDARD PLC, NOT THE SAFETY CHAIN. It can withhold permission to run
	 * and has no authority to prevent a stop: the E-stop is inside `FTrackDrives`
	 * and the brakes are fail-safe, so neither routes through here. That is
	 * constraint 7 made structural, and it is asserted rather than promised.
	 */
	FPlcUnit Plc;

	/** Turn the key. Refused with a readable reason; see FPlcUnit::WhyNotRun. */
	void SetPlcMode(EPlcMode Wanted);

	/** The operator's walkdown: the course is empty and I have looked. */
	void DeclareCourseClear();

	// Reads every platform's instruments and runs its crew. Once per frame, at the
	// top of the scan with the other inputs.
	void ServeStations(float DeltaSeconds);

	/** Covers the block-boundary switches with every train's span. Once per scan. */
	void ScanBlockSensors();

	/** Compares the counter against the interlocking; a difference stops the ride. */
	void CrossCheckOccupancy();

	/**
	 * THE CONTROL PANEL, and it is GENERATED rather than authored. It walks the
	 * same ordered block and zone lists the geometry and the physics walk, and for
	 * each element emits its control-room counterpart: a block becomes an
	 * indicator, a powered run becomes a VFD module, a platform becomes a sequence
	 * readout. Add a block to a layout and an indicator appears, because there is
	 * nowhere else for it to come from.
	 *
	 * It is a second generated VIEW over the canonical data, not a second copy of
	 * it — every number here is read live from FRideSignals, FTrackDrives,
	 * FBlockCounter and FStationProcess, and nothing is computed for display.
	 *
	 * 2D first, deliberately: a modelled control booth is a presentation layer over
	 * this, later, not a separate system.
	 */
	void DrawControlPanel(UCanvas* Canvas, APlayerController* PC);

	/** Registration handle for the debug-canvas draw. */
	FDelegateHandle PanelDrawHandle;

	/**
	 * THE EVENT LOG, and it is the difference between a status display and a
	 * record. The panel can say what is holding a train right NOW; without this it
	 * cannot say what tripped the E-stop thirty seconds ago, and "what happened
	 * just before it stopped" is the first question anybody asks.
	 *
	 * A ring, so it cannot grow without bound on a ride left running overnight, and
	 * short because a control-room screen shows the last few and an operator scrolls
	 * for the rest. Appended by the same places that log — a violation, a drive
	 * fault, an E-stop, a detection disagreement — so there is one story rather than
	 * two that can disagree.
	 */
	struct FTURideEvent
	{
		float AtSeconds = 0.f;
		FString Text;
		bool bBad = true;
	};
	TArray<FTURideEvent> EventLog;
	float RideClock = 0.f;

	void LogEvent(const FString& Text, bool bBad = true);

	/**
	 * EVERY STATE TRANSITION, WHICH IS THE OTHER HALF OF THE LOG.
	 *
	 * `LogEvent` above records the six things that go WRONG. That leaves the
	 * ordinary question unanswerable — "did that lamp ever light?" — because
	 * nothing routine is written down and a screenshot is the whole of the
	 * evidence. This walks the same block, platform, drive and console lists the
	 * panel walks and records what MOVED.
	 *
	 * Edge, not level: a log that wrote every point every scan would be thirty
	 * lines a frame. The change detection and its seeding rule live in
	 * FSignalWatch, where they are tested.
	 *
	 * Verbose by design and off by default — it is a diagnostic instrument, and
	 * the panel's eight-line log is the thing you read while standing there.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Signalling")
	bool bLogStateTransitions = false;

	FSignalWatch StateWatch;
	void LogTransitions();

	/**
	 * TIER 3'S INPUT, AND THE ONE THING IT GETS.
	 *
	 * Constraint 7's show layer is READ-ONLY to the ride: a train passes a
	 * sensor, the DMX side is told, and nothing comes back. DMX512 agrees at the
	 * wire level, being unidirectional by design with no return path.
	 *
	 * So this is a SUBSCRIBER, and it is run at the very end of the scan, after
	 * the outputs are written and after the fingerprint is taken. That ordering
	 * is the claim: everything hashed above is the ride, and this cannot reach
	 * back into any of it. Asserted in test_twotrains.cpp — the same circuit with
	 * and without the show layer is identical to the bit.
	 *
	 * The publisher owns its OWN FSignalWatch rather than sharing StateWatch's
	 * channel range, on that header's advice: a collision between two consumers
	 * of one range is silent and reads as a transition that never happened.
	 *
	 * Triggers are empty until something authors them, so this costs one edge
	 * compare per channel and fires nothing.
	 */
	FShowPublisher ShowPublisher;
	FShowBus ShowBus;
	void PublishShowEvents();

	// Operator -> maintenance -> off, on one key. Three states on the key that
	// already showed the panel beats a second binding nobody remembers.
	void CyclePanelView()
	{
		// ASKING FOR ONE OVERLAY MEANS YOU WANT OVERLAYS. Without this the key
		// appears dead while [F2] is holding everything off, and the state it
		// silently changed underneath is waiting to surprise somebody later.
		bHideOverlays = false;
		switch (PanelView)
		{
		case ETUPanelView::Operator:    PanelView = ETUPanelView::Maintenance; break;
		case ETUPanelView::Maintenance: PanelView = ETUPanelView::Off; break;
		default:                        PanelView = ETUPanelView::Operator; break;
		}
	}

	// Will the station at this zone let its train go? True where there is no
	// station, which is every device that is not a platform.
	bool StationSaysGo(std::size_t Zone) const;

	/**
	 * PRE-LAUNCH: is the device about to take this train ready to take it?
	 *
	 * The step between "everything is secured" and "you may go", and it belongs to
	 * the DEVICE rather than the platform — a launch armed and charged, a chain
	 * actually turning. The interlocking asks whether the next blocks are CLEAR;
	 * this asks whether the one taking the train is READY, which is a different
	 * question and the one a real console puts its own lamp on.
	 *
	 * Plain track is trivially ready: a term that denied a dispatch onto open
	 * course would stop the ride rather than protect it.
	 */
	bool DeviceAheadIsReady(double AtS) const;

	// The operator's controls. The dispatch button is bound on both edges because
	// the RELEASE is half the safety rule.
	void PressDispatch() { bDispatchHeld = true; }
	void ReleaseDispatch() { bDispatchHeld = false; }
	void PressEmergencyStop();
	// The reset button as a contact rather than an event. Monitored 0-1-0: the
	// stop clears on the RELEASE, so a taped button cannot restart the ride.
	void PressResetButton();
	void ReleaseResetButton();
	void ResetEmergencyStop();

	/** "I have seen this." Silences the alarm; fixes nothing. Reset needs it first. */
	void AcknowledgeFaults();

	bool bDispatchHeld = false;

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

	// A SECOND, INDEPENDENT MEANS OF KNOWING WHERE THE TRAINS ARE. One sensor per
	// block boundary, and a counter deriving occupancy from their trips alone — no
	// position, no train identity, nothing the interlocking is reading.
	//
	// The point is the DISAGREEMENT. FRideSignals is handed each train's exact span
	// every frame; this has nothing but rising and falling edges. Two ways of
	// knowing the same fact, arrived at from different information, and if they
	// ever differ then one of them is wrong and neither can say which. That is what
	// a real installation buys with a second detection method, and it is what turns
	// the sensor layer from a proven-equivalent curiosity into something the ride's
	// safety actually rests on.
	//
	// CIRCUITS ONLY. FBlockCounter is a counter over a RING — block N-1 is bounded
	// by sensor N-1 and sensor 0 — and on a point-to-point layout that wrap is a
	// lie: sensor 0 going low is a train being placed at the start, not one leaving
	// the end. Null on an open layout, deliberately.
	TUniquePtr<FTrackSensors> BlockSensors;
	TUniquePtr<FBlockCounter> Counter;

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

	/**
	 * THE ORBIT CAMERA, whose arithmetic lives in Prototypes/Shell/CameraRig.h
	 * and is tested there — this is only the wiring.
	 *
	 * Kept in the prototypes' own right-handed metres, and converted at the same
	 * boundary everything else is, so the framing maths never has to know which
	 * way round Unreal's Y is.
	 */
	FOrbitState Orbit;
	bool bOrbitFramed = false;

	/**
	 * THE RIDE PROFILE AS A GRAPH — the view that makes a spike diagnosable.
	 *
	 * The in-world traces show WHERE something happens; this shows WHAT, against
	 * a labelled axis you can read a number off. Different questions, which is
	 * why both exist rather than one replacing the other.
	 *
	 * Drawn on the debug canvas exactly as the control panel is, and for the same
	 * reason: it needs no asset, so it works the moment somebody presses the key.
	 * Every number in it comes from the tested `GraphAxis.h`.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Ride profile")
	bool bShowProfileGraph = false;

	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Ride profile",
		meta = (EditCondition = "bShowProfileGraph"))
	ETUProfileChannel ProfileChannel = ETUProfileChannel::VerticalG;

	void DrawProfileGraph(class UCanvas* Canvas);

	/**
	 * THE DIAGNOSTICS PANEL. TrackValidate has produced exactly the right data
	 * since Phase 0 and none of it has ever been visible outside a log.
	 *
	 * The rules it draws by live in Prototypes/Shell/DiagnosticsModel.h and are
	 * tested there. REPORT, NEVER REPAIR applies here too: a row has a place to
	 * GO and no action to TAKE, and there is nowhere to put a fix button.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Diagnostics")
	bool bShowDiagnostics = true;

	/** Support placement refuses more than it places on a layout with a helix,
	 *  and those refusals are the point — but they are noise while somebody is
	 *  still shaping the track, so they are opt-in. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Diagnostics")
	bool bShowSupportFindings = false;

	FDiagnostics Diagnostics;
	std::vector<FTrackDiagnostic> LastDiagnostics;

	/**
	 * What the DEVICES will do, as opposed to whether the geometry is valid.
	 *
	 * Kept the same way LastDiagnostics is, and for the same reason: it is
	 * produced during the rebuild where the derived zones and the ride profile
	 * both exist, and consumed later by the panel. Computing it in
	 * BuildDiagnostics instead would mean deriving the zones a second time, and a
	 * second derivation is a second thing that can disagree with the first.
	 */
	std::vector<FDeviceFinding> LastDeviceFindings;

	/**
	 * WHICH SEGMENT IS SELECTED, and the thing that turns a finding from a
	 * sentence into somewhere to go.
	 *
	 * Editable so the Details panel can drive it today, and set by clicking a
	 * diagnostics row at runtime. -1 is nothing selected, which is a real state
	 * rather than "segment 0" — framing segment 0 because nobody chose anything
	 * would move the camera for no reason anybody asked for.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Diagnostics",
		meta = (ClampMin = "-1"))
	int32 SelectedSegment = -1;

	/** Where a diagnostics row was drawn, so a click can find it again. Rebuilt
	 *  every frame the panel draws, which is what makes this immediate-mode
	 *  rather than a retained widget tree: there is nothing to keep in sync. */
	TArray<FVector4> DiagRowRects;

	/** [Z] — frame just the selected segment, which is what a validation warning
	 *  wants you looking at. Falls back to the whole track when nothing is
	 *  selected, because a key that did nothing would read as broken. */
	void FrameSelectedSegment();

	/** Click. Immediate-mode: the rows drawn last frame are hit-tested against
	 *  the mouse, and the whole hit-test is a rectangle compare. */
	void ClickDiagnostics();

	/** THE CAMERA REMEMBERS EACH MODE. Switching Build to Ride to Build must not
	 *  lose your place — cheap, obvious, and never added later because it is
	 *  never the most urgent bug. Tested in Prototypes/Shell/CameraRig.h. */
	FCameraRigs CameraRigs;
	ETUCameraMode LastCameraMode = ETUCameraMode::Rider;

	/**
	 * OPERATE MODE, AND WHAT IT MEANS FOR THE REST OF THE APPLICATION.
	 *
	 * The generated control panel has existed since Phase 3 and has been a debug
	 * overlay the whole time. Hosting it means BUILD and OPERATE become different
	 * places rather than the same place with a key held down — and the difference
	 * that actually matters is not which panel is up, it is that
	 * `FSession::EditsAllowed()` is false in one of them.
	 *
	 * That is constraint 1 one level up: a ride that is running is not a ride
	 * being edited, and an edit landing mid-lap would change the geometry under a
	 * train. Structural rather than remembered — an edit arriving in Operate is
	 * REFUSED, so a panel that forgot cannot corrupt a running ride.
	 *
	 * The session's rules are tested in Prototypes/Shell/SessionState.h.
	 */
	FSession Session;

	/**
	 * THE SIM CLOCK, which is an OPERATOR'S tool rather than a debug convenience.
	 *
	 * Watching a buffer count down at quarter speed is how somebody learns what
	 * the interlocking is doing, and stepping one scan at a time is how they see
	 * a permissive drop the frame a restraint opens. Neither is visible at 1x.
	 *
	 * IT SCALES THE CLOCK, NOT THE STEP. The scan period stays 1/SimHz, and time
	 * scale changes how much wall-clock time is fed to the accumulator — so a
	 * quarter-speed run is the SAME SEQUENCE OF SCANS, just delivered more
	 * slowly, and FSimDigest still matches a real-time run of the same session.
	 * Scaling the step instead would change every rate inside the controller and
	 * make slow motion a different ride.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Simulation",
		meta = (ClampMin = "0.05", ClampMax = "4.0"))
	float TimeScale = 1.f;

	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Simulation")
	bool bSimPaused = false;

	/** Scans still owed to a [.] press. Stepping is counted rather than timed, so
	 *  one press is exactly one scan whatever the frame rate. */
	int32 StepsOwed = 0;

	void TogglePause() { bSimPaused = !bSimPaused; }
	void StepOneScan() { StepsOwed += 1; }
	void SlowDown() { TimeScale = FMath::Max(0.05f, TimeScale * 0.5f); }
	void SpeedUp() { TimeScale = FMath::Min(4.f, TimeScale * 2.f); }

	/** Frame the station, which is where an operator stands. */
	void FrameStation();

	/**
	 * THE MAIN MENU. The first screen anyone sees, and the first thing that makes
	 * this an application rather than a project.
	 *
	 * Drawn on the same canvas as every other panel and clicked the same
	 * immediate-mode way, so it needs no asset and works the moment somebody runs
	 * it. The list's own rules — most-recent-first, deduplicated across three
	 * spellings of one path, a missing file kept and marked rather than pruned,
	 * and a file that will not load carrying the loader's REASON — are in
	 * Prototypes/Shell/TrackBrowser.h and are tested there.
	 */
	FTrackBrowser Browser;

	/** Where each menu row was drawn, so a click can find it. Immediate mode:
	 *  rebuilt every frame, nothing to keep in sync. */
	TArray<FVector4> MenuRowRects;
	TArray<int32> MenuRowAction;   // >=0 template, -1000-n recent index, -1 open

	/**
	 * THE SEGMENT EDITOR, AS A RUNTIME PANEL.
	 *
	 * The Details panel is the developer path; this is the shipping one, and
	 * constraint 1 still holds absolutely — numeric entry only, no drag handles
	 * in the viewport, ever.
	 *
	 * TYPED, NOT NUDGED. A numeric field needs digits, a decimal point, a minus
	 * sign and backspace, and every one of those is a bindable key — so this is
	 * genuine typed entry rather than a stepper wearing its name. What it does
	 * not have is selection, clipboard or IME, which is why the field shows a
	 * caret and edits from the right rather than pretending to be a text box.
	 *
	 * Which fields show, and what happens to a value whose field stops showing,
	 * are decided in Prototypes/Shell/SegmentEditorModel.h and tested there:
	 * HIDDEN IS NOT DELETED, so flipping Arc to Straight and back keeps a radius.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Editor")
	bool bShowSegmentEditor = false;

	/** Which field is taking keystrokes, or Count for none. */
	EEditField FocusedField = EEditField::Count;

	/** What has been typed so far. Committed on Enter, discarded on Escape —
	 *  never applied per keystroke, because "3" on the way to "30" is a segment
	 *  3 m long and a rebuild nobody asked for. */
	FString FieldBuffer;

	TArray<FVector4> EditorRowRects;
	TArray<int32> EditorRowField;    // EEditField as int, or -1000-n for a segment row

	void DrawSegmentEditor(class UCanvas* Canvas);
	void ClickSegmentEditor();
	void TypeDigit(int32 D);
	void TypePoint();
	void TypeMinus();
	void TypeBackspace();

	/**
	 * CONTEXT, WHICH IS THE PART PEOPLE LEAVE OUT — and `FInputMap`'s own test
	 * says so: it is why one key can mean two things without either being a
	 * mistake, and why a GLOBAL binding conflicts with every context.
	 *
	 * Three keys are genuinely wanted twice here. Backspace is the emergency stop
	 * AND deletes a digit; the full stop steps one scan AND types a decimal
	 * point. Binding both to each would fire both — and an operator typing a
	 * radius would E-STOP THE RIDE, which is not a UI wart, it is the worst kind
	 * of surprise this project can produce.
	 *
	 * So they dispatch on whether a field has focus. Editing wins while editing;
	 * the operator's controls win the rest of the time.
	 */
	bool IsTypingInField() const { return FocusedField != EEditField::Count; }

	/**
	 * [ and ] — STEP THE SELECTION, and show where it went.
	 *
	 * Clicking a row was the only way to move the selection, which is fine when
	 * you know the index and useless when finding it is the whole problem. The
	 * arc-length column answers "where is segment 12"; this answers "walk me
	 * along until I recognise it".
	 *
	 * IT FRAMES ONLY IN ORBIT. Yanking the camera off a train mid-lap because
	 * somebody pressed a bracket is a worse surprise than not moving — and orbit
	 * is the mode where looking at the layout is what you are doing.
	 */
	void StepSelection(int32 Delta);
	void SelectPrevSegment() { StepSelection(-1); }
	void SelectNextSegment() { StepSelection(+1); }
	void KeyBackspace();
	void KeyPeriod();

	/** Ten of them, because BindKey wants a no-argument method. Tedious and
	 *  honest: the alternative is a lambda per key doing the same thing. */
	void Type0() { TypeDigit(0); } void Type1() { TypeDigit(1); }
	void Type2() { TypeDigit(2); } void Type3() { TypeDigit(3); }
	void Type4() { TypeDigit(4); } void Type5() { TypeDigit(5); }
	void Type6() { TypeDigit(6); } void Type7() { TypeDigit(7); }
	void Type8() { TypeDigit(8); } void Type9() { TypeDigit(9); }
	void CommitField();
	void CancelField();
	void ToggleSegmentEditor() { bShowSegmentEditor = !bShowSegmentEditor; CancelField(); bHideOverlays = false; }

	/** The authored value behind a field, and the way back. One place, so the
	 *  panel and the model cannot disagree about which float is which. */
	double ReadField(const FTUTrackSegment& S, EEditField F) const;
	void WriteField(FTUTrackSegment& S, EEditField F, double V);
	static EEditKind KindOf(ETUSegmentKind K);

	void DrawMainMenu(class UCanvas* Canvas);
	void ClickMainMenu();

	/**
	 * ONE BINDING FOR THE LEFT MOUSE, dispatching by mode.
	 *
	 * Two handlers bound to one key both fire, and which one "wins" depends on
	 * registration order — the exact ambiguity FInputMap's conflict test exists
	 * to warn about, and it would be poor form to ship the thing the test warns
	 * against in the same repository.
	 */
	void ClickPrimary();

	/** Load a template — which names a PRESET rather than carrying its own
	 *  geometry, because five measured worked examples already ship. */
	void StartFromTemplate(int32 Index);

	/** [Tab] — Build to Operate and back. The camera, the panels and whether
	 *  edits are accepted all follow from it, which is the point of a mode. */
	void CycleAppMode();

	// ===================== WHAT THE SHELL IS ALLOWED TO SEE =====================
	//
	// A deliberate public island in a private section. Everything around it is
	// implementation the UI has no business touching; these four reads and one
	// command are the whole surface a frame widget needs, and naming that
	// surface is cheaper than making the class public and hoping.
public:
	/**
	 * Enter a mode directly — what a clicked tab calls, where [Tab] calls
	 * CycleAppMode. BOTH GO THROUGH ApplyAppMode, so the keyboard and the mouse
	 * cannot end up with two ideas of what a mode does.
	 *
	 * `bConfirmed` is passed straight to FSession::Enter and means "the person
	 * has been asked about unsaved work and said yes". A caller that passes true
	 * without having asked is lying to a class that cannot tell, which is why
	 * MayEnter is a separate call the shell is expected to make first.
	 */
	void EnterAppMode(EAppMode Wanted, bool bConfirmed = false);

	/**
	 * THE SHELL'S FRAME. Created once at BeginPlay and given this actor to read.
	 *
	 * A class reference rather than a hard-coded spawn, so the layout asset can
	 * be swapped without touching code — which is the whole point of the split
	 * that puts behaviour in C++ and layout in the asset.
	 *
	 * NULL IS A VALID STATE. The debug-canvas panels still work and are still the
	 * only UI in a build where this asset is missing; a shell that refused to run
	 * without its chrome would make the asset a hard dependency of the ride,
	 * which it is not.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|UI")
	TSubclassOf<class UTUFrameWidget> FrameWidgetClass;

private:
	UPROPERTY(Transient)
	TObjectPtr<class UTUFrameWidget> FrameWidget;

public:

	/**
	 * THE FRAME'S READ-ONLY WINDOW ONTO THE RIDE.
	 *
	 * Accessors rather than public members: the UI needs to READ four things and
	 * has no business writing any of them. Session in particular is a state
	 * machine whose whole value is that its rules are in one place — handing a
	 * widget a mutable reference to it would put mode logic in the widget within
	 * a week.
	 */
	const FSession& GetSession() const { return Session; }
	int32 NumTrains() const { return Trains.Num(); }
	int32 GetScanOverruns() const { return ScanOverruns; }
	double GetScanTimeDroppedS() const { return ScanTimeDroppedS; }

private:

	/** What the shell is showing, for the mode banner. */
	void DrawModeBanner(class UCanvas* Canvas);
	void BuildDiagnostics();

	/** What a mode DOES, once the session has allowed it. */
	void ApplyAppMode(EAppMode Want);
	void DrawDiagnosticsPanel(class UCanvas* Canvas);
	void ToggleDiagnostics() { bShowDiagnostics = !bShowDiagnostics; bHideOverlays = false; }
	void CycleProfileChannel();
	void ToggleProfileGraph() { bShowProfileGraph = !bShowProfileGraph; bHideOverlays = false; }

	/**
	 * [F2] — EVERY OVERLAY OFF, AND BACK EXACTLY AS IT WAS.
	 *
	 * A MASTER GATE, NOT A SAVE-AND-RESTORE. The obvious version snapshots the
	 * eight toggles, clears them, and puts them back — and it is wrong the moment
	 * anything else touches one while hidden, which is a bug that only shows up
	 * as "my panel came back that I had turned off". One bool consulted at draw
	 * time restores the exact previous state for free, because nothing was ever
	 * changed.
	 *
	 * What it does NOT hide is the track mesh, the trains, or the main menu.
	 * Those are the product; this hides the instrumentation drawn over it.
	 *
	 * The wireframe and the ride-profile trace are PERSISTENT debug lines drawn
	 * once at BeginPlay, so they have to be flushed and redrawn rather than
	 * merely skipped — which is why this is a function and not a one-line lambda.
	 */
	bool bHideOverlays = false;
	void ToggleOverlays();

	/** [F1] — the settings screen, hosted by the frame. */
	void ToggleSettings();

	/** [F] — frame the whole track. The thing you press constantly when a
	 *  validation warning points somewhere and you have no idea where. */
	void FrameWholeTrack();

	/** Mouse wheel. Multiplicative, because the same notch has to mean the same
	 *  proportion of the distance at 10 m and at 1000 m. */
	void OrbitZoomIn() { Orbit.Zoom(0.9); }
	void OrbitZoomOut() { Orbit.Zoom(1.0 / 0.9); }
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
	/**
	 * WASDQE IS POLLED, NOT BOUND AS AN AXIS.
	 *
	 * BindAxisKey asserts on IsAxis1D() and a letter key is a BUTTON — the six
	 * binds this replaces were always wrong, and only survived because `ensure`
	 * logs rather than kills. MouseX and MouseY are genuine 1D axes and stay
	 * bound; those are the only two here that ever were.
	 *
	 * Polling is also simply less: six bindings and six one-line methods become
	 * one function, and there is no accumulate-then-reset dance to get wrong.
	 */
	void PollMovementKeys();
	void AxisLookYaw(float V) { LookYaw += V; }
	void AxisLookPitch(float V) { LookPitch += V; }
};
