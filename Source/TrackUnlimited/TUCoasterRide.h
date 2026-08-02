// Phase 0 vertical slice: a hand-authored track, a cart that follows it with
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

	/** Rebuild the reference layout, discarding edits. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|Track")
	bool bResetToReferenceLayout = false;

	/** The reference ride: station, eased 25 degree lift, drop, teardrop loop, banked turn, brakes. */
	static TArray<FTUTrackSegment> ReferenceLayout();

	/** Ride from the seat rather than watching from outside. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	bool bRideCamera = true;

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

	/**
	 * Metres, nose to tail. Zero is a point mass at the heartline.
	 *
	 * A train's speed is governed by the height of its centre of mass, so a
	 * long one softens sharp features — it does not pay the full height of a
	 * crest it is straddling. On this layout 15 m takes the peak vertical from
	 * +4.30 g to +4.20 and FIRMS the loop apex from +1.34 to +1.52, because
	 * straddling the top of a loop means more speed at the top.
	 */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited", meta = (ClampMin = "0.0", UIMax = "30.0"))
	float TrainLengthM = 15.f;

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
	TUniquePtr<FTrain> Train;

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
	float StoppedFor = 0.f;
};
