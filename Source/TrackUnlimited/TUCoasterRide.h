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

#include "TUCoasterRide.generated.h"

class UCameraComponent;
class UStaticMeshComponent;

UCLASS()
class TRACKUNLIMITED_API ATUCoasterRide : public APawn
{
	GENERATED_BODY()

public:
	ATUCoasterRide();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Ride from the seat rather than watching from outside. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	bool bRideCamera = true;

	/** Draw the heartline and rail centreline, since there is no track mesh yet. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	bool bDrawTrack = true;

	/** Speed, G-forces and block state on screen. The point of the slice. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	bool bShowTelemetry = true;

	/** Restart the ride this many seconds after the train comes to rest. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited")
	float RestartDelaySeconds = 3.f;

private:
	void BuildTrack();
	void DrawTrack() const;

	/** Prototype metres/right-handed -> Unreal centimetres/left-handed. */
	FVector ToWorld(const FVec3& V) const;
	FQuat ToWorldRotation(const FTrackFrame& Frame) const;

	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited")
	TObjectPtr<USceneComponent> Root;

	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited")
	TObjectPtr<UStaticMeshComponent> Cart;

	UPROPERTY(VisibleAnywhere, Category = "TrackUnlimited")
	TObjectPtr<UCameraComponent> Camera;

	// Not UPROPERTYs: plain C++ with no reflection needed, and deliberately so.
	FTrack Track;
	TUniquePtr<FTrain> Train;

	// Landmarks along the track, in metres, filled in by BuildTrack.
	double BrakeStartS = 0.0;
	float StoppedFor = 0.f;
};
