// The authored segment, reflected for the editor.
//
// This is FAuthoredSegment (Prototypes/TrackSpline/TrackIO.h) with UPROPERTYs
// on it, and nothing else. The maths, the validation and the save format all
// live in the prototypes; this exists so Unreal's Details panel can edit them.
//
// Using the Details panel IS the numeric entry UI. It already has typed fields
// with unit-aware spinners, add/remove/reorder, copy/paste, multi-select and
// its own undo — reimplementing that in Slate would be a large amount of code
// to arrive back where we started. It also happens to satisfy the project's
// first architectural constraint exactly: numbers in a list, no drag handles
// anywhere near the viewport.
//
// UNITS: metres and DEGREES, matching FAuthoredSegment. Not centimetres — the
// conversion to Unreal's units happens once, at ToWorld, and nowhere else.
// Curvature stays 1/m because it is not an angle: an easement out of a straight
// has an endpoint with no radius, and 0 says that where a radius would need
// infinity.

#pragma once

#include "CoreMinimal.h"

#include "TrackSpline/TrackIO.h"

#include "TUTrackSegment.generated.h"

UENUM(BlueprintType)
enum class ETUSegmentKind : uint8
{
	Straight UMETA(DisplayName = "Straight"),
	Arc UMETA(DisplayName = "Arc (constant radius)"),
	Clothoid UMETA(DisplayName = "Clothoid (transition)"),
	Helix UMETA(DisplayName = "Helix"),
	Raw UMETA(DisplayName = "Raw curvature (hills, imports)"),
};

UENUM(BlueprintType)
enum class ETURollMode : uint8
{
	// Measured from the rotation-minimising frame. Defined everywhere,
	// including vertical and inverted track. Roll 0 is NOT level on
	// non-planar track — see Docs/GLOSSARY.md, "holonomy".
	PathRelative UMETA(DisplayName = "Roll (path-relative)"),
	// Measured from the horizon: what a spirit level reads. What a human means
	// by "bank this turn 30 degrees". Undefined pointing straight up, so do not
	// use it through an inversion.
	WorldBank UMETA(DisplayName = "Bank (from horizon)"),
};

// Powered track. One shape covers all of them — they differ only in target
// speed and in how hard they may push or hold back. Derived into FTrackZone by
// contiguous run, so a lift is however many segments in a row say "Lift".
UENUM(BlueprintType)
enum class ETUSegmentZone : uint8
{
	None UMETA(DisplayName = "Unpowered"),
	Lift UMETA(DisplayName = "Lift chain"),
	Launch UMETA(DisplayName = "Launch"),

	// Friction or magnetic brakes ONLY. Can slow a train to any speed including
	// a stop, and can never start one again — so it is a TRIM, and a train
	// commanded to hold here stands for the rest of the session.
	Brake UMETA(DisplayName = "Brake run (trim only)"),

	// Brakes AND drive tyres, which is what every real block brake is. Identical
	// in physics to a lift chain: it can hold a train against gravity and push it
	// away again, which is the pair of authorities a dispatcher needs before it
	// may stop a train anywhere.
	//
	// A separate enumerator rather than a reused Lift because block boundaries
	// fall where the KIND changes — two holding devices in a row have to stay two
	// blocks, or the queueing position they exist to provide merges away.
	//
	// Whether it can stop the train in the distance available is a LAYOUT
	// question this cannot answer: v^2/2a against the block length. On the
	// two-train preset the mid-course brake fails that test by a factor of 1.5,
	// which is why it is authored Brake and not this.
	BlockBrake UMETA(DisplayName = "Block brake (brakes + drive tyres)"),

	// Drive tyres where riders board. Physically identical to a block brake — the
	// same pair of authorities, the same MakeLift — and a separate enumerator for
	// exactly the reason BlockBrake is one: block boundaries fall where the KIND
	// changes.
	//
	// Without it a station authored as Lift MERGES INTO THE LIFT HILL behind it,
	// because a run is a contiguous stretch of the same kind and the two are then
	// the same kind. That is what the reference layout did, and it is a signalling
	// defect rather than a cosmetic one: a station and a lift sharing one block
	// means no train can be in the station while another is on the lift, which is
	// the whole reason a real ride puts a boundary between them.
	//
	// APPENDED, NOT INSERTED. This is a uint8 UENUM whose values are serialised
	// into every level that has a track in it; inserting in the middle would
	// silently renumber every zone already authored.
	Station UMETA(DisplayName = "Station (drive tyres, riders board)"),
};

// Starter layouts. Each one is a worked example of the authored vocabulary
// rather than decoration, and each is checked before shipping for the things
// that make a preset worth having: it closes back to station height, it is C2,
// it does not pass through itself, and the train actually gets round.
UENUM(BlueprintType)
enum class ETUPresetLayout : uint8
{
	// Launch, coast, brake. Three straights and nothing else — the smallest
	// complete ride, and the rig the rolling-resistance calibration was measured
	// on. Start here to see what a segment list even is.
	FlatRig UMETA(DisplayName = "Flat rig (launch, coast, brake)"),

	// Lift, drop, an airtime hill, a banked turnaround, brakes. No inversion, and
	// unlike the reference layout it does not pass through itself — closest
	// approach 7.77 m against the reference's 0.19. Gentler and friendlier.
	OutAndBack UMETA(DisplayName = "Out and back (airtime, no inversion)"),

	// The full thing: eased lift, 34 degree drop, teardrop loop, banked turn.
	// Every figure quoted in Docs/REFERENCE_LAYOUT.md comes from this.
	Reference UMETA(DisplayName = "Reference layout (loop + banked turn)"),

	// The only layout that can run two trains, and the only CLOSED one — it comes
	// back to the station in position, heading and roll, not merely in height, so
	// a lap is a lap rather than a jump. An oval: launch and climb out, level
	// turnaround at the top of the hill, drop and airtime back, turnaround at
	// station height, in.
	//
	// FIVE places a train may be held — station, mid-course, and three across the
	// pre-station approach — so a queue forms outside the station rather than on
	// open course. Every other preset has three blocks and ONE holding place: a
	// block boundary only falls where a powered run starts or ends, they each have
	// just two powered runs, and the second is a trim brake, which can stop a train
	// and never start one again. Set TrainCount above 1 on those and the extras are
	// refused.
	TwoTrainCircuit UMETA(DisplayName = "Two-train circuit (closed, 8 blocks)"),
};

UENUM(BlueprintType)
enum class ETUCameraMode : uint8
{
	// At the chosen seat, in the rider's own frame — so it inverts through the
	// loop, which is the point.
	Rider UMETA(DisplayName = "Rider (on-ride)"),

	// Behind and above, held level with the world. Deliberately NOT the rider's
	// frame: a chase camera that inherits roll turns upside down through an
	// inversion, which is disorienting rather than dramatic and is what the old
	// non-ride view did.
	Chase UMETA(DisplayName = "Chase"),

	// Detached and flown by hand. The ride carries on without you, which is the
	// point — standing beside the loop watching a train come through is a thing
	// only a free camera can do.
	Free UMETA(DisplayName = "Free (WASD, mouse, Q/E, Shift)"),
};

USTRUCT(BlueprintType)
struct FTUTrackSegment
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Segment")
	ETUSegmentKind Kind = ETUSegmentKind::Straight;

	/** Metres along the track. A helix derives its own length from radius, climb and turns. */
	UPROPERTY(EditAnywhere, Category = "Segment", meta = (ClampMin = "0.05", UIMin = "1.0",
		EditCondition = "Kind != ETUSegmentKind::Helix", EditConditionHides))
	float Length = 20.f;

	/** Metres. Positive turns left, negative turns right. */
	UPROPERTY(EditAnywhere, Category = "Segment", meta = (UIMin = "-200.0", UIMax = "200.0",
		EditCondition = "Kind == ETUSegmentKind::Arc || Kind == ETUSegmentKind::Helix",
		EditConditionHides))
	float Radius = 30.f;

	/** 1/metres. Zero is straight — which is why this is curvature and not a radius. */
	UPROPERTY(EditAnywhere, Category = "Segment",
		meta = (EditCondition = "Kind == ETUSegmentKind::Clothoid", EditConditionHides))
	float CurvatureStart = 0.f;

	UPROPERTY(EditAnywhere, Category = "Segment",
		meta = (EditCondition = "Kind == ETUSegmentKind::Clothoid", EditConditionHides))
	float CurvatureEnd = 0.0333f;

	/** Degrees. Positive ascends. */
	UPROPERTY(EditAnywhere, Category = "Segment", meta = (UIMin = "-60.0", UIMax = "60.0",
		EditCondition = "Kind == ETUSegmentKind::Helix", EditConditionHides))
	float ClimbAngleDegrees = 15.f;

	/** Revolutions about the helix axis. */
	UPROPERTY(EditAnywhere, Category = "Segment", meta = (UIMin = "0.1", UIMax = "5.0",
		EditCondition = "Kind == ETUSegmentKind::Helix", EditConditionHides))
	float Turns = 1.f;

	/** Degrees at the start of the segment. See RollMode for what it is measured FROM. */
	UPROPERTY(EditAnywhere, Category = "Roll", meta = (UIMin = "-180.0", UIMax = "180.0"))
	float RollStartDegrees = 0.f;

	UPROPERTY(EditAnywhere, Category = "Roll", meta = (UIMin = "-180.0", UIMax = "180.0"))
	float RollEndDegrees = 0.f;

	// Arcs and clothoids default to bank-from-horizon, because banking a turn is
	// the commonest thing anyone authors and "bank 30 degrees" should mean 30
	// degrees off level. Everything else defaults to path-relative, which is the
	// mode that is defined everywhere — inversions are exactly where a horizon
	// reference stops meaning anything.
	UPROPERTY(EditAnywhere, Category = "Roll")
	ETURollMode RollMode = ETURollMode::PathRelative;

	// -------- Raw only: the curvature profile itself, for hills and imports.

	UPROPERTY(EditAnywhere, Category = "Raw curvature",
		meta = (EditCondition = "Kind == ETUSegmentKind::Raw", EditConditionHides))
	float YawCurvatureStart = 0.f;

	UPROPERTY(EditAnywhere, Category = "Raw curvature",
		meta = (EditCondition = "Kind == ETUSegmentKind::Raw", EditConditionHides))
	float YawCurvatureEnd = 0.f;

	UPROPERTY(EditAnywhere, Category = "Raw curvature",
		meta = (EditCondition = "Kind == ETUSegmentKind::Raw", EditConditionHides))
	float PitchCurvatureStart = 0.f;

	UPROPERTY(EditAnywhere, Category = "Raw curvature",
		meta = (EditCondition = "Kind == ETUSegmentKind::Raw", EditConditionHides))
	float PitchCurvatureEnd = 0.f;

	/** 1/metres. Rotates the curvature vector about the tangent — what makes a helix a helix. */
	UPROPERTY(EditAnywhere, Category = "Raw curvature",
		meta = (EditCondition = "Kind == ETUSegmentKind::Raw", EditConditionHides))
	float Torsion = 0.f;

	// -------- Powered track.

	UPROPERTY(EditAnywhere, Category = "Zone")
	ETUSegmentZone Zone = ETUSegmentZone::None;

	/** Metres/second. Chain speed for a lift, target for a launch, release speed for a brake. */
	UPROPERTY(EditAnywhere, Category = "Zone", meta = (ClampMin = "0.0", UIMax = "60.0",
		EditCondition = "Zone != ETUSegmentZone::None", EditConditionHides))
	float ZoneSpeed = 4.f;

	/**
	 * Ratchets, chain dogs or a catch car: a train cannot roll backwards through
	 * this segment.
	 *
	 * NOT a zone, and deliberately a separate field rather than another
	 * ETUSegmentZone. A zone is a control device — it has a speed, an authority,
	 * and something commanding it. This is a SAFETY device: no speed, nothing to
	 * command, and it fails closed by being passive. They also overlap freely,
	 * because a lift hill is both at once.
	 *
	 * It does nothing at all while a ride works, which is the point. It only
	 * matters the day a hill is too tall, and then it is the difference between a
	 * train held on the lift and a train loose on the track going the wrong way.
	 */
	UPROPERTY(EditAnywhere, Category = "Zone")
	bool bAntiRollback = false;
};

// Reflected editor struct -> the authored model the prototypes understand.
// One direction only, matching BuildSegment: this is a view onto the data, not
// a second copy of it.
inline FAuthoredSegment ToAuthored(const FTUTrackSegment& S)
{
	FAuthoredSegment A;
	switch (S.Kind)
	{
	case ETUSegmentKind::Straight: A.Kind = ESegmentKind::Straight; break;
	case ETUSegmentKind::Arc: A.Kind = ESegmentKind::Arc; break;
	case ETUSegmentKind::Clothoid: A.Kind = ESegmentKind::Clothoid; break;
	case ETUSegmentKind::Helix: A.Kind = ESegmentKind::Helix; break;
	case ETUSegmentKind::Raw: A.Kind = ESegmentKind::Raw; break;
	}
	A.Length = S.Length;
	A.Radius = S.Radius;
	A.CurvatureStart = S.CurvatureStart;
	A.CurvatureEnd = S.CurvatureEnd;
	A.ClimbAngleDegrees = S.ClimbAngleDegrees;
	A.Turns = S.Turns;
	A.RollStartDegrees = S.RollStartDegrees;
	A.RollEndDegrees = S.RollEndDegrees;
	A.RollMode = S.RollMode == ETURollMode::WorldBank ? ERollMode::WorldBank
													 : ERollMode::PathRelative;

	A.RawSegment.Length = S.Length;
	A.RawSegment.YawCurvatureStart = S.YawCurvatureStart;
	A.RawSegment.YawCurvatureEnd = S.YawCurvatureEnd;
	A.RawSegment.PitchCurvatureStart = S.PitchCurvatureStart;
	A.RawSegment.PitchCurvatureEnd = S.PitchCurvatureEnd;
	A.RawSegment.Torsion = S.Torsion;
	return A;
}
