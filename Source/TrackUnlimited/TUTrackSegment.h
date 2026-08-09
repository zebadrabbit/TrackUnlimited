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

	// A platform split in two. High-throughput rides put riders OFF at one and ON
	// at another, with track between them, often themed as different scenes — and
	// they have to be separate blocks, because the entire point is emptying one
	// train while another is still being filled.
	//
	// The difference is not decorative: an UNLOAD platform releases its train with
	// the restraints still OPEN, since it runs empty to the load platform ready to
	// board. Demanding locked restraints there would deadlock the ride — nobody is
	// aboard to close them.
	StationUnload UMETA(DisplayName = "Station: unload only"),
	StationLoad UMETA(DisplayName = "Station: load only"),
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

	// The same closed oval, re-zoned for a SMALL-BATCH operation: 6 m vehicles, an
	// unload platform and THREE loading positions, so the ride runs a queue of
	// trains through a station rather than one at a time.
	//
	// Identical geometry, deliberately. That shape closes to 0.000000 m because of
	// its LEG LENGTHS, and leg A is 176 m of flat whether it is one 26 m station
	// plus a 150 m launch or four short platforms plus a 136 m one. Keep the total
	// and the closure comes along unchanged, along with every G figure measured
	// against it.
	//
	// EIGHT places a train may be held against the two-train circuit's five, so it
	// carries seven. Loading three at once is the point: a rider who needs longer
	// to board holds up only the trains BEHIND them — measured at 52 extra seconds
	// costing the ride 5.5 s at the back and the full 52 at the front.
	SmallBatch UMETA(DisplayName = "Small batch (unload + 3 loading positions)"),
};

// Which side of the track an evacuation catwalk runs down.
//
// APPENDED-SAFE by construction: this is a new enum rather than an addition to
// an existing one, so nothing already serialised into a level can be renumbered
// by it. If values are ever added, append them — `ETUSegmentZone` is the
// cautionary tale.
UENUM(BlueprintType)
enum class ETUWalkway : uint8
{
	None UMETA(DisplayName = "None"),
	Left UMETA(DisplayName = "Left side"),
	Right UMETA(DisplayName = "Right side"),
	Both UMETA(DisplayName = "Both sides"),
};

// Who is looking at the control panel.
//
// NOT a difficulty setting, and not the same UI with fewer buttons. It is the
// split every real installation already has: the operator station a ride is
// dispatched from, and the engineering page it is diagnosed from. An operator
// console does not show motor current — that lives on a maintenance HMI, and
// putting it on the dispatch screen is as wrong as leaving it out entirely.
//
// So each view answers a different question. The operator's is "may this train
// go, and if not, what is holding it". Maintenance asks "what is this machine
// actually doing", which needs the numbers behind the same facts.
UENUM(BlueprintType)
enum class ETUPanelView : uint8
{
	Off UMETA(DisplayName = "Off"),

	// What the ride is doing, in states rather than quantities: where the trains
	// are, what each platform is waiting for, whether a drive is running.
	Operator UMETA(DisplayName = "Operator (dispatch)"),

	// The same panel with the instrumentation behind it — commanded against
	// output against motor, torque, fault acknowledgement, and the second
	// detection method's own counts to read against the interlocking's.
	Maintenance UMETA(DisplayName = "Maintenance (engineering)"),
};

// A TRACK STYLE: the cross-section's dimensions and what it is painted.
//
// One struct, because a style is not two decisions. Change a gauge without
// changing the colour and you have a different track that looks the same, which
// is exactly the confusion a "style" is supposed to prevent.
//
// PER CONSTRAINT 5, every figure here is generic. Real steel coaster track spans
// roughly 0.76-1.22 m gauge, 100-127 mm running rail with ~10 mm wall, and
// tubular spines to ~0.36 m; the presets below sit inside that and are named for
// what they are rather than for who builds them.
USTRUCT(BlueprintType)
struct FTUTrackStyle
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Style")
	FString Name = TEXT("Steel - modern");

	// ---- Colour. Three, because the mesher emits three buffers and that was the
	// reason: running rail is polished where the wheels touch it, spine is painted
	// structure, ties are usually neither.
	UPROPERTY(EditAnywhere, Category = "Style")
	FLinearColor RailColour = FLinearColor(0.62f, 0.65f, 0.70f);

	UPROPERTY(EditAnywhere, Category = "Style")
	FLinearColor SpineColour = FLinearColor(0.86f, 0.31f, 0.16f);

	UPROPERTY(EditAnywhere, Category = "Style")
	FLinearColor TieColour = FLinearColor(0.30f, 0.32f, 0.35f);

	// ---- Section. The same knobs FTrackProfile carries, surfaced so a style is
	// one thing to pick rather than a colour plus six numbers somewhere else.
	UPROPERTY(EditAnywhere, Category = "Style", meta = (ClampMin = "0.4", ClampMax = "2.0"))
	float GaugeM = 1.10f;

	UPROPERTY(EditAnywhere, Category = "Style", meta = (ClampMin = "0.05", ClampMax = "0.3"))
	float RailDiameterM = 0.115f;

	UPROPERTY(EditAnywhere, Category = "Style", meta = (ClampMin = "0.05", ClampMax = "2.0"))
	float SpineDropM = 0.45f;

	UPROPERTY(EditAnywhere, Category = "Style", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float SpineDiameterM = 0.35f;

	UPROPERTY(EditAnywhere, Category = "Style", meta = (ClampMin = "0.3", ClampMax = "5.0"))
	float TieSpacingM = 1.00f;

	UPROPERTY(EditAnywhere, Category = "Style", meta = (ClampMin = "0.02", ClampMax = "0.2"))
	float TieDiameterM = 0.05f;
};

// The shipped styles. Named for what they ARE.
UENUM(BlueprintType)
enum class ETUTrackStyleName : uint8
{
	// Slim rails, a deep box-ish spine, ties every metre. What most modern steel
	// track looks like at a distance.
	SteelModern UMETA(DisplayName = "Steel - modern"),

	// Fatter rails, a shallower and heavier spine, ties closer together. Reads as
	// older ironwork without being anybody's in particular.
	SteelClassic UMETA(DisplayName = "Steel - classic tubular"),

	// Narrow gauge, small section, tight tie spacing. For the small-batch
	// vehicles the preset already ships.
	SteelCompact UMETA(DisplayName = "Steel - compact / family"),
};

// Which trace the ride-profile graph is showing. ONE AT A TIME rather than four
// overlaid, because four traces on one axis is a picture rather than a reading —
// and per-channel scale is exactly what the Phase 1 legibility card asked for.
UENUM(BlueprintType)
enum class ETUProfileChannel : uint8
{
	VerticalG UMETA(DisplayName = "Vertical G"),
	LateralG UMETA(DisplayName = "Lateral G"),
	Speed UMETA(DisplayName = "Speed"),

	// The one felt G structurally cannot contain: it models the rider as a point
	// at the heartline, so spinning that point costs exactly nothing.
	RollRate UMETA(DisplayName = "Roll rate"),
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

	// STANDING AT THE CONSOLE, on the platform, where an operator actually works.
	//
	// APPENDED, NOT INSERTED. This is a uint8 UENUM serialised into every level
	// that has a ride in it; inserting in the middle would silently renumber every
	// camera already chosen.
	//
	// It gives up the view Operate's orbit was chosen for — far enough back to see
	// the train AND the block ahead of it — and that is the right trade rather than
	// a regression: a real operator cannot see the mid-course brake either. They
	// read it off the block schematic, which the panel already draws, and the
	// difference is that here the schematic stops being a nicety and becomes the
	// instrument. Orbit is still one press away for when you want the god's eye.
	Console UMETA(DisplayName = "Console (at the platform)"),

	// Detached and flown by hand. The ride carries on without you, which is the
	// point — standing beside the loop watching a train come through is a thing
	// only a free camera can do.
	Free UMETA(DisplayName = "Free (WASD, mouse, Q/E, Shift)"),

	// Around a point, which is what you want while EDITING rather than watching.
	// Free-fly is for going somewhere; orbit is for looking at the thing you are
	// already at, and every 3D editor has both because neither does the other's
	// job. [F] frames the whole track, and the framing arithmetic — which checks
	// BOTH screen axes, and uses the bounding sphere so orbiting cannot lose the
	// subject — is tested in Prototypes/Shell/CameraRig.h.
	Orbit UMETA(DisplayName = "Orbit (drag to turn, wheel to zoom, F to frame)"),
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
	/**
	 * WHAT THIS DEVICE CAN PUSH AND PULL WITH, m/s². Authored, because it is a
	 * property of the hardware somebody specified.
	 *
	 * These were one hardcoded 6.0 for every device on every ride — the same
	 * defect as the merged zone speeds before them, a number nobody typed
	 * standing in for one they should have. A chain hauling at 0.61 g is nothing
	 * like a real chain; a friction brake at 0.61 g is weaker than the 0.8 g a
	 * real one is specified at.
	 *
	 * WHICH ONE APPLIES IS STILL THE KIND'S DECISION. A trim has no tractive
	 * authority whatever is typed here and a launch has no braking authority —
	 * those are what the enumerators MEAN, and a number cannot grant an authority
	 * the device does not have. This sets the magnitude of the ones it does.
	 *
	 * 6.0 is the default on both, so every measured figure in the docs is
	 * unmoved until somebody types something.
	 */
	UPROPERTY(EditAnywhere, Category = "Zone", meta = (ClampMin = "0.1", UIMax = "15.0",
		EditCondition = "Zone != ETUSegmentZone::None", EditConditionHides))
	float ZoneAccel = 6.f;

	UPROPERTY(EditAnywhere, Category = "Zone", meta = (ClampMin = "0.1", UIMax = "15.0",
		EditCondition = "Zone != ETUSegmentZone::None", EditConditionHides))
	float ZoneDecel = 6.f;

	/**
	 * THE FRICTION PAD, WHICH IS A SECOND DEVICE, m/s². Zero means it has none.
	 *
	 * A real block brake is two machines on one stretch of track: a pad that can
	 * only ever remove energy, and drive tyres that push and hold. They are
	 * separate hardware, specified separately, and one can fail without the
	 * other — so a pad that bites at 8 m/s² feeding tyres that convey at 0.5 is
	 * an ordinary specification and was previously inexpressible, because a
	 * single rate has to be wrong for one of them.
	 *
	 * ZERO BY DEFAULT, so every preset builds the zones it always did and every
	 * canonical figure is unmoved. Author a rate and the pad appears.
	 *
	 * It is a CEILING, never a setpoint: below the commanded speed it does
	 * nothing at all, which is what makes it a brake. There is no way to author
	 * one that pushes.
	 */
	UPROPERTY(EditAnywhere, Category = "Zone", meta = (ClampMin = "0.0", UIMax = "15.0",
		EditCondition = "Zone != ETUSegmentZone::None", EditConditionHides))
	float ZoneBrakeDecel = 0.f;

	UPROPERTY(EditAnywhere, Category = "Zone")
	bool bAntiRollback = false;

	/**
	 * A new DEVICE starts here, even though the kind and the speed are the same as
	 * the segment before.
	 *
	 * A zone is a contiguous run of segments describing one machine, and normally
	 * the run ends where the kind or the speed changes — which is enough for almost
	 * everything, because two different devices are almost always different in one
	 * of those. This is for the case where they are not: three loading positions on
	 * one platform, or a queue of brake sections behind the scenes, where the
	 * devices are genuinely identical and are still separate machines with separate
	 * motors and separate blocks.
	 *
	 * Set it on the FIRST segment of the new device.
	 *
	 * Measured on a three-position platform: 52 extra seconds of loading at the
	 * REAR position costs the ride 5.5 s and does not delay the two trains in front
	 * by a single frame, where the same delay at the FRONT costs the full 52 —
	 * because positions are three loading bays, not a queue for one.
	 */
	UPROPERTY(EditAnywhere, Category = "Zone", meta = (
		EditCondition = "Zone != ETUSegmentZone::None", EditConditionHides))
	bool bStartsNewDevice = false;

};

/**
 * A solid decked evacuation catwalk with handrails, running alongside the track
 * from one point to another.
 *
 * A HUMAN PLACES THESE. Not derived, not inferred, and deliberately not offered
 * as a suggestion: where a walkway *should* go by the logic of "a train can stop
 * here" may be somewhere architecturally impossible to reach, structurally
 * unsupportable, or actively dangerous to stand. Guessing would produce a
 * confident answer about a physical structure this model knows nothing about.
 * The layout says where a walkway would HELP; only a person says where one goes.
 *
 * AUTHORED AS A START AND A STOP, which is why this is its own list rather than
 * a flag on FTUTrackSegment. A segment is 20–60 m of geometry chosen for the
 * shape of the ride; a catwalk begins and ends where the structure allows, which
 * is routinely partway along one. Snapping walkways to segment boundaries would
 * be an arbitrary constraint with nothing to do with where a person can walk.
 *
 * NOT A ZONE, for exactly the reasons bAntiRollback is not one. A zone is a
 * CONTROL device: it has a speed, an authority, and something commanding it
 * every scan. A catwalk has none of those, cannot release a train, and so is no
 * more a place to park one than a trim brake is. It OVERLAPS zones freely — a
 * launch with walkways down both sides is both at once, which is the usual case.
 *
 * SIDE MATTERS. A train is boarded and evacuated from a specific side, so a
 * catwalk on the far side of the track from the restraints is a walkway for
 * staff rather than an evacuation route. Reachability treats abutting spans of
 * different sides as one continuous route — a person steps across at the join —
 * so the side is reported rather than required to match.
 *
 * Rendering the deck, the railings and the lighting is Phase 4. The placement
 * and the check are not blocked on any of it.
 */
USTRUCT(BlueprintType)
struct FTUWalkway
{
	GENERATED_BODY()

	/**
	 * Metres along the track, from the start.
	 *
	 * ABSOLUTE ARC LENGTH, and the cost of that is worth knowing: lengthening a
	 * segment upstream shifts every walkway after it, because arc length is
	 * derived from the segment list. The alternative — anchoring to a segment
	 * index plus an offset — survives upstream edits and breaks differently when
	 * that segment is deleted or resized instead. Absolute was chosen because it
	 * is what a person placing a structure actually knows ("it runs from the
	 * brake run to the transfer"), and because a walkway is placed against a
	 * finished layout rather than during one.
	 */
	UPROPERTY(EditAnywhere, Category = "Walkway", meta = (ClampMin = "0.0"))
	float StartS = 0.f;

	UPROPERTY(EditAnywhere, Category = "Walkway", meta = (ClampMin = "0.0"))
	float EndS = 0.f;

	UPROPERTY(EditAnywhere, Category = "Walkway")
	ETUWalkway Side = ETUWalkway::Both;
};

// ===================== THE EDITOR STRUCT AND THE AUTHORED MODEL =====================
//
// TWO DIRECTIONS NOW, AND THAT IS NOT A REVERSAL OF THE OLD RULE. What has no
// inverse is `BuildSegment`: deriving "radius 20, 15 degrees, 2 turns" back out of
// an integrated curvature profile is the recovery problem the authored model
// exists to avoid needing to solve. These two are authored-to-authored — the same
// typed numbers in two spellings — so the pair IS lossless, and it has to be,
// because it is what a save and an open go through.
//
// The zone mapping is a THIRD list of device names, after the enum and the file's
// strings. It cannot be generated from either: one is a UENUM the reflection
// system owns and the other is text stored on disc for ever. What keeps it honest
// is that both switches are exhaustive with no `default:`, so adding a device
// fails to compile here rather than silently mapping to unpowered track.
inline EAuthoredZone ToAuthoredZone(ETUSegmentZone Z)
{
	switch (Z)
	{
	case ETUSegmentZone::None: return EAuthoredZone::None;
	case ETUSegmentZone::Lift: return EAuthoredZone::Lift;
	case ETUSegmentZone::Launch: return EAuthoredZone::Launch;
	case ETUSegmentZone::Brake: return EAuthoredZone::Brake;
	case ETUSegmentZone::BlockBrake: return EAuthoredZone::BlockBrake;
	case ETUSegmentZone::Station: return EAuthoredZone::Station;
	case ETUSegmentZone::StationUnload: return EAuthoredZone::StationUnload;
	case ETUSegmentZone::StationLoad: return EAuthoredZone::StationLoad;
	}
	return EAuthoredZone::None;
}

inline ETUSegmentZone FromAuthoredZone(EAuthoredZone Z)
{
	switch (Z)
	{
	case EAuthoredZone::None: return ETUSegmentZone::None;
	case EAuthoredZone::Lift: return ETUSegmentZone::Lift;
	case EAuthoredZone::Launch: return ETUSegmentZone::Launch;
	case EAuthoredZone::Brake: return ETUSegmentZone::Brake;
	case EAuthoredZone::BlockBrake: return ETUSegmentZone::BlockBrake;
	case EAuthoredZone::Station: return ETUSegmentZone::Station;
	case EAuthoredZone::StationUnload: return ETUSegmentZone::StationUnload;
	case EAuthoredZone::StationLoad: return ETUSegmentZone::StationLoad;
	}
	return ETUSegmentZone::None;
}

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

	// THE CONTROL LAYER. Absent here until 2026-08-08, which meant a saved track
	// was geometry with no brakes, no station and no lift — and nothing could see
	// it, because the ride is the same shape either way.
	A.Zone = ToAuthoredZone(S.Zone);
	A.ZoneSpeed = S.ZoneSpeed;
	A.ZoneAccel = S.ZoneAccel;
	A.ZoneDecel = S.ZoneDecel;
	A.ZoneBrakeDecel = S.ZoneBrakeDecel;
	A.bAntiRollback = S.bAntiRollback;
	A.bStartsNewDevice = S.bStartsNewDevice;

	A.RawSegment.Length = S.Length;
	A.RawSegment.YawCurvatureStart = S.YawCurvatureStart;
	A.RawSegment.YawCurvatureEnd = S.YawCurvatureEnd;
	A.RawSegment.PitchCurvatureStart = S.PitchCurvatureStart;
	A.RawSegment.PitchCurvatureEnd = S.PitchCurvatureEnd;
	A.RawSegment.Torsion = S.Torsion;
	return A;
}

// The direction an OPEN goes. Everything `ToAuthored` reads is written back, and
// the fields it does not carry keep the struct's own defaults — which is right:
// they are editor state (selection, the derived StartS/EndS) rather than anything
// somebody typed.
inline FTUTrackSegment FromAuthored(const FAuthoredSegment& A)
{
	FTUTrackSegment S;
	switch (A.Kind)
	{
	case ESegmentKind::Straight: S.Kind = ETUSegmentKind::Straight; break;
	case ESegmentKind::Arc: S.Kind = ETUSegmentKind::Arc; break;
	case ESegmentKind::Clothoid: S.Kind = ETUSegmentKind::Clothoid; break;
	case ESegmentKind::Helix: S.Kind = ETUSegmentKind::Helix; break;
	case ESegmentKind::Raw: S.Kind = ETUSegmentKind::Raw; break;
	}
	S.Length = static_cast<float>(A.Kind == ESegmentKind::Raw ? A.RawSegment.Length : A.Length);
	S.Radius = static_cast<float>(A.Radius);
	S.CurvatureStart = static_cast<float>(A.CurvatureStart);
	S.CurvatureEnd = static_cast<float>(A.CurvatureEnd);
	S.ClimbAngleDegrees = static_cast<float>(A.ClimbAngleDegrees);
	S.Turns = static_cast<float>(A.Turns);
	S.RollStartDegrees = static_cast<float>(A.RollStartDegrees);
	S.RollEndDegrees = static_cast<float>(A.RollEndDegrees);
	S.RollMode = A.RollMode == ERollMode::WorldBank ? ETURollMode::WorldBank
													: ETURollMode::PathRelative;

	S.Zone = FromAuthoredZone(A.Zone);
	S.ZoneSpeed = static_cast<float>(A.ZoneSpeed);
	S.ZoneAccel = static_cast<float>(A.ZoneAccel);
	S.ZoneDecel = static_cast<float>(A.ZoneDecel);
	S.ZoneBrakeDecel = static_cast<float>(A.ZoneBrakeDecel);
	S.bAntiRollback = A.bAntiRollback;
	S.bStartsNewDevice = A.bStartsNewDevice;

	S.YawCurvatureStart = static_cast<float>(A.RawSegment.YawCurvatureStart);
	S.YawCurvatureEnd = static_cast<float>(A.RawSegment.YawCurvatureEnd);
	S.PitchCurvatureStart = static_cast<float>(A.RawSegment.PitchCurvatureStart);
	S.PitchCurvatureEnd = static_cast<float>(A.RawSegment.PitchCurvatureEnd);
	// Torsion only means anything on a Raw segment — every other kind derives its
	// own, and the file stores these three only for Raw for exactly that reason.
	S.Torsion = static_cast<float>(A.RawSegment.Torsion);
	return S;
}
