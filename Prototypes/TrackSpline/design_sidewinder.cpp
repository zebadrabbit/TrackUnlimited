// The SIDEWINDER, designed: the sixth template, and the first ride on this
// project built to a brief rather than to prove a feature.
//
//   clang++ -std=c++17 -O2 -Wall -Wextra -o design_sidewinder design_sidewinder.cpp
//
// THE BRIEF (developer, 2026-08-23): about 45 s, a train of 6-8 cars, a chain
// lift and a drop, banking left AND right, a helix, several small hills back
// to the station, block brakes enough for two trains moving and one staging,
// the station a little above the ground.
//
// THIS PROGRAM IS THE DESIGN. It authors the ride in the same vocabulary the
// editor and the presets use, SOLVES the two closure lengths rather than
// eyeballing them (the drop straight is the height lever, the fill straight
// the plan lever -- the same pair the two-train circuit uses), runs the ride
// through the physics, and prints every figure the preset and the docs quote.
// Run it before changing a number in SidewinderLayout().
//
// UX NOTES WERE THE OTHER DELIVERABLE. Everything here that the RUNTIME editor
// could not have done is marked UX: -- the list is on the Trello card.

#include "TrackIO.h"
#include "TrackClose.h"
#include "TrackValidate.h"
#include "../TrainPhysics/RideProfile.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double G = 9.81;
inline double Deg(double D) { return D * Pi / 180.0; }
inline double BankDeg(double SpeedMs, double R) { return std::atan2(SpeedMs * SpeedMs, G * R) * 180.0 / Pi; }

struct FDesign
{
    FTrackDocument Doc;
    std::size_t DropIndex = 0;     // the height lever
    std::size_t FillIndex = 0;     // the plan lever (along)
    std::size_t HelixIndex = 0;    // the heading lever -- see the helix note
    std::size_t Turn1ArcIndex = 0; // recorded for reference; see the snake for the across lever
    std::size_t SnakeArcIndex = 0; // the across lever -- see the snake
    std::size_t TrimIndex = 0;     // the return's TRUE along lever -- the one -X leg, see the trim
    std::size_t HomeRunIndex = 0;  // the height feedback's knob -- see the home descent
    std::size_t McbrIndex = 0;     // the RETURN's own along lever -- level, so height-neutral
    double HelixTurns = 0.0;       // the closed-form starting guess for the solver
    double SinT2Out = 0.0;         // what the payback climb actually used
    double T2HeadingOut = 0.0;     // the payback arc's plan turn, for the calibration step
    double HomeDropOut = 0.0;      // what the home descent actually used, for the feedback

    void Add(const FAuthoredSegment& A) { Doc.Segments.push_back(A); }

    std::size_t Straight(double L, EAuthoredZone Z = EAuthoredZone::None, double Speed = 4.0,
                         double Accel = 6.0, double Decel = 6.0, double Pad = 0.0)
    {
        FAuthoredSegment A; A.Kind = ESegmentKind::Straight; A.Length = L;
        A.Zone = Z; A.ZoneSpeed = Speed; A.ZoneAccel = Accel; A.ZoneDecel = Decel; A.ZoneBrakeDecel = Pad;
        Add(A); return Doc.Segments.size() - 1;
    }
    // UX: a vertical curve is two RAW segments with pitch curvature, which the
    // runtime editor cannot author at all -- Raw is not a cycle destination
    // and has no fields. Every hill in every preset comes from this helper.
    void EasedPitch(double DeltaRad, double K, EAuthoredZone Z = EAuthoredZone::None, double Speed = 4.0,
                    double Accel = 6.0)
    {
        const double Kk = DeltaRad >= 0 ? K : -K;
        const double L = std::fabs(DeltaRad) / K;
        FAuthoredSegment In; In.Kind = ESegmentKind::Raw; In.RawSegment.Length = L;
        In.RawSegment.PitchCurvatureEnd = Kk; In.Zone = Z; In.ZoneSpeed = Speed; In.ZoneAccel = Accel; In.ZoneDecel = Accel; Add(In);
        FAuthoredSegment Out; Out.Kind = ESegmentKind::Raw; Out.RawSegment.Length = L;
        Out.RawSegment.PitchCurvatureStart = Kk; Out.Zone = Z; Out.ZoneSpeed = Speed; Out.ZoneAccel = Accel; Out.ZoneDecel = Accel; Add(Out);
    }
    // (A "TwistPitch" -- EasedPitch carrying torsion as a twist trim -- lived
    // here for one measured hour. Frame twist is SOLID-ANGLE holonomy, and a
    // 12 degree pitch blip sweeps almost none: 0.25 rad of commanded torsion
    // moved the seam twist 0.0037 rad, a 1.5% gain. Only sustained turning
    // can twist the frame, and bending the payback to do it warps the ride.
    // The answer is neither: see the WorldBank tail at the end of
    // Sidewinder().)
    // Author the components for a desired WORLD curvature pair in a path frame
    // twisted by Rot about the tangent: authored = R(-Rot) x desired. The
    // integrator applies authored yaw about PathUp and pitch about PathLateral,
    // and a helix leaves those axes rolled by its holonomy -- this is how a
    // segment in the twisted region still bends the way its author meant.
    // Exact, not approximate: a constant rotation of a linear ramp is a linear
    // ramp, which is the whole of what a Raw segment can carry.
    static void Twisted(double YawW, double PitchW, double Rot, double& AyOut, double& ApOut)
    {
        AyOut = YawW * std::cos(Rot) + PitchW * std::sin(Rot);
        ApOut = -YawW * std::sin(Rot) + PitchW * std::cos(Rot);
    }
    // Pitch eased in and out WITHOUT leaving the turn: a Raw pair that ramps
    // yaw curvature between its endpoints while pitch curvature rises and
    // falls. What lets a helix climb or descend and still hand its easements
    // LEVEL track -- the coupling note at the helix says why that matters.
    // Bank is held, not ramped: un-banking is the exit clothoid's job, and
    // doing it here would be the exact coupling this helper exists to avoid.
    // Rot authors the pair for a frame already twisted by that much.
    // UX: the runtime editor cannot author this at all -- Raw again, and the
    // first place the design needs yaw AND pitch curvature on one segment.
    void TurningPitch(double DeltaRad, double K, double YawStart, double YawEnd, double Bank,
                      double Rot = 0.0)
    {
        const double Kk = DeltaRad >= 0 ? K : -K;
        const double L = std::fabs(DeltaRad) / K;
        const double YawMid = 0.5 * (YawStart + YawEnd);
        FAuthoredSegment In; In.Kind = ESegmentKind::Raw; In.RawSegment.Length = L;
        Twisted(YawStart, 0.0, Rot, In.RawSegment.YawCurvatureStart, In.RawSegment.PitchCurvatureStart);
        Twisted(YawMid, Kk, Rot, In.RawSegment.YawCurvatureEnd, In.RawSegment.PitchCurvatureEnd);
        In.RollStartDegrees = In.RollEndDegrees = Bank; In.RollMode = ERollMode::WorldBank; Add(In);
        FAuthoredSegment Out; Out.Kind = ESegmentKind::Raw; Out.RawSegment.Length = L;
        Twisted(YawMid, Kk, Rot, Out.RawSegment.YawCurvatureStart, Out.RawSegment.PitchCurvatureStart);
        Twisted(YawEnd, 0.0, Rot, Out.RawSegment.YawCurvatureEnd, Out.RawSegment.PitchCurvatureEnd);
        Out.RollStartDegrees = Out.RollEndDegrees = Bank; Out.RollMode = ERollMode::WorldBank; Add(Out);
    }
    // A clothoid, authored for a twisted frame: same linear curvature ramp,
    // components rotated. Raw because a Clothoid segment has no pitch field.
    void TwistedClothoid(double L, double K0, double K1, double Roll0, double Roll1, double Rot)
    {
        FAuthoredSegment A; A.Kind = ESegmentKind::Raw; A.RawSegment.Length = L;
        Twisted(K0, 0.0, Rot, A.RawSegment.YawCurvatureStart, A.RawSegment.PitchCurvatureStart);
        Twisted(K1, 0.0, Rot, A.RawSegment.YawCurvatureEnd, A.RawSegment.PitchCurvatureEnd);
        A.RollStartDegrees = Roll0; A.RollEndDegrees = Roll1; A.RollMode = ERollMode::WorldBank; Add(A);
    }
    // Clothoid in, arc, clothoid out, roll ramped across the easements.
    // UX: three segments and seven numbers for one banked turn; the radius and
    // the bank are typed three times each.
    void BankedTurn(double R, double ArcDeg, double Ease, double Bank)
    {
        const double Arc = Deg(ArcDeg) * std::fabs(R) - Ease;
        FAuthoredSegment A; A.Kind = ESegmentKind::Clothoid; A.Length = Ease; A.CurvatureStart = 0; A.CurvatureEnd = 1.0 / R;
        A.RollStartDegrees = 0; A.RollEndDegrees = Bank; A.RollMode = ERollMode::WorldBank; Add(A);
        FAuthoredSegment B; B.Kind = ESegmentKind::Arc; B.Length = Arc; B.Radius = R;
        B.RollStartDegrees = Bank; B.RollEndDegrees = Bank; B.RollMode = ERollMode::WorldBank; Add(B);
        FAuthoredSegment C; C.Kind = ESegmentKind::Clothoid; C.Length = Ease; C.CurvatureStart = 1.0 / R; C.CurvatureEnd = 0;
        C.RollStartDegrees = Bank; C.RollEndDegrees = 0; C.RollMode = ERollMode::WorldBank; Add(C);
    }
    // Up, over, down, back to level: a symmetric airtime hill of a given
    // approach angle. Net height zero by construction.
    void Hill(double AngleDeg, double K)
    {
        EasedPitch(Deg(AngleDeg), K);
        EasedPitch(-2.0 * Deg(AngleDeg), K);
        EasedPitch(Deg(AngleDeg), K);
    }
};

// The overrides warm-start the outer loop in main(): the solver's three levers
// and the measured rise trim are fed back in, because the untwist segment is
// authored FROM the helix's turn count and has to be re-authored when the
// solver moves it. Zero means "use the closed-form starting guess".
FDesign Sidewinder(double RiseTrimRad = 0.0, double TurnsOverride = 0.0,
                   double DropOverride = 0.0, double FillOverride = 0.0,
                   double SnakeOverride = 0.0, double SinT2Override = 0.0,
                   double HomeDropOverride = 0.0)
{
    FDesign D;
    // Seven 3 m cars: 21 m. Every holding device is 30 m, which is the train
    // plus the 1 m nose clearance plus margin.
    const double Station = 26.0;   // 21 m train + 1 m nose clearance + margin

    // ---- STATION and the LIFT. Chain at a REALISTIC 3 m/s (2026-08-25):
    // it ran 8 m/s -- 28.8 km/h, a launch wearing a chain's name -- because
    // the crest delivery speed was doing energy work. Real chains crawl at
    // 2-4 m/s, and the user noticed. The energy the fast chain carried now
    // matters less because of the CREST BRAKE below.
    D.Straight(Station, EAuthoredZone::Station, 3.0, 1.5, 1.5);            // 0
    // 8 m/s^2 of grip STAYS: a 35 degree chain needs g*sin(35) = 5.6 just to
    // HOLD, whatever speed it runs -- grip is force, not speed.
    // UX: nothing says a device's grip is under its own gradient; the audit
    // checks whether a brake can stop a train and not whether a lift can lift.
    D.Straight(6.0, EAuthoredZone::Lift, 3.0, 8.0, 8.0);                    // 1  chain engages
    D.EasedPitch(Deg(35.0), 0.05, EAuthoredZone::Lift, 3.0, 8.0);                // 2-3
    D.Straight(56.0, EAuthoredZone::Lift, 3.0, 8.0, 8.0);                   // 4  the climb (56: at 46 the train died 1 m short of the last brake)
    D.EasedPitch(-Deg(35.0), 0.05, EAuthoredZone::Lift, 3.0, 8.0);               // 5-6 over the crest

    // ---- THE CREST BRAKE (2026-08-25), the one place a mid-run hold is
    // HONEST on this ride: everywhere on the course a train released from
    // rest cannot climb turn 2 -- the original crash -- but a pre-drop block
    // brake at the top stops a 3 m/s arrival trivially and releases into the
    // ENTIRE drop, so it can never strand anyone. It is also the buffer the
    // slow chain wants: a train waits here for the course to clear instead of
    // hanging on the chain mid-climb. Real rides carry exactly this device
    // for exactly these reasons. Fifth holding place; the ride still ships
    // three trains, so the flow gains slack rather than traffic.
    //
    // AND ITS TYRES RUN AT 8 m/s -- TRANSPORT TYRES, the user's own words.
    // The energy budget was solved with the crest DELIVERING 8; slow the
    // chain to a realistic 3 and hand the drop 3, and the whole ride runs
    // 55 m2/s2 short -- measured, the through-train stalled at 1146 m on
    // turn 2's climb. A block brake's tyres are a setpoint pushed toward
    // from either side, so the plateau boosts 3 to 8 in under half its
    // length and the drop gets the delivery every margin was measured at.
    D.Straight(26.0, EAuthoredZone::BlockBrake, 8.0, 3.0, 3.0, 4.0);        // 7  the crest plateau

    // ---- THE DROP. 40 degrees; the straight's length is SOLVED for closure.
    // 50 degrees was the first number and its easements ALONE descended 60 m
    // from a 26 m crest -- the height lever had nowhere to go. UX: the editor
    // shows no height at a segment end, so that is found by riding it.
    D.EasedPitch(-Deg(40.0), 0.025);                                        // 8-9
    D.DropIndex = D.Straight(DropOverride > 0.0 ? DropOverride : 15.0);     // 10 height lever
    D.EasedPitch(Deg(40.0), 0.025);                                         // 11-12 pull-out, level (0.025 keeps the bottom near 3.2 g)

    // ---- THE FILL, on the OUTBOUND leg at the bottom of the drop: the plan
    // lever, where the train is fastest and a straight costs least. A hill
    // stood here for most of the design; every metre of it was a metre of
    // dead fill on the return leg, which is where the energy runs out.
    // UX: the plan balance of an oval -- outbound extent equals return
    // extent -- is nowhere in the editor; it is found by the seam not closing.
    D.FillIndex = D.Straight(FillOverride > 0.0 ? FillOverride : 40.0);

    // ---- TURN 1: 180 left at about 22 m/s. Its arc is the closure's ACROSS
    // lever (FreeRadius): turn 2 is a climbing helix now, whose easements do
    // not project onto the plan quite like a level turn's, and a few
    // centimetres of across error have to land somewhere an author said they
    // may. The radius moves by fractions of a metre.
    D.BankedTurn(35.0, 180.0, 40.0, BankDeg(27.0, 35.0));   // 27 m/s is what a 46 m crest delivers here; banked for 22 it read 0.8 g lateral
    D.Turn1ArcIndex = D.Doc.Segments.size() - 2;

    // ---- MID-COURSE TRIM at 25 m/s. It was a BLOCK brake, and that was wrong
    // twice over, measured (2026-08-24): a train arrives at 26.6 m/s and
    // stopping it needs twice the device -- commanded to hold, it overran into
    // the block and REAR-ENDED the train ahead -- and a train RELEASED here
    // from a standing start left at 13 m/s where the course ahead needs 25,
    // so it valleyed on hill 3 every time; the preset STAGED a train here at
    // boot, which is why the first thing anybody saw was the valley. A device
    // that can neither stop what arrives nor deliver a train that completes
    // is a trim wearing a block brake's name. Now it is a trim, which is what
    // its 25 m/s pass-through always was; the HOLD moved to after the helix,
    // where a stopped train has only the rise left to pay for.
    // ITS LENGTH IS A CLOSURE LEVER, and the only one of its kind: this is
    // the single straight on the 180-degree leg, so it is the ride's ONE
    // piece of -X length. The fill and the home stretch both run +X -- the
    // return comes back through two 180s, so its final approach PARALLELS
    // the outbound -- and lengthening either pushes the seam the same way.
    // Measured the expensive way: the crest brake's +22 m of outbound left
    // every +X lever pinned at a floor and the solver honestly 1.7 m short,
    // until this leg was freed to absorb it.
    D.TrimIndex = D.Straight(30.0, EAuthoredZone::Brake, 26.0, 6.0, 4.0);   // 22  pad only; Decel is its rate

    // ---- THE SNAKE: right, left, left, right at 40 m. Banking both ways, and
    // plan-neutral, so the closure never sees it.
    // PLAN-NEUTRAL BY SYMMETRY: R,L then L,R with equal angles and easements
    // puts the line back where it was. (A straight between the halves of one S
    // was tried as a third, across lever, and was the only thing NOT closing:
    // the helix's easements are symmetric too, so nothing needed moving.)
    // The SECOND bend's arc is the closure's ACROSS lever (FreeLength): turn 2
    // is a climbing helix now, whose pitched easement does not project onto
    // the plan quite like a level one, and the few centimetres of across
    // error have to land somewhere an author said they may. A 45 degree
    // bend's length moves the line across as well as along; the solver uses
    // fractions of a metre of it.
    const double Sb = BankDeg(24.0, 45.0);   // banked for the speed it actually gets; at R 35 for 20 m/s the lateral read 0.84 g
    // 45 degree bends with 25 m easements. They were 25 degree bends, which at
    // R 35 is 15 m of arc -- LESS than the two 25 m easements turn (20.5 deg
    // each) -- so the arc came out 9.7 m NEGATIVE and AddSegment silently
    // dropped all four. The snake still closed (symmetric), but the zone walk
    // counted the missing metres and put every later device 39 m early,
    // which is where "the final brake stops driving 20 m in" came from.
    // UX: a negative arc should be a red row, not a vanished segment.
    D.BankedTurn(-45.0, 45.0, 25.0, -Sb);
    D.BankedTurn(45.0, 45.0, 25.0, Sb);
    D.SnakeArcIndex = D.Doc.Segments.size() - 2;
    if (SnakeOverride > 0.0) { D.Doc.Segments[D.SnakeArcIndex].Length = SnakeOverride; }
    D.BankedTurn(45.0, 45.0, 25.0, Sb);
    D.BankedTurn(-45.0, 45.0, 25.0, -Sb);

    // ---- HILLS 2 and 3, smaller. (Three was the brief; the third cost the
    // helix its entry speed and went.)
    D.Hill(16.0, 0.035);
    D.Hill(12.0, 0.04);

    // ---- THE HELIX: one full turn left, DESCENDING 4 degrees, R 18.
    // A LEVEL full-turn helix CROSSES ITSELF AT GRADE. One full plan turn has
    // winding: the exit easement must pass through the entry easement, and it
    // did -- measured at 0.109 m rail centre to rail centre (the rails alone
    // are 1.22 m wide), and photographed from the ride before it was measured.
    // So the descent is not styling: it is the vertical separation at the
    // crossing, about 6 m against the 3 m collision corridor.
    //
    // It was level because the clothoid OUT of a pitched, banked helix turned
    // 13 degrees of pitch on its own -- un-banking while pitched puts the
    // easement's yaw curvature partly into pitch. The answer is to LEVEL OUT
    // WHILE STILL TURNING: TurningPitch holds the yaw curvature while pitch
    // eases away, so both clothoids run exactly as level as they always did.
    // UX: nothing in the editor says a helix should be entered at its climb
    // angle (TrackSpline.h), let alone that leaving one wants a segment kind
    // the runtime editor cannot author.
    const double HR = 20.0;
    const double HClimbDeg = 3.0;              // the descent through the turn -- enough for the
                                               // corridor (~4.5 m at the crossing), cheap on the climb back
    const double HClimb = Deg(HClimbDeg);
    const double HC = std::cos(HClimb);
    const double HKyaw = HC * HC / HR;         // a true helix's curvature at that climb
    const double HEase = 20.0;
    const double HPitchK = 0.035;
    const double HPitchL = HClimb / HPitchK;   // each Raw piece of a TurningPitch pair
    const double Hb = BankDeg(19.0, HR);       // banked for the speed it measures, not the one it wants
    // A DESCENDING HELIX TWISTS THE FRAME, AND THE TWIST IS GEOMETRY. Three
    // constructions were measured before this one:
    //   - LEVEL (as shipped): crosses itself at 0.109 m, because one full plan
    //     turn has winding and both easements sit at the same height.
    //   - Authored level, entered pitched: constant curvature with no torsion
    //     is a TILTED CIRCLE, not a helix -- it came back up (+1.4 m net).
    //   - MakeHelix at -4 with a counter-torsion "untwist" after it: torsion
    //     does not twist the frame, it rotates a segment's own curvature
    //     vector (CurvatureAt). The twist is PARALLEL-TRANSPORT HOLONOMY,
    //     2 pi Turns sin(climb) = 0.33 rad, and no authored field removes
    //     what geometry put in -- the exit easements landed in a frame rolled
    //     19 degrees, over-pitched by exactly that rotation (fitted to three
    //     decimals), and the seam ended 13 degrees down. That is also where
    //     the historical "13 degrees of pitch" came from.
    // So the twisted stretch is authored FOR the twisted frame -- Twisted(),
    // exact for linear ramps -- and the twist itself is PAID BACK where the
    // layout already owes a climb: turn 2 below is a RISING helix whose
    // opposite holonomy cancels this one.
    const double TurnClothoids = 2.0 * (HEase / HR) * 0.5;
    const double TurnPairs = 2.0 * HPitchL * (1.0 / HR + HKyaw);
    const double HelixTurnsGuess = (2.0 * Pi - TurnClothoids - TurnPairs) / (2.0 * Pi);
    const double HelixTurns = TurnsOverride > 0.0 ? TurnsOverride : HelixTurnsGuess;
    D.HelixTurns = HelixTurnsGuess;
    const double Phi = 2.0 * Pi * HelixTurns * std::sin(HClimb);   // the holonomy, exactly
    { FAuthoredSegment A; A.Kind = ESegmentKind::Clothoid; A.Length = HEase; A.CurvatureStart = 0; A.CurvatureEnd = 1.0 / HR;
      A.RollEndDegrees = Hb; A.RollMode = ERollMode::WorldBank; D.Add(A); }
    D.TurningPitch(-HClimb, HPitchK, 1.0 / HR, HKyaw, Hb);   // pitch down, still turning
    { FAuthoredSegment H; H.Kind = ESegmentKind::Helix; H.Radius = HR; H.ClimbAngleDegrees = -HClimbDeg; H.Turns = HelixTurns;
      H.RollStartDegrees = Hb; H.RollEndDegrees = Hb; H.RollMode = ERollMode::WorldBank; D.Add(H);
      D.HelixIndex = D.Doc.Segments.size() - 1; }
    D.TurningPitch(HClimb, HPitchK, HKyaw, 1.0 / HR, Hb, Phi);   // level out, still turning -- twisted frame
    D.TwistedClothoid(HEase, 1.0 / HR, 0.0, Hb, 0.0, Phi);       // and unbank, likewise

    // ---- TURN 2, THE PAYBACK: the 180 CLIMBS now, as a helix of opposite
    // holonomy. Its climb is solved from the helix's own turn count --
    // 0.5 turns of this against HelixTurns of that -- so the two twists
    // cancel exactly and the frame leaves the turn straight: everything from
    // here to the seam is plain authoring again. The climb replaces the old
    // 3 m rise (the return now owes about 12 m, which the solver takes back
    // out of the drop), and a helix on a 35 m cylinder projects onto the plan
    // as the same 35 m circle as turn 1, which is what keeps the across
    // balance. Still taken at the low level: the climb at the END spends the
    // train's last energy, and arriving at the brakes slow is what brakes
    // want.
    const double T2R = 35.0;
    const double T2Ease = 40.0;
    // The guess assumes half a turn of payback; only the torsioned ARC pays,
    // and how much is not worth deriving when it can be MEASURED -- main()
    // reads the residual twist off the level home stretch (WorldBankOf on
    // track whose authored bank is zero) and feeds the climb back in until
    // the residual is arcseconds. Same idiom as the seam trim.
    const double SinT2 = SinT2Override > 0.0 ? SinT2Override
                       : 2.0 * HelixTurns * std::sin(HClimb);
    const double T2Climb = std::asin(SinT2);
    const double T2C = std::cos(T2Climb);
    const double T2K = T2C * T2C / T2R;
    const double T2Bank = BankDeg(17.0, T2R);
    D.SinT2Out = SinT2;
    // The 180 budget, and EVERY resident of the circle pays into it: the two
    // easements AND the level-out pair, which stays at full curvature while
    // pitch eases away. Forgetting the pair made turn 2 turn 193.5 degrees,
    // measured -- the return line ran 13.5 degrees skew and no lever could
    // reach it.
    const double T2PairYaw = (T2Climb / 0.03) * (T2K + 1.0 / T2R);
    const double T2Heading = Pi - T2Ease * T2K - T2PairYaw;
    const double T2L2 = T2Heading / (T2K * T2C);
    D.TurningPitch(T2Climb, 0.03, 0.0, 0.0, 0.0, Phi);         // pitch up on the straight, twisted frame
    D.TwistedClothoid(T2Ease, 0.0, T2K, 0.0, T2Bank, Phi);     // ease into the circle, climbing
    {
        // The climbing arc itself: constant components authored for the frame
        // as it stands, plus the helix's own torsion -- which rotates them at
        // exactly the rate the frame untwists, so the APPLIED field is a true
        // helix from end to end while Phi decays to zero.
        FAuthoredSegment H2; H2.Kind = ESegmentKind::Raw;
        D.T2HeadingOut = T2Heading;
        H2.RawSegment.Length = T2L2;
        FDesign::Twisted(T2K, 0.0, Phi, H2.RawSegment.YawCurvatureStart, H2.RawSegment.PitchCurvatureStart);
        H2.RawSegment.YawCurvatureEnd = H2.RawSegment.YawCurvatureStart;
        H2.RawSegment.PitchCurvatureEnd = H2.RawSegment.PitchCurvatureStart;
        H2.RawSegment.Torsion = Phi / H2.RawSegment.Length;
        H2.RollStartDegrees = H2.RollEndDegrees = T2Bank; H2.RollMode = ERollMode::WorldBank; D.Add(H2);
    }
    D.TurningPitch(-T2Climb, 0.03, T2K, 1.0 / T2R, T2Bank);    // level out, still turning -- frame is straight now
    { FAuthoredSegment C2; C2.Kind = ESegmentKind::Clothoid; C2.Length = T2Ease; C2.CurvatureStart = 1.0 / T2R; C2.CurvatureEnd = 0;
      C2.RollStartDegrees = T2Bank; C2.RollMode = ERollMode::WorldBank; D.Add(C2); }
    const std::size_t TailStart = D.Doc.Segments.size();   // the WorldBank tail begins here -- see the end

    // ---- THE SEAM TRIM, HERE, where the residual is BORN -- the payback
    // chain's imperfections leave the track pitched a few degrees. Trimmed at
    // the very end instead, the whole home stretch rode that slope: the home
    // descent under-delivered, the brakes sat on a 5 degree gradient, the
    // seam ended 11 m high and the solver paid for it by sinking the entire
    // mid-course 19 m. Measured by main()'s outer loop, cancelled here, so
    // everything after this line is authored on genuinely level track.
    if (std::fabs(RiseTrimRad) > 1e-9)
    {
        D.EasedPitch(-RiseTrimRad, std::max(1e-4, std::fabs(RiseTrimRad) / 4.0));
    }

    // ---- THE HOME DESCENT. Turn 2's climb has to end ABOVE the station, or
    // the ride's whole low stretch sits a climb's depth underground and the
    // station stops being "a little above the ground" -- measured: without
    // this the lowest point was 17.9 m down and the train died on the climb.
    // Sized in closed form: the low stretch sits at (HomeDrop - chain climb),
    // and the brief's shipped figure for the station above it is 3.2 m. A
    // FEEDBACK loop was tried here first and fought the closure solver to a
    // 20 m standstill -- the lowest point is a bang-bang function of this
    // number, and bang-bang is exactly what a per-pass correction cannot
    // chase. Closed form is dull and converges.
    {
        const double T2PairLen = 2.0 * (T2Climb / 0.03);
        const double ChainClimb = (T2Ease + T2L2 + T2PairLen) * SinT2;
        const double HomeDrop = HomeDropOverride > 0.0 ? HomeDropOverride
                              : std::max(1.0, ChainClimb - 3.2);
        D.HomeDropOut = HomeDrop;
        const double HomeAngle = Deg(12.0);
        const double PairDrop = 2.0 * (HomeAngle / 0.03) * std::sin(HomeAngle * 0.5);
        const double Run = std::max(5.0, (HomeDrop - PairDrop) / std::sin(HomeAngle));
        D.EasedPitch(-HomeAngle, 0.03);
        // NOT a solver lever, deliberately: it was one for a while, and the
        // solver's plan closure and the height feedback then wrote the same
        // segment and chased each other -- home drop walked 12 to 30 m while
        // the lowest point wandered. The return's along lever is the LEVEL
        // mid-course brake below; this run belongs to the height loop alone.
        D.HomeRunIndex = D.Straight(Run);
        D.EasedPitch(HomeAngle, 0.03);
    }

    // ---- THE MID-COURSE BLOCK BRAKE, on the LEVEL HOME STRETCH (2026-08-24).
    // Where a block brake goes is never "mid-course": it is WHERE A STOPPED
    // TRAIN CAN STILL FINISH, and on this ride that is after the helix and
    // the climb have both been paid for -- a train released from rest here
    // rolls into the final brake on the tyres' own push. 40 m at least, so
    // the pad can stop what arrives with the whole train over the device --
    // and its length is the closure's RETURN lever (FreeLength): dead level,
    // so the solver can trade plan length here without touching a height,
    // and a longer brake is only ever safer. NOTE it runs +X like the fill --
    // the trim leg is the counterweight, not this; a 62 m default was tried
    // here to "balance" the crest and moved the seam the wrong way.
    D.McbrIndex = D.Straight(40.0, EAuthoredZone::BlockBrake, 12.0, 6.0, 6.0, 6.0);

    // ---- THE FINAL BLOCK BRAKE, and home. Pad at 6, measured: at 3 it could
    // not stop what arrives and overran toward a station with a train in it.
    D.Straight(Station, EAuthoredZone::BlockBrake, 3.0, 3.0, 3.0, 6.0);

    // ---- THE WORLDBANK TAIL: everything from the seam trim home is rolled
    // against the HORIZON, not the path frame. The payback chain leaves a
    // residual ~1.4 degrees of frame twist that nothing authored can remove
    // (holonomy is solid-angle: the climb secant warps the ride, torsion on
    // a pitch blip moves it 1.5% -- both measured), and a PathRelative tail
    // WEARS that twist: the rails arrived at the seam rolled, a 4 cm crack
    // in the spine, photographed. Rolled WorldBank, the rails are
    // horizon-true whatever the path frame carries, the seam matches
    // PHYSICALLY, and the residual lives where residuals belong -- in a
    // frame nothing renders. Roll never touches the centreline, so the
    // solved geometry is bit-identical.
    for (std::size_t i = TailStart; i < D.Doc.Segments.size(); ++i)
    {
        D.Doc.Segments[i].RollMode = ERollMode::WorldBank;
    }
    return D;
}

// Zones from contiguous runs of equal kind and speed, as the actor derives them.
struct FZoneSpan { double Start, End; EAuthoredZone Z; double Speed, Accel, Decel, Pad; };

std::vector<FZoneSpan> CollectZones(const FTrackDocument& Doc)
{
    std::vector<FZoneSpan> Out;
    double S = 0.0; std::size_t i = 0;
    while (i < Doc.Segments.size())
    {
        const FAuthoredSegment& A = Doc.Segments[i];
        const double Start = S; const EAuthoredZone Z = A.Zone; const double Sp = A.ZoneSpeed;
        const double Accel = A.ZoneAccel, Decel = A.ZoneDecel, Pad = A.ZoneBrakeDecel;
        while (i < Doc.Segments.size() && Doc.Segments[i].Zone == Z && Doc.Segments[i].ZoneSpeed == Sp)
        { S += BuildSegment(Doc.Segments[i]).Length; ++i; }
        if (Z != EAuthoredZone::None) { Out.push_back({Start, S, Z, Sp, Accel, Decel, Pad}); }
    }
    return Out;
}

const char* ZoneName(EAuthoredZone Z)
{
    return Z == EAuthoredZone::Station ? "STATION" : Z == EAuthoredZone::Lift ? "LIFT"
         : Z == EAuthoredZone::BlockBrake ? "BLOCK BRAKE" : "BRAKE";
}

void AddZones(FTrain& Train, const std::vector<FZoneSpan>& Zones)
{
    for (const FZoneSpan& Z : Zones)
    {
        if (Z.Z == EAuthoredZone::Brake) { Train.AddZone(MakeBrake(Z.Start, Z.End, Z.Speed, Z.Decel)); continue; }
        FTrackZone T{Z.Start, Z.End, Z.Speed, Z.Accel, Z.Decel};
        if (Z.Pad > 0.0) { T.BrakeLimit = Z.Speed; T.BrakeDecel = Z.Pad; }
        Train.AddZone(T);
    }
}
} // namespace

int main()
{
    // ---- AUTHOR, CLOSE, MEASURE THE SEAM PITCH, RE-AUTHOR. An outer loop,
    // because two authored numbers depend on what the solve itself moves: the
    // untwist segment is authored FROM the helix's turn count, and the rise
    // trim FROM the seam's residual pitch. The coupling is weak (dPhi is
    // dTurns scaled by sin 4 degrees), so this converges in a few passes;
    // each pass warm-starts the solver with the last pass's solved numbers.
    FDesign D;
    FClosureResult R;
    double Trim = 0.0, Turns = 0.0, SinT2 = 0.0, HomeDrop = 0.0;
    double Drop = 0.0, Fill = 0.0, Snake = 0.0, Mcbr = 0.0, TrimL = 0.0;
    for (int Outer = 0; Outer < 14; ++Outer)
    {
        // The parameters the AUTHORING depends on warm-start (Turns, the trim,
        // the payback climb, the home drop); the pure lengths deliberately do
        // NOT -- Gauss-Newton restarted from a half-solved state after new
        // segments appeared walked the fill into its lower bound and could
        // not get out, where a cold start converges every pass.
        //
        // AND THE COLD START IS MULTI-START ON THE FILL. Gauss-Newton is a
        // local method and this layout keeps finding its corners: three
        // ordinary edits in a row have left one starting point in a basin
        // where the solver trades heading for position, pins the fill at a
        // bound, and declares no step improves anything. The fill is the
        // lever it happens to, so the fill gets the spread of starts; the
        // first converged one wins.
        static const double FillTries[] = {0.0, 17.0, 8.0, 70.0, 130.0};
        for (const double FillTry : FillTries)
        {
            D = Sidewinder(Trim, Turns, 0.0, FillTry, 0.0, SinT2, HomeDrop);
            FTrack T0 = BuildTrack(D.Doc);
            // EVERY AUTHORED SEGMENT MUST BUILD. A negative arc (easements
            // longer than the turn) is refused by AddSegment without a word,
            // and the zone walk below would then disagree with the track
            // about where things are.
            if (T0.NumSegments() != D.Doc.Segments.size())
            {
                for (std::size_t i = 0; i < D.Doc.Segments.size(); ++i)
                {
                    const double L = BuildSegment(D.Doc.Segments[i]).Length;
                    if (!(L > 0.0)) { std::printf("segment %zu has length %g and was REFUSED\n", i, L); }
                }
                return 3;
            }

            // Position in all three axes and heading; roll closes by
            // construction because every bank returns to zero.
            FClosureTarget Target = CircuitTarget(T0);
            Target.bMatchHeading = true;
            FClosureOptions Opt; Opt.MaxIterations = 120;
            // TIGHTER THAN THE ACTOR'S 1 mm SEAM BAR, deliberately: the
            // layout stores these numbers as FLOATS, and a solve that stops
            // at 0.997 mm arrives in the engine just over 1 mm and the
            // circuit refuses to wrap. Measured: the first transcription
            // shipped exactly that.
            Opt.PositionTolerance = 2e-5;
            Opt.bApplyOnFailure = true;   // keep partial progress for the measurements below
            R = SolveClosure(D.Doc, Target,
                {FreeLength(D.DropIndex, 1.0, 200.0), FreeLength(D.FillIndex, 1.0, 400.0),
                 FreeField(D.HelixIndex, EClosureField::Turns, 0.5, 1.2),
                 FreeLength(D.SnakeArcIndex, 2.0, 60.0),
                 FreeLength(D.TrimIndex, 25.0, 150.0),
                 FreeLength(D.McbrIndex, 40.0, 90.0)}, Opt);
            if (R.bConverged) { break; }
        }
        Turns = D.Doc.Segments[D.HelixIndex].Turns;
        Drop = D.Doc.Segments[D.DropIndex].Length;
        Fill = D.Doc.Segments[D.FillIndex].Length;
        Snake = D.Doc.Segments[D.SnakeArcIndex].Length;
        TrimL = D.Doc.Segments[D.TrimIndex].Length;
        Mcbr = D.Doc.Segments[D.McbrIndex].Length;

        const FTrack Tc = BuildTrack(D.Doc);
        const FTrackFrame E = Tc.EvaluateAt(Tc.TotalLength());
        const double PitchEnd = std::asin(E.Tangent.Z);

        // The residual twist, read where it has nowhere to hide: the final
        // brake is authored dead level, so any bank the frame carries there
        // is holonomy turn 2 failed to pay back.
        const double TwistRes = WorldBankOf(Tc.EvaluateAt(Tc.TotalLength() - 10.0));

        // The lowest point, for the home drop: the brief wants the station "a
        // little above the ground", and 3.2 m is what the shipped ride had.
        double Lowest = 0.0;
        {
            FTrackFrame W = Tc.EvaluateAt(0.0);
            for (double S = 0.0; S < Tc.TotalLength(); )
            {
                const double N = std::min(S + 2.0, Tc.TotalLength());
                W = Tc.AdvanceFrom(W, S, N); S = N;
                Lowest = std::min(Lowest, W.Position.Z);
            }
        }

        const FTrackFrame A0 = Tc.EvaluateAt(0.0);
        const double SeamRollPhys = std::atan2(
            Dot(Cross(A0.Up, E.Up), A0.Tangent), Dot(A0.Up, E.Up));

        std::printf("  pass %2d: %s gap %.6f m, heading %.4f deg, seam pitch %+.5f deg, "
                    "twist %+.4f deg, SEAM ROLL %+.5f deg, lowest %+.2f m (trim %+.4f, home %.2f)\n",
            Outer, R.bConverged ? "closed," : "OPEN,", R.After.ActiveError,
            R.After.HeadingError * 180.0 / Pi, PitchEnd * 180.0 / Pi,
            TwistRes * 180.0 / Pi, SeamRollPhys * 180.0 / Pi, Lowest, Trim * 180.0 / Pi,
            HomeDrop > 0.0 ? HomeDrop : 12.0);
        // The PHYSICAL seam roll -- the same Up-vector measure the actor's
        // circuit check uses, because the design must gate on what the rails
        // actually do, not on a path-frame number. The residual frame twist
        // is GAUGE now (the WorldBank tail wears the horizon, not the
        // frame), so this reads ~0 however much holonomy the payback chain
        // leaves behind; TwistRes stays printed as the size of what the
        // gauge is hiding. Both cancellation routes were measured and
        // rejected: the climb secant warped the ride (trim to 9 degrees,
        // turn 2 flattened), and torsion on the home pitch pair moved the
        // twist 1.5% of commanded -- holonomy is solid-angle, and a pitch
        // blip sweeps almost none.
        if (R.bConverged && std::fabs(PitchEnd) < 2e-6
            && std::fabs(SeamRollPhys) < 1.2e-4
            && std::fabs(Lowest + 3.2) < 0.3) { break; }
        Trim += PitchEnd;
        // Not on the first pass: with no trim yet the tail rides the full
        // residual pitch and the measurements below are garbage.
        if (Outer > 0)
        {
            HomeDrop = std::max(1.0, D.HomeDropOut + 0.7 * (-3.2 - Lowest));
        }
        (void)SinT2;
    }
    std::printf("closure: %s -- gap %.6f m, heading %.6f deg\n",
        R.bConverged ? "CONVERGED" : "FAILED", R.After.ActiveError, R.After.HeadingError * 180.0 / Pi);
    std::printf("  drop straight = %.7f m   fill straight = %.7f m   helix turns = %.7f   snake arc = %.7f m   trim = %.7f m   mcbr = %.7f m\n",
        Drop, Fill, Turns, Snake, TrimL, Mcbr);
    if (!R.bConverged) { std::printf("%s\n", R.Message.c_str()); return 1; }

    // ---- RIDE IT. Seven cars.
    const FTrack T = BuildTrack(D.Doc);
    const std::vector<FZoneSpan> Zones = CollectZones(D.Doc);
    for (const FZoneSpan& Z : Zones)
    {
        std::printf("  zone %-11s %7.1f - %7.1f m  %.1f m/s\n", ZoneName(Z.Z), Z.Start, Z.End, Z.Speed);
    }
    FTrainConfig C; C.TrainLength = 21.0;
    FTrain Train(T, C);
    AddZones(Train, Zones);
    const FRideProfile P = RunRideProfile(Train, T, 0.5);
    std::printf("track: %zu segments, %.1f m\n", D.Doc.Segments.size(), T.TotalLength());
    std::printf("ride:  %s, %.1f s, top %.1f m/s (%.1f km/h) at %.0f m\n",
        P.bCompleted ? "completed" : "STALLED", P.Duration, P.TopSpeed, P.TopSpeed * 3.6, P.TopSpeedS);
    std::printf("       vertical %.2f..%.2f g, lateral %.2f g, roll rate %.1f deg/s\n",
        P.MinVerticalG, P.MaxVerticalG, P.MaxAbsLateralG, P.MaxAbsRollRate);
    std::printf("       (min vertical at %.0f m, max at %.0f m, lateral at %.0f m, roll rate at %.0f m)\n",
        P.MinVerticalGS, P.MaxVerticalGS, P.MaxAbsLateralGS, P.MaxAbsRollRateS);
    std::printf("       height %.1f..%.1f m (station at 0: %.1f m above the lowest point)\n",
        P.LowestHeight, P.HighestHeight, -P.LowestHeight);
    if (!P.bCompleted) { std::printf("       stalled at %.1f m, %.1f m up\n", P.StalledAtS, P.StalledHeight); }

    int Failures = P.bCompleted ? 0 : 1;

    // ---- EVERY AUTHORED SEGMENT WAS BUILT, and the validator's errors count.
    // A negative arc -- a turn whose easements ate more than the turn -- is
    // refused by FTrack::AddSegment and silently dropped by BuildTrack, so the
    // circuit still closed by symmetry and every later device sat 39 m early
    // (Sidewinder UX list, item 3). This program never ran the validator; the
    // editor does, and its BadLength row now says the consequence. Here it is a
    // failed design, with the sentence.
    for (const FTrackDiagnostic& Dg : ValidateTrack(BuildSegments(D.Doc)))
    {
        if (!Dg.bIsError) { continue; }
        std::printf("validator: segment %zu: %s\n", Dg.SegmentIndex, Dg.Message.c_str());
        ++Failures;
    }
    if (T.NumSegments() != D.Doc.Segments.size())
    {
        std::printf("BUILT %zu of %zu authored segments -- something was dropped\n",
            T.NumSegments(), D.Doc.Segments.size());
        ++Failures;
    }

    // ---- THE COLLISION CORRIDOR, measured the short way round. The level
    // helix shipped crossing itself at 0.109 m because nothing measured this;
    // now the design cannot print its figures without passing its own corridor.
    const FTrackProfile Cross;
    const FClearanceReport Clear = AnalyseSelfClearance(T, Cross, 0.5, 12.0, true);
    std::printf("clearance: closest approach %.2f m at %.0f m vs %.0f m (corridor %.1f m)%s\n",
        Clear.ClosestApproach, Clear.AtS, Clear.AndS, CollisionCorridorM,
        Clear.ClosestApproach < CollisionCorridorM ? "  INSIDE THE CORRIDOR" : "");
    if (Clear.ClosestApproach < CollisionCorridorM) { ++Failures; }

    // ---- EVERY HOLDING DEVICE, asked the two questions the 2026-08-24 crash
    // asked. CAN IT STOP WHAT ARRIVES: the zone is commanded to hold and the
    // ride run at it; the nose must come to rest inside the device, because a
    // hold that overruns parks a train in the next block, which is the
    // rear-end. CAN IT RELEASE A TRAIN THAT COMPLETES: from a standing start
    // at its middle -- where the preset stages trains at boot -- the train
    // must reach the next holding device. A place that fails either is not a
    // place a train may be held, whatever its zone calls itself.
    std::vector<std::size_t> Holds;
    for (std::size_t z = 0; z < Zones.size(); ++z)
    {
        if (Zones[z].Z != EAuthoredZone::Brake) { Holds.push_back(z); }
    }
    const double Dt = 1.0 / 240.0;
    for (std::size_t hi = 0; hi < Holds.size(); ++hi)
    {
        const std::size_t h = Holds[hi];
        // Stop what arrives. The station is where the run starts, so it is
        // asked only the release question.
        if (Zones[h].Start > 1.0)
        {
            FTrain Tr(T, C);
            AddZones(Tr, Zones);
            Tr.SetZoneTargetSpeed(h, 0.0);
            if (Zones[h].Pad > 0.0) { Tr.SetZoneBrakeLimit(h, 0.0); }
            Tr.Place(0.0, 0.0);
            int Still = 0;
            for (int i = 0; i < 240 * 600 && Still <= 480; ++i)
            {
                Tr.Step(Dt);
                Still = Tr.GetSpeed() < 0.02 ? Still + 1 : 0;
            }
            const double Overrun = Tr.GetFrontS() - Zones[h].End;
            std::printf("hold %-11s %6.1f-%6.1f  commanded to stop: nose rests %5.2f m %s the device end%s\n",
                ZoneName(Zones[h].Z), Zones[h].Start, Zones[h].End, std::fabs(Overrun),
                Overrun > 0.0 ? "PAST" : "inside",
                Overrun > 0.0 ? "  CANNOT STOP WHAT ARRIVES" : "");
            if (Overrun > 0.0) { ++Failures; }
        }
        // Release a train that completes, to the NEXT holding device (for the
        // last one, home across the seam).
        {
            FTrain Tr(T, C);
            AddZones(Tr, Zones);
            Tr.Place(0.5 * (Zones[h].Start + Zones[h].End), 0.0);
            const double Goal = hi + 1 < Holds.size() ? Zones[Holds[hi + 1]].Start
                                                      : T.TotalLength() - 0.5;
            int Still = 0; bool bMade = false; double ValleyAt = 0.0;
            for (int i = 0; i < 240 * 600; ++i)
            {
                Tr.Step(Dt);
                if (Tr.GetDistance() >= Goal) { bMade = true; break; }
                if (Tr.GetSpeed() < 0.02) { if (++Still > 480) { ValleyAt = Tr.GetDistance(); break; } }
                else { Still = 0; }
            }
            if (bMade)
            {
                std::printf("hold %-11s %6.1f-%6.1f  released from rest: completes to the next device\n",
                    ZoneName(Zones[h].Z), Zones[h].Start, Zones[h].End);
            }
            else
            {
                std::printf("hold %-11s %6.1f-%6.1f  released from rest: VALLEYS at %.1f m\n",
                    ZoneName(Zones[h].Z), Zones[h].Start, Zones[h].End, ValleyAt);
                ++Failures;
            }
        }
    }
    std::printf("holding places: %zu (expect 5: station, lift, crest brake, post-helix brake, final brake)\n",
        Holds.size());

    // ---- The numbers to transcribe into SidewinderLayout().
    std::printf("\nTRANSCRIBE:\n  const double DropLen = %.7f;\n  const double FillLen = %.7f;\n"
        "  const double HelixTurns = %.7f;   // solved (closed-form guess was %.7f)\n"
        "  const double SnakeArc2 = %.7f;   // the second bend's arc, the across lever\n"
        "  const double TrimLen = %.7f;   // the -X leg, the return's true along lever\n"
        "  const double McbrLen = %.7f;   // the home-stretch brake\n"
        "  const double HomeDropM = %.7f;\n"
        "  const double SeamTrimRad = %.10f;\n",
        D.Doc.Segments[D.DropIndex].Length, D.Doc.Segments[D.FillIndex].Length,
        D.Doc.Segments[D.HelixIndex].Turns, D.HelixTurns, Snake, TrimL,
        D.Doc.Segments[D.McbrIndex].Length, D.HomeDropOut, Trim);
    if (Failures > 0) { std::printf("\nDESIGN CHECKS FAILED: %d\n", Failures); }
    return Failures == 0 ? 0 : 2;
}
