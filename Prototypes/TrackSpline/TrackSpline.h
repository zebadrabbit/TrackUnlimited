// TrackUnlimited Phase 0 prototype: curvature-continuous track spline math.
// Plain C++, no engine dependencies — designed to port into UE5 C++ later.
// Design reference: Docs/PROJECT_PLAN.md Section 5, "Track representation".
//
// The track is an ordered list of segments, each defined as a *curvature
// profile over arc length* rather than as control points. Geometry comes from
// integrating a moving frame along the curve.
//
// This is why: C2 continuity becomes a property of the data instead of
// something fitted after the fact. If curvature is continuous across a joint,
// the geometry is — no solving, no post-hoc smoothing. A clothoid (curvature
// linear in s) is exactly what makes a continuous joint between a straight
// (k = 0) and an arc (k = 1/R) possible, which is why real track alignment is
// designed this way and why NL2's track feels smooth where a naive
// position-only spline does not.
//
// Roll (banking) is carried on the same curvilinear parameter but integrated
// separately: banking rotates the rails *around* the heartline and must not
// perturb the path. The integrated curve IS the heartline; rail centreline is
// derived from it (see FTrack::RailCentreAt).
//
// Units: metres, radians, seconds. UE uses centimetres — convert at the port
// boundary, not in here.
// Frame: right-handed, Tangent x Lateral = Up. +Lateral is the rider's left.

#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

// ---------------------------------------------------------------- vector math

struct FVec3
{
    double X = 0.0, Y = 0.0, Z = 0.0;
};

inline FVec3 operator+(const FVec3& A, const FVec3& B) { return {A.X + B.X, A.Y + B.Y, A.Z + B.Z}; }
inline FVec3 operator-(const FVec3& A, const FVec3& B) { return {A.X - B.X, A.Y - B.Y, A.Z - B.Z}; }
inline FVec3 operator*(const FVec3& A, double S) { return {A.X * S, A.Y * S, A.Z * S}; }
inline double Dot(const FVec3& A, const FVec3& B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
inline FVec3 Cross(const FVec3& A, const FVec3& B)
{
    return {A.Y * B.Z - A.Z * B.Y, A.Z * B.X - A.X * B.Z, A.X * B.Y - A.Y * B.X};
}
inline double Length(const FVec3& A) { return std::sqrt(Dot(A, A)); }

// Rotate V about a unit axis by Angle radians (Rodrigues).
inline FVec3 RotateAbout(const FVec3& V, const FVec3& UnitAxis, double Angle)
{
    const double C = std::cos(Angle);
    const double S = std::sin(Angle);
    return V * C + Cross(UnitAxis, V) * S + UnitAxis * (Dot(UnitAxis, V) * (1.0 - C));
}

// ------------------------------------------------------------------- segments

// One parametric segment. Curvature varies linearly over the segment's arc
// length, which covers the whole authored vocabulary:
//   straight  — yaw = pitch = 0
//   arc       — yaw constant (= 1/R)
//   clothoid  — yaw linear from one curvature to another
// Roll likewise varies linearly across the segment.
//
// ponytail: linear curvature ramps only. That is the standard transition and
// covers every segment type in the Phase 1 editor vocabulary; add a cubic
// (or arbitrary) profile when an authoring case actually needs one.
struct FTrackSegment
{
    double Length = 0.0;

    // 1/m. +ve yaw turns left; +ve pitch raises the nose.
    double YawCurvatureStart = 0.0;
    double YawCurvatureEnd = 0.0;
    double PitchCurvatureStart = 0.0;
    double PitchCurvatureEnd = 0.0;

    // Radians. +ve banks *into* a +ve (left) turn: the rider's up-vector tilts
    // toward the centre of the turn, the way a motorcycle leans.
    double RollStart = 0.0;
    double RollEnd = 0.0;
};

inline FTrackSegment MakeStraight(double InLength, double Roll = 0.0)
{
    FTrackSegment Seg;
    Seg.Length = InLength;
    Seg.RollStart = Seg.RollEnd = Roll;
    return Seg;
}

// Radius +ve = left turn, -ve = right turn.
inline FTrackSegment MakeArc(double InLength, double Radius, double Roll = 0.0)
{
    FTrackSegment Seg = MakeStraight(InLength, Roll);
    Seg.YawCurvatureStart = Seg.YawCurvatureEnd = 1.0 / Radius;
    return Seg;
}

// The transition curve. Curvature ramps linearly, so it joins a straight to an
// arc (or two arcs) with no curvature step — the whole point of the exercise.
inline FTrackSegment MakeClothoid(double InLength, double CurvatureStart, double CurvatureEnd,
                                  double RollStart = 0.0, double RollEnd = 0.0)
{
    FTrackSegment Seg;
    Seg.Length = InLength;
    Seg.YawCurvatureStart = CurvatureStart;
    Seg.YawCurvatureEnd = CurvatureEnd;
    Seg.RollStart = RollStart;
    Seg.RollEnd = RollEnd;
    return Seg;
}

// ---------------------------------------------------------------------- frame

// Two bases, and the difference between them is the whole heartline idea:
// the path frame is what the track *does*, the rider frame is what the rider
// *feels*. Banking rotates the second around the first.
struct FTrackFrame
{
    FVec3 Position;    // heartline position
    FVec3 Tangent;     // shared by both bases — roll is about this axis
    FVec3 Lateral;     // rider's left, banked
    FVec3 Up;          // rider's up, banked
    FVec3 PathLateral; // unbanked left — the direction the path actually bends
    FVec3 PathUp;      // unbanked up
    double Roll = 0.0;
    double YawCurvature = 0.0;
    double PitchCurvature = 0.0;
};

// -------------------------------------------------------------------- gravity

constexpr double GravityMs2 = 9.80665;

// Felt acceleration in G, as the rider experiences it, at a given frame and
// speed. Tangential (accelerating/braking) force is the physics prototype's
// job; this is the geometric part — the bit banking has to get right.
// Signs follow one rule on both axes: the value is the felt vector resolved
// onto a rider axis, and the rider is pressed the *opposite* way. Vertical +1
// is sitting on level track, pressed down into the seat; Lateral +1 likewise
// presses the rider toward their right.
struct FGForces
{
    double Lateral = 0.0;
    double Vertical = 0.0;
};

inline FGForces FeltG(const FTrackFrame& F, double SpeedMs)
{
    const double V2 = SpeedMs * SpeedMs;
    // The path bends along the *unbanked* axes — banking rotates the rider, not
    // the trajectory. Resolving onto the banked axes instead is the classic way
    // to get a banked turn that never cancels its own lateral G.
    const FVec3 PathAccel = F.PathLateral * (V2 * F.YawCurvature) + F.PathUp * (V2 * F.PitchCurvature);
    const FVec3 Felt = PathAccel + FVec3{0.0, 0.0, GravityMs2};
    return {Dot(Felt, F.Lateral) / GravityMs2, Dot(Felt, F.Up) / GravityMs2};
}

// ---------------------------------------------------------------------- track

class FTrack
{
public:
    // Height of the heartline above the rail centreline. ~1.1 m puts it at
    // roughly rider chest height for a sit-down train.
    explicit FTrack(double InHeartlineHeight = 1.1)
        : HeartlineHeight(InHeartlineHeight)
    {
    }

    // Rejects degenerate segments rather than storing them; returns false if the
    // segment was refused. This is the one shared entry point, so guarding here
    // covers every way a bad segment could get in. A zero-length segment whose
    // endpoint curvatures happen to match its neighbours would otherwise satisfy
    // IsCurvatureContinuous at both joints while bridging a real discontinuity,
    // and a negative length silently desynchronises S from arc length. The
    // comparison form also rejects NaN.
    bool AddSegment(const FTrackSegment& Seg)
    {
        if (!(Seg.Length > 0.0))
        {
            return false;
        }
        Segments.push_back(Seg);
        return true;
    }

    std::size_t NumSegments() const { return Segments.size(); }

    double TotalLength() const
    {
        double Sum = 0.0;
        for (const FTrackSegment& Seg : Segments)
        {
            Sum += Seg.Length;
        }
        return Sum;
    }

    // Evaluate the heartline frame at arc length S (clamped to the track).
    //
    // At an exact joint the ENDING segment supplies roll and curvature. That
    // only matters on curvature-discontinuous data, and it is not reliable in
    // floating point anyway — with authored lengths that do not sum exactly,
    // roughly a quarter of joint queries land on the following segment instead.
    // Do not depend on which side you get; sample either side of a joint.
    //
    // Use the returned Tangent for direction; do NOT finite-difference Position
    // to recover it. Steps = ceil(Span/MaxStep) re-discretises per call, so
    // position is not smooth in S below ~1e-9 m and a difference quotient stops
    // converging under h ~ 1e-4. The Tangent is exact to 1e-15 rad — free and
    // better.
    //
    // ponytail: re-integrates from the start on every call — O(track length),
    // about 0.2 ms on a 1 km track. Fine for tests and offline analysis; build a
    // cached arc-length sample table when the editor needs interactive scrubbing
    // or the physics tick needs this every frame.
    FTrackFrame EvaluateAt(double S) const
    {
        const double Total = TotalLength();
        S = S < 0.0 ? 0.0 : (S > Total ? Total : S);

        // Unrolled path frame: banking must not perturb the path, so roll is
        // applied only at the end, as a rotation about the tangent.
        FVec3 P{0.0, 0.0, 0.0};
        FVec3 T{1.0, 0.0, 0.0};
        FVec3 L{0.0, 1.0, 0.0};
        FVec3 U{0.0, 0.0, 1.0};

        double Remaining = S;
        std::size_t Index = 0;
        double Local = 0.0;
        for (std::size_t i = 0; i < Segments.size(); ++i)
        {
            Index = i;
            const double Span = Remaining < Segments[i].Length ? Remaining : Segments[i].Length;
            Integrate(Segments[i], 0.0, Span, P, T, L, U);
            Local = Span;
            Remaining -= Span;
            if (Remaining <= 0.0)
            {
                break;
            }
        }

        return Finish(Index, Local, P, T, L, U);
    }

    // Continue an already-evaluated frame forward to ToS, instead of starting
    // over at the track beginning. Cost is O(ToS - FromS), not O(ToS).
    //
    // This is the fast path a ticking train wants, and the reason it exists is
    // measured: a 425 m layout is ~42,500 integrator steps per EvaluateAt, so
    // one call per frame at 60 Hz does not fit in a frame. Advancing 0.33 m —
    // one tick at 20 m/s — is about 33 steps instead.
    //
    // `From` must be the frame this track produced at FromS; its PathLateral
    // and PathUp carry the unrolled basis the integrator needs. The result is
    // not bit-identical to EvaluateAt(ToS), because the steps land in different
    // places, but both are the same second-order integration of the same curve
    // and the difference is far below the 0.027 mm/km accumulation either way.
    FTrackFrame AdvanceFrom(const FTrackFrame& From, double FromS, double ToS) const
    {
        const double Total = TotalLength();
        const double Target = ToS < 0.0 ? 0.0 : (ToS > Total ? Total : ToS);
        if (Target == FromS)
        {
            // Already there — `From` IS the answer. Worth special-casing: a
            // stalled train asks this every tick, and falling through to a full
            // evaluation would make standing still the most expensive thing the
            // simulation does.
            return From;
        }
        if (Target < FromS || Segments.empty())
        {
            return EvaluateAt(Target);
        }

        FVec3 P = From.Position;
        FVec3 T = From.Tangent;
        FVec3 L = From.PathLateral;
        FVec3 U = From.PathUp;

        std::size_t Index = 0;
        double Local = 0.0;
        Locate(FromS, Index, Local);

        double Remaining = Target - FromS;
        while (Remaining > 0.0 && Index < Segments.size())
        {
            const FTrackSegment& Seg = Segments[Index];
            const double Span = std::min(Remaining, Seg.Length - Local);
            if (Span > 0.0)
            {
                Integrate(Seg, Local, Local + Span, P, T, L, U);
                Local += Span;
                Remaining -= Span;
            }
            if (Remaining <= 0.0)
            {
                break;
            }
            ++Index;
            Local = 0.0;
        }
        if (Index >= Segments.size())
        {
            Index = Segments.size() - 1;
            Local = Segments[Index].Length;
        }

        return Finish(Index, Local, P, T, L, U);
    }

    // The rails hang below the heartline; banking swings them around it, which
    // is the entire reason the heartline model exists.
    FVec3 RailCentreAt(double S) const
    {
        const FTrackFrame F = EvaluateAt(S);
        return F.Position - F.Up * HeartlineHeight;
    }

    // Are all segment joints free of curvature (and roll) steps? A step in
    // curvature is a step in lateral G — the jolt clothoids exist to remove.
    //
    // ponytail: checks curvature and roll value, not roll *rate*. A roll-rate
    // step is a jerk rather than a jolt — worth flagging once the physics
    // prototype can measure whether it's felt.
    bool IsCurvatureContinuous(double Tolerance = 1e-9) const
    {
        for (std::size_t i = 0; i + 1 < Segments.size(); ++i)
        {
            const FTrackSegment& A = Segments[i];
            const FTrackSegment& B = Segments[i + 1];
            if (std::fabs(A.YawCurvatureEnd - B.YawCurvatureStart) > Tolerance ||
                std::fabs(A.PitchCurvatureEnd - B.PitchCurvatureStart) > Tolerance ||
                std::fabs(A.RollEnd - B.RollStart) > Tolerance)
            {
                return false;
            }
        }
        return true;
    }

private:
    static double Lerp(double A, double B, double T) { return A + (B - A) * T; }

    // Which segment owns arc length S, and how far into it. At an exact joint
    // the ENDING segment wins, matching EvaluateAt.
    void Locate(double S, std::size_t& OutIndex, double& OutLocal) const
    {
        double Acc = 0.0;
        for (std::size_t i = 0; i < Segments.size(); ++i)
        {
            if (S <= Acc + Segments[i].Length || i + 1 == Segments.size())
            {
                OutIndex = i;
                OutLocal = S - Acc;
                return;
            }
            Acc += Segments[i].Length;
        }
        OutIndex = 0;
        OutLocal = 0.0;
    }

    // Turn an integrated path frame into the rider frame. Shared by EvaluateAt
    // and AdvanceFrom so the roll convention can only ever be defined once.
    FTrackFrame Finish(std::size_t Index, double Local,
                  const FVec3& P, const FVec3& T, const FVec3& L, const FVec3& U) const
    {
        FTrackFrame Out;
        Out.Position = P;
        Out.Tangent = T;
        if (!Segments.empty())
        {
            const FTrackSegment& Seg = Segments[Index];
            const double A = Seg.Length > 0.0 ? Local / Seg.Length : 0.0;
            Out.Roll = Lerp(Seg.RollStart, Seg.RollEnd, A);
            Out.YawCurvature = Lerp(Seg.YawCurvatureStart, Seg.YawCurvatureEnd, A);
            Out.PitchCurvature = Lerp(Seg.PitchCurvatureStart, Seg.PitchCurvatureEnd, A);
        }
        Out.PathLateral = L;
        Out.PathUp = U;
        // Negated: a right-hand rotation about the tangent drops the rider's
        // right side, so the sign is flipped to make +ve roll bank *into* a
        // +ve (left) turn, which is what an author means by "bank left".
        Out.Lateral = RotateAbout(L, T, -Out.Roll);
        Out.Up = RotateAbout(U, T, -Out.Roll);
        return Out;
    }

    // Exponential-midpoint integration: half-rotate the frame, translate along
    // the tangent, half-rotate again. Symmetric, second order, and keeps the
    // frame orthonormal by construction (it only ever applies rotations),
    // unlike a componentwise RK4 which drifts and needs re-orthonormalising.
    // Integrates the sub-range [From, To] of a segment's own arc length, so
    // that a partial advance samples the curvature ramp at the right place.
    static void Integrate(const FTrackSegment& Seg, double From, double To,
                          FVec3& P, FVec3& T, FVec3& L, FVec3& U)
    {
        const double Span = To - From;
        if (Span <= 0.0)
        {
            return;
        }
        const double MaxStep = 0.01; // metres
        int Steps = static_cast<int>(std::ceil(Span / MaxStep));
        if (Steps < 1)
        {
            Steps = 1;
        }
        const double Ds = Span / Steps;

        for (int i = 0; i < Steps; ++i)
        {
            const double Mid = From + (i + 0.5) * Ds;
            const double A = Seg.Length > 0.0 ? Mid / Seg.Length : 0.0;
            const double Yaw = Lerp(Seg.YawCurvatureStart, Seg.YawCurvatureEnd, A);
            const double Pitch = Lerp(Seg.PitchCurvatureStart, Seg.PitchCurvatureEnd, A);

            // Darboux vector, no tangential (roll) component: roll is applied
            // at evaluation time so it cannot bend the path.
            const FVec3 Omega = U * Yaw - L * Pitch;
            const double Rate = Length(Omega);

            if (Rate > 0.0)
            {
                const FVec3 Axis = Omega * (1.0 / Rate);
                const double Half = Rate * Ds * 0.5;
                T = RotateAbout(T, Axis, Half);
                L = RotateAbout(L, Axis, Half);
                U = RotateAbout(U, Axis, Half);
                P = P + T * Ds;
                T = RotateAbout(T, Axis, Half);
                L = RotateAbout(L, Axis, Half);
                U = RotateAbout(U, Axis, Half);
            }
            else
            {
                P = P + T * Ds;
            }
        }
    }

    // ponytail: a helix is a REPRESENTATION gap, not a missing parameter — do
    // not reach for it before Phase 1 without reading this.
    // "Constant radius + constant climb" is not constant pitch curvature: it is
    // zero pitch curvature with a fixed pitch angle, yawing about the *world*
    // vertical rather than the body up-axis. Authored the obvious way it yields
    // a tilted planar circle that returns to its starting height after a full
    // turn (measured out-of-plane deviation 1.9e-12 m over 300 m — it really is
    // a flat circle, not a bad helix). A true helix comes out of this integrator
    // only from yaw and pitch of constant magnitude rotating at the torsion rate
    // (Yaw = k*cos(tau*s), Pitch = k*sin(tau*s)), which a linear curvature ramp
    // cannot express; approximating it costs roughly one segment per metre.
    // Needs a non-linear profile shape, or a world-vertical yaw term in
    // Integrate. Not "just the authoring parameterisation".

    double HeartlineHeight;
    std::vector<FTrackSegment> Segments;
};
