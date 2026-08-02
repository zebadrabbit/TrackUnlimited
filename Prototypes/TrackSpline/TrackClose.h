// TrackUnlimited Phase 1: circuit closure — measuring the gap, and solving it.
// Plain C++17, no engine dependency, same conventions as TrackSpline.h.
//
// PHASE0_FINDINGS calls this "the one real cost of the representation". You
// author curvature and you arrive wherever you arrive; a control-point model
// would close a circuit for free because the endpoint IS an authored value.
// Here it is a consequence, and consequences have to be solved for.
//
// The measured cost, from the findings: a symmetric oval closes at 0.0000 m,
// but easements of 20 m vs 8 m leave a 0.93 m gap, and radii of 30 m vs 45 m
// leave 29.63 m. Heading closes exactly in every case, because total turn angle
// is authored directly — so position is the hard part and heading usually comes
// free. The vertical slice hit the same wall from the other axis: its layout
// ends 8.5 m BELOW its station, which is a height gap nobody typed.
//
// Two halves, and the first is worth having on its own:
//
//   MeasureClosure — how far off is it, on each axis separately. This is what
//   an editor status bar shows continuously, and it needs no solver.
//
//   SolveClosure — adjust chosen AUTHORED parameters until the gap closes.
//
// It solves for authored parameters, never derived ones. Driving a curvature
// field directly would produce a segment whose radius says 30 and whose
// curvature says 0.0341 — the two representations disagreeing is exactly what
// TrackIO.h exists to prevent, and a solver is no more entitled to do it than
// a file format is.
//
// Units: metres, radians internally. FAuthoredSegment angles stay degrees, and
// the solver works in whatever unit the field it is moving is stored in.

#pragma once

#include "TrackIO.h"
#include "TrackSpline.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

// ------------------------------------------------------------------ measuring

// Where a circuit wants to end up. Position is nearly always wanted; heading
// usually closes on its own because the author typed the turn angles; roll is
// rarely interesting because it is authored directly at the last segment.
struct FClosureTarget
{
    FVec3 Position{0.0, 0.0, 0.0};
    FVec3 Tangent{1.0, 0.0, 0.0};
    double Roll = 0.0; // radians, path-relative — matches FTrackFrame::Roll

    // Per axis, because "closed" is not one question. A circuit wants all three.
    // A point-to-point layout — which is most of what exists right now — wants
    // only Z: the brake run has to arrive at platform HEIGHT, and it has no
    // business ending up back at the station's footprint. Asking for XY there
    // would be asking the wrong thing and getting a truthful "impossible".
    bool bMatchX = true;
    bool bMatchY = true;
    bool bMatchZ = true;

    bool bMatchHeading = false;
    bool bMatchRoll = false;
};

struct FClosureGap
{
    FVec3 Position{0.0, 0.0, 0.0}; // end minus target
    double PositionError = 0.0;    // magnitude of the above
    double HeightError = 0.0;      // the Z component alone

    double HeadingError = 0.0; // radians between the two tangents
    double RollError = 0.0;    // radians

    // PositionError over only the axes the target asked about. This is what
    // convergence is judged on — a height-only target must not be held to an
    // X gap it was never asked to close.
    double ActiveError = 0.0;

    // Height gets its own field because it is the one that does not announce
    // itself. A track that ends 8.5 m low still LOOKS closed from above, and
    // the vertical slice shipped that way — the cart simply fell through where
    // the station should have been.
};

inline FClosureGap MeasureClosure(const FTrack& Track, const FClosureTarget& Target)
{
    FClosureGap G;
    const FTrackFrame End = Track.EvaluateAt(Track.TotalLength());

    G.Position = End.Position - Target.Position;
    G.PositionError = Length(G.Position);
    G.HeightError = G.Position.Z;

    // Well-conditioned angle: acos of a dot product loses precision exactly
    // where it matters, near zero error. Same fix NL2Csv.h needed, measured
    // there at 110x better.
    const FVec3 Axis = Cross(End.Tangent, Target.Tangent);
    G.HeadingError = std::atan2(Length(Axis), Dot(End.Tangent, Target.Tangent));

    G.RollError = End.Roll - Target.Roll;

    double Active = 0.0;
    if (Target.bMatchX) { Active += G.Position.X * G.Position.X; }
    if (Target.bMatchY) { Active += G.Position.Y * G.Position.Y; }
    if (Target.bMatchZ) { Active += G.Position.Z * G.Position.Z; }
    G.ActiveError = std::sqrt(Active);
    return G;
}

// A closed circuit ends where it started, so the target is the start frame.
// Reads the track's actual opening frame rather than assuming the integrator's
// hardcoded +X start, so this keeps working if that ever becomes authored.
inline FClosureTarget CircuitTarget(const FTrack& Track)
{
    const FTrackFrame Start = Track.EvaluateAt(0.0);
    FClosureTarget T;
    T.Position = Start.Position;
    T.Tangent = Start.Tangent;
    T.Roll = Start.Roll;
    return T;
}

// For a point-to-point layout: come back to platform HEIGHT and nothing else.
// A station-to-brake-run ride is not supposed to end where it started in plan,
// so a full circuit target would report an impossibility that is not a fault.
inline FClosureTarget HeightTarget(const FTrack& Track)
{
    FClosureTarget T = CircuitTarget(Track);
    T.bMatchX = false;
    T.bMatchY = false;
    return T;
}

// ------------------------------------------------------------------- freedoms

// Which authored number the solver may move. Nothing is free unless the author
// says so — a solver that helpfully retuned a lift hill's radius to close a
// circuit would be destroying the design to satisfy the arithmetic.
enum class EClosureField
{
    Length,            // straight, arc, clothoid
    Radius,            // arc, helix
    CurvatureStart,    // clothoid
    CurvatureEnd,      // clothoid
    ClimbAngleDegrees, // helix
    Turns,             // helix
    RollEndDegrees,    // any kind
};

struct FClosureFreedom
{
    std::size_t SegmentIndex = 0;
    EClosureField Field = EClosureField::Length;

    // Bounds, in the field's own units. Defaults are deliberately loose except
    // where crossing a value would change the geometry's meaning rather than
    // its size — see MinAbs below.
    double Min = -1e9;
    double Max = 1e9;

    // For Radius only: the sign is LOCKED to whatever the segment already has,
    // and the magnitude is floored here. Radius +ve is a left turn and -ve is a
    // right turn, so a solver allowed to cross zero would silently reverse a
    // turn on its way through an infinite curvature — and PHASE0_FINDINGS
    // records what a zero radius actually produces: not a visible NaN, but a
    // clean, straight segment that passes every check.
    double MinAbs = 5.0;
};

inline FClosureFreedom FreeLength(std::size_t Index, double MinLength = 1.0,
                                  double MaxLength = 1000.0)
{
    FClosureFreedom F;
    F.SegmentIndex = Index;
    F.Field = EClosureField::Length;
    F.Min = MinLength;
    F.Max = MaxLength;
    return F;
}

inline FClosureFreedom FreeRadius(std::size_t Index, double MinAbsRadius = 5.0,
                                  double MaxAbsRadius = 5000.0)
{
    FClosureFreedom F;
    F.SegmentIndex = Index;
    F.Field = EClosureField::Radius;
    F.MinAbs = MinAbsRadius;
    F.Max = MaxAbsRadius;
    F.Min = -MaxAbsRadius;
    return F;
}

inline FClosureFreedom FreeField(std::size_t Index, EClosureField Field, double Min, double Max)
{
    FClosureFreedom F;
    F.SegmentIndex = Index;
    F.Field = Field;
    F.Min = Min;
    F.Max = Max;
    return F;
}

// ------------------------------------------------------------------- options

struct FClosureOptions
{
    double PositionTolerance = 1e-3; // metres. 1 mm is far below what track hardware holds.
    int MaxIterations = 60;

    // Position is in metres and heading is in radians, so the two cannot be
    // summed without a scale. This says how many metres of position error one
    // radian of heading error is worth. It is a judgement about what the author
    // cares about, not a physical constant — hence a knob.
    double HeadingWeightMetres = 20.0;
    double RollWeightMetres = 5.0;

    // Write the solved numbers back only if it actually closed. A track left
    // with un-round authored values that STILL does not close is worse than the
    // one the author typed: they have lost their own numbers and gained
    // nothing. Set true to keep a partial improvement.
    bool bApplyOnFailure = false;
};

struct FClosureResult
{
    bool bConverged = false;
    FClosureGap Before;
    FClosureGap After; // best reached, whether or not it was applied
    bool bApplied = false;

    int Iterations = 0;
    int Evaluations = 0; // full track rebuild + walk; the real cost
    std::string Message;
};

namespace TrackCloseDetail
{

inline double* FieldPtr(FAuthoredSegment& A, EClosureField Field)
{
    switch (Field)
    {
    case EClosureField::Length:
        return A.Kind == ESegmentKind::Raw ? &A.RawSegment.Length : &A.Length;
    case EClosureField::Radius: return &A.Radius;
    case EClosureField::CurvatureStart: return &A.CurvatureStart;
    case EClosureField::CurvatureEnd: return &A.CurvatureEnd;
    case EClosureField::ClimbAngleDegrees: return &A.ClimbAngleDegrees;
    case EClosureField::Turns: return &A.Turns;
    case EClosureField::RollEndDegrees: return &A.RollEndDegrees;
    }
    return nullptr;
}

inline const char* FieldName(EClosureField Field)
{
    switch (Field)
    {
    case EClosureField::Length: return "length";
    case EClosureField::Radius: return "radius";
    case EClosureField::CurvatureStart: return "curvatureStart";
    case EClosureField::CurvatureEnd: return "curvatureEnd";
    case EClosureField::ClimbAngleDegrees: return "climbAngleDeg";
    case EClosureField::Turns: return "turns";
    case EClosureField::RollEndDegrees: return "rollEndDeg";
    }
    return "?";
}

// Solve A x = b for small dense A, Gaussian elimination with partial pivoting.
// N is at most the number of free parameters, so this is a handful of rows and
// a library would be more dependency than arithmetic.
inline bool SolveDense(std::vector<double>& A, std::vector<double>& B, int N,
                       std::vector<double>& X)
{
    for (int Col = 0; Col < N; ++Col)
    {
        int Pivot = Col;
        for (int Row = Col + 1; Row < N; ++Row)
        {
            if (std::fabs(A[Row * N + Col]) > std::fabs(A[Pivot * N + Col]))
            {
                Pivot = Row;
            }
        }
        if (std::fabs(A[Pivot * N + Col]) < 1e-14)
        {
            return false; // singular: the free parameters do not span the gap
        }
        if (Pivot != Col)
        {
            for (int K = 0; K < N; ++K)
            {
                std::swap(A[Pivot * N + K], A[Col * N + K]);
            }
            std::swap(B[Pivot], B[Col]);
        }
        for (int Row = Col + 1; Row < N; ++Row)
        {
            const double F = A[Row * N + Col] / A[Col * N + Col];
            if (F == 0.0)
            {
                continue;
            }
            for (int K = Col; K < N; ++K)
            {
                A[Row * N + K] -= F * A[Col * N + K];
            }
            B[Row] -= F * B[Col];
        }
    }
    X.assign(static_cast<std::size_t>(N), 0.0);
    for (int Row = N - 1; Row >= 0; --Row)
    {
        double Sum = B[Row];
        for (int K = Row + 1; K < N; ++K)
        {
            Sum -= A[Row * N + K] * X[static_cast<std::size_t>(K)];
        }
        X[static_cast<std::size_t>(Row)] = Sum / A[Row * N + Row];
    }
    return true;
}

} // namespace TrackCloseDetail

// -------------------------------------------------------------------- solving

// Damped Gauss-Newton (Levenberg) on the authored parameters the caller freed.
// The residual is one full rebuild-and-walk of the track, so this is not cheap
// and the count is reported rather than hidden.
//
// ponytail: rebuilds and re-walks the WHOLE track per residual, including the
// part before the first free parameter, which cannot have changed. Caching the
// frame at the earliest free segment and using AdvanceFrom from there is the
// obvious win when the free parameters sit near the end — which is the common
// case. Left undone until an editor measures it slow; Evaluations is reported
// so there is a number to measure.
inline FClosureResult SolveClosure(FTrackDocument& Doc, const FClosureTarget& Target,
                                   const std::vector<FClosureFreedom>& Freedoms,
                                   const FClosureOptions& Options = FClosureOptions())
{
    using namespace TrackCloseDetail;

    FClosureResult R;
    const int K = static_cast<int>(Freedoms.size());

    // Rows: position (3), optionally heading (3, rank 2) and roll (1).
    int Rows = 0;
    if (Target.bMatchX) { ++Rows; }
    if (Target.bMatchY) { ++Rows; }
    if (Target.bMatchZ) { ++Rows; }
    if (Target.bMatchHeading) { Rows += 3; }
    if (Target.bMatchRoll) { Rows += 1; }

    if (K == 0 || Rows == 0)
    {
        R.Message = "nothing to solve: no free parameters, or no constraints";
        R.Before = R.After = MeasureClosure(BuildTrack(Doc), Target);
        return R;
    }

    // Validate the freedoms before touching anything. A bad index or a field
    // the segment kind does not have would otherwise be a silent no-op that
    // looks like an unsolvable track.
    for (const FClosureFreedom& F : Freedoms)
    {
        if (F.SegmentIndex >= Doc.Segments.size())
        {
            R.Message = "free parameter names segment " + std::to_string(F.SegmentIndex)
                        + ", which does not exist";
            R.Before = R.After = MeasureClosure(BuildTrack(Doc), Target);
            return R;
        }
    }

    FTrackDocument Work = Doc;
    const FTrackDocument Original = Doc;

    // Current parameter values, and the sign lock for any radius.
    std::vector<double> P(static_cast<std::size_t>(K));
    std::vector<double> RadiusSign(static_cast<std::size_t>(K), 0.0);
    for (int i = 0; i < K; ++i)
    {
        double* Ptr = FieldPtr(Work.Segments[Freedoms[static_cast<std::size_t>(i)].SegmentIndex],
                               Freedoms[static_cast<std::size_t>(i)].Field);
        P[static_cast<std::size_t>(i)] = *Ptr;
        if (Freedoms[static_cast<std::size_t>(i)].Field == EClosureField::Radius)
        {
            RadiusSign[static_cast<std::size_t>(i)] = (*Ptr < 0.0) ? -1.0 : 1.0;
        }
    }

    auto Clamp = [&](int i, double V) {
        const FClosureFreedom& F = Freedoms[static_cast<std::size_t>(i)];
        if (F.Field == EClosureField::Radius)
        {
            // Magnitude floored, sign locked. Crossing zero would flip the turn
            // direction and pass through an infinite curvature to do it.
            const double S = RadiusSign[static_cast<std::size_t>(i)];
            double Mag = std::fabs(V);
            if (Mag < F.MinAbs) { Mag = F.MinAbs; }
            if (Mag > std::fabs(F.Max)) { Mag = std::fabs(F.Max); }
            return S * Mag;
        }
        if (V < F.Min) { return F.Min; }
        if (V > F.Max) { return F.Max; }
        return V;
    };

    auto Apply = [&](const std::vector<double>& Params) {
        for (int i = 0; i < K; ++i)
        {
            double* Ptr =
                FieldPtr(Work.Segments[Freedoms[static_cast<std::size_t>(i)].SegmentIndex],
                         Freedoms[static_cast<std::size_t>(i)].Field);
            *Ptr = Params[static_cast<std::size_t>(i)];
        }
    };

    // One rebuild and one walk per call, and that is the unit Evaluations
    // counts. Anything that walks the track twice here doubles the cost of the
    // whole solve, since the Jacobian calls this 2K times per iteration.
    auto Residual = [&](const std::vector<double>& Params, std::vector<double>& Out) {
        Apply(Params);
        ++R.Evaluations;
        const FTrack Built = BuildTrack(Work);
        const FTrackFrame End = Built.EvaluateAt(Built.TotalLength());
        const FClosureGap G = MeasureClosure(Built, Target);
        Out.clear();
        if (Target.bMatchX) { Out.push_back(G.Position.X); }
        if (Target.bMatchY) { Out.push_back(G.Position.Y); }
        if (Target.bMatchZ) { Out.push_back(G.Position.Z); }
        if (Target.bMatchHeading)
        {
            // The tangent DIFFERENCE, not the angle between them. The angle is
            // non-negative, so it has no gradient sign at zero and Gauss-Newton
            // cannot descend through it; the vector difference is smooth there.
            // Rank 2, not 3, since both are unit — least squares handles that.
            const FVec3 D = End.Tangent - Target.Tangent;
            Out.push_back(D.X * Options.HeadingWeightMetres);
            Out.push_back(D.Y * Options.HeadingWeightMetres);
            Out.push_back(D.Z * Options.HeadingWeightMetres);
        }
        if (Target.bMatchRoll)
        {
            Out.push_back(G.RollError * Options.RollWeightMetres);
        }
        return G;
    };

    auto Norm = [](const std::vector<double>& V) {
        double S = 0.0;
        for (const double X : V) { S += X * X; }
        return std::sqrt(S);
    };

    std::vector<double> Res;
    R.Before = Residual(P, Res);
    R.After = R.Before;
    double Best = Norm(Res);
    std::vector<double> BestP = P;

    if (R.Before.ActiveError <= Options.PositionTolerance
        && (!Target.bMatchHeading || R.Before.HeadingError <= 1e-6))
    {
        R.bConverged = true;
        R.Message = "already closed; nothing changed";
        Doc = Original;
        return R;
    }

    double Lambda = 1e-3;
    std::vector<double> Trial(static_cast<std::size_t>(K));
    std::vector<double> TrialRes;
    std::vector<double> J(static_cast<std::size_t>(Rows * K));

    for (int Iter = 0; Iter < Options.MaxIterations; ++Iter)
    {
        R.Iterations = Iter + 1;

        // Central-difference Jacobian. The step is deliberately COARSE:
        // PHASE0_FINDINGS records that the endpoint is not smooth in arc length
        // below ~1e-9 m, because Steps = ceil(Span/MaxStep) re-discretises per
        // call, and that a difference quotient stops converging under h ~ 1e-4.
        // A textbook 1e-8 step here would be differentiating the integrator's
        // step-count staircase rather than the geometry.
        for (int i = 0; i < K; ++i)
        {
            const double Scale = std::fmax(1.0, std::fabs(P[static_cast<std::size_t>(i)]));
            const double H = 1e-3 * Scale;

            std::vector<double> Plus = P, Minus = P;
            Plus[static_cast<std::size_t>(i)] = Clamp(i, P[static_cast<std::size_t>(i)] + H);
            Minus[static_cast<std::size_t>(i)] = Clamp(i, P[static_cast<std::size_t>(i)] - H);
            const double Span = Plus[static_cast<std::size_t>(i)] - Minus[static_cast<std::size_t>(i)];
            if (Span == 0.0)
            {
                for (int Row = 0; Row < Rows; ++Row) { J[static_cast<std::size_t>(Row * K + i)] = 0.0; }
                continue;
            }
            std::vector<double> RPlus, RMinus;
            Residual(Plus, RPlus);
            Residual(Minus, RMinus);
            for (int Row = 0; Row < Rows; ++Row)
            {
                J[static_cast<std::size_t>(Row * K + i)] =
                    (RPlus[static_cast<std::size_t>(Row)] - RMinus[static_cast<std::size_t>(Row)])
                    / Span;
            }
        }

        // Normal equations with Levenberg damping on the diagonal.
        std::vector<double> A(static_cast<std::size_t>(K * K), 0.0);
        std::vector<double> B(static_cast<std::size_t>(K), 0.0);
        for (int a = 0; a < K; ++a)
        {
            for (int b = 0; b < K; ++b)
            {
                double S = 0.0;
                for (int Row = 0; Row < Rows; ++Row)
                {
                    S += J[static_cast<std::size_t>(Row * K + a)]
                         * J[static_cast<std::size_t>(Row * K + b)];
                }
                A[static_cast<std::size_t>(a * K + b)] = S;
            }
            double S = 0.0;
            for (int Row = 0; Row < Rows; ++Row)
            {
                S += J[static_cast<std::size_t>(Row * K + a)] * Res[static_cast<std::size_t>(Row)];
            }
            B[static_cast<std::size_t>(a)] = -S;
        }
        for (int a = 0; a < K; ++a)
        {
            A[static_cast<std::size_t>(a * K + a)] *= (1.0 + Lambda);
            A[static_cast<std::size_t>(a * K + a)] += 1e-12;
        }

        std::vector<double> Step;
        if (!SolveDense(A, B, K, Step))
        {
            R.Message = "the freed parameters cannot move the endpoint in the direction the "
                        "gap needs — free a different one, or one more";
            break;
        }

        for (int i = 0; i < K; ++i)
        {
            Trial[static_cast<std::size_t>(i)] =
                Clamp(i, P[static_cast<std::size_t>(i)] + Step[static_cast<std::size_t>(i)]);
        }
        const FClosureGap TrialGap = Residual(Trial, TrialRes);
        const double TrialNorm = Norm(TrialRes);

        if (TrialNorm < Best)
        {
            P = Trial;
            Res = TrialRes;
            Best = TrialNorm;
            BestP = Trial;
            R.After = TrialGap;
            Lambda = std::fmax(Lambda / 3.0, 1e-9);

            if (TrialGap.ActiveError <= Options.PositionTolerance
                && (!Target.bMatchHeading || TrialGap.HeadingError <= 1e-6))
            {
                R.bConverged = true;
                break;
            }
        }
        else
        {
            // Rejected: trust the linear model less and try a shorter step.
            Lambda *= 8.0;
            if (Lambda > 1e9)
            {
                R.Message = "no step in any freed parameter improves the gap; this is as "
                            "close as these freedoms reach";
                break;
            }
        }
    }

    if (R.bConverged)
    {
        Apply(BestP);
        Doc = Work;
        R.bApplied = true;
        if (R.Message.empty())
        {
            R.Message = "closed";
        }
    }
    else if (Options.bApplyOnFailure)
    {
        Apply(BestP);
        Doc = Work;
        R.bApplied = true;
    }
    else
    {
        // The author's own numbers are worth more than a partial improvement
        // they did not ask for. Same instinct as TrackValidate.h: report the
        // problem, do not half-fix it.
        Doc = Original;
        if (R.Message.empty())
        {
            R.Message = "did not reach tolerance; the document was left unchanged";
        }
    }
    return R;
}
