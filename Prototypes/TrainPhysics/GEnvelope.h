// TrackUnlimited: judging a ride profile against published acceleration envelopes.
// Plain C++17, no engine dependency, same conventions as RideProfile.h.
//
// RunRideProfile already reports felt G on three axes and the extremes with their
// locations. This turns those numbers into a VERDICT — not "4.25 g at S=310 m"
// but "4.25 g sustained for 2.1 s at S=310 m exceeds the 1-3 s allowance of 4 g
// by 0.25 g".
//
// REPORT, NEVER REPAIR, exactly as TrackValidate does. A track that leaves an
// envelope is still buildable and still rideable; it is told where and by how
// much. Nothing here alters a layout, and nothing refuses to run one.
//
// ---------------------------------------------------------------------------
// PROVENANCE, AND WHY IT IS WRITTEN DOWN HERE
//
// The limit tables below came from a Cowork research session (2026-08-03,
// recorded on the Phase 3.75 Trello card) and are NOT verified against the
// published documents. ASTM F2291 and EN 13814 are both paywalled; nobody on
// this project has checked a copy. They are internally consistent and plausibly
// shaped, which is not the same as correct.
//
// This is the same hygiene Docs/CONTROL_ARCHITECTURE.md uses for its standards
// table, and for the same reason: a tool that prints "ASTM F2291" carries more
// authority than an unverified table has earned.
//
// THE CODE IS DELIBERATELY INDIFFERENT TO THE NUMBERS. Bands are data; swapping
// in a verified table is editing Harmonised() and changes nothing else. If you
// have a copy of either standard, that function is the whole of what to check.
// ---------------------------------------------------------------------------
//
// Axes are the patron-body-fixed frame the standards use, which is what
// FRideSample already carries:
//
//   +Gz   spine, eyeballs-down. Pulling out of a drop. Sample.VerticalG.
//   -Gz   airtime, eyeballs-up. Sample.VerticalG negative.
//   +-Gy  lateral. Sample.LateralG.
//   +Gx   chest-to-back, eyeballs-in. A launch. Sample.TangentialG positive.
//   -Gx   back-to-chest. Braking. Sample.TangentialG negative.

#pragma once

#include "RideProfile.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

// Which standard's limits are being applied. They are harmonised on the numbers
// below, so this does NOT currently change any threshold — it changes what the
// verdict says it applied, which matters because the two differ in a way this
// cannot yet honour. See FGVerdict::MeasurementNote.
enum class EGStandard
{
    ASTM_F2291,
    EN_13814,
};

// One rung of a tolerance curve.
//
// READ IT AS: no continuous run above `Threshold` may last longer than
// `MaxSeconds`. That is the inverted form of how the standards publish it
// ("allowable +Gz is 6 g up to 1 s, 4 g from 1 to 3 s, ..."), and the inversion
// is worth spelling out because it is not obvious and looks wrong at a glance.
//
// Published:  limit(T) = 6 for T<=1s, 4 for 1<T<=3s, 3 for 3<T<=12s, ...
// Inverted:   above 6 g -> 0 s allowed.  above 4 g -> 1 s.  above 3 g -> 3 s.
//
// Check the two agree on a case: 5.9 g held for 2.9 s. Published says T=2.9 sits
// in the 1-3 s band, limit 4 g, so 5.9 g fails. Inverted says the run above 4 g
// lasted 2.9 s against an allowance of 1 s, so it fails. Same verdict.
//
// Getting this backwards produces a checker that passes 5.9 g for three seconds,
// which is the failure mode worth guarding: it is silent, and it is generous.
struct FGBand
{
    double Threshold = 0.0;   // g
    double MaxSeconds = 0.0;  // seconds a run above Threshold may last
};

struct FGEnvelope
{
    // Descending thresholds. Each axis judged independently.
    std::vector<FGBand> PosGz;
    std::vector<FGBand> NegGz;   // thresholds on the MAGNITUDE of negative Gz
    std::vector<FGBand> AbsGy;   // thresholds on |Gy|
    std::vector<FGBand> PosGx;
    std::vector<FGBand> NegGx;   // thresholds on the MAGNITUDE of negative Gx

    // Below this, an event is an IMPACT and sits outside the sustained envelope
    // entirely rather than being a short violation of it. Roughly neuromuscular
    // reaction time: under it the body has not begun to respond, so a sustained-
    // load curve is not the right instrument. Impacts are still REPORTED.
    double ImpactSeconds = 0.20;

    // Jerk, over a least-squares window rather than sample to sample — a
    // difference between adjacent samples is dominated by whatever the
    // integrator did, not by what the rider feels.
    double JerkWindowSeconds = 0.10;
    double JerkDesignGPerSec = 10.0;
    double JerkAllowableGPerSec = 15.0;

    // A reversal shorter than this is judged against HALVED sustained values,
    // because the body is still loaded the other way when the load flips.
    double ReversalMinSeconds = 0.2;
    double ReversalDerate = 0.5;

    // F2137's SARC filter. 5 Hz for evaluating what a patron experiences, 1 Hz
    // for restraint design. FILTER BEFORE JUDGING — unfiltered, every integrator
    // artefact and every curvature joint reads as a violation, and the tool
    // becomes noise with a standard's name on it.
    double FilterHz = 5.0;

    EGStandard Standard = EGStandard::ASTM_F2291;

    // The shipped table. UNVERIFIED — see the provenance note at the top.
    static FGEnvelope Harmonised(EGStandard For = EGStandard::ASTM_F2291)
    {
        FGEnvelope E;
        E.Standard = For;

        // +Gz  6 g <=1 s | 4 g 1-3 s | 3 g 3-12 s | 2 g 12-40 s | 1 g >40 s
        E.PosGz = {{6.0, 0.0}, {4.0, 1.0}, {3.0, 3.0}, {2.0, 12.0}, {1.0, 40.0}};

        // -Gz  1.1 g at 3 s. Published as a single point rather than a curve, so
        // it is encoded as one: a run beyond 1.1 g of airtime may last 3 s.
        E.NegGz = {{1.1, 3.0}};

        // +-Gy  3.0 g <=2 s | 2.5 g 2-4 s | 1.25 g longer
        E.AbsGy = {{3.0, 0.0}, {2.5, 2.0}, {1.25, 4.0}};

        // +Gx  6 g with a headrest | 2.0 g without, max 4 s.
        // The headrest figure is taken as the ceiling because every coaster this
        // project models has one; the 2.0 g no-headrest case is a restraint
        // decision the segment vocabulary cannot currently express.
        E.PosGx = {{6.0, 0.0}, {2.0, 4.0}};

        // -Gx  2.0 g, max 4 s.
        E.NegGx = {{2.0, 4.0}};

        return E;
    }
};

enum class EGFindingKind
{
    Sustained,   // a run held above a band for longer than the band allows
    Impact,      // shorter than ImpactSeconds: outside the sustained envelope
    Jerk,        // rate of change over the least-squares window
    Combined,    // the normalised elliptical sum across all three axes
    Reversal,    // a sign flip too brief to be judged at full allowance
};

struct FGFinding
{
    EGFindingKind Kind = EGFindingKind::Sustained;
    const char* Axis = "";
    double AtS = 0.0;        // metres — where to go and look
    double AtTime = 0.0;     // seconds since dispatch
    double Duration = 0.0;   // seconds the run lasted
    double Value = 0.0;      // the worst value reached in the run
    double Limit = 0.0;      // what was allowed
    double Exceedance() const { return Value - Limit; }
};

struct FGVerdict
{
    bool bPasses = true;                 // no Sustained/Combined/Reversal findings
    std::vector<FGFinding> Findings;     // violations, worst first
    std::vector<FGFinding> Impacts;      // reported, never a pass/fail
    std::vector<FGFinding> JerkNotes;    // above design rate but under allowable

    EGStandard Standard = EGStandard::ASTM_F2291;
    double FilteredAtHz = 5.0;
    std::size_t SamplesJudged = 0;

    // WHAT THIS DID NOT DO, carried in the verdict rather than a comment,
    // because a conformance result that hides its own gaps is worse than none.
    std::string MeasurementNote;
    std::string NotApplied;
};

namespace GEnvelopeDetail
{
    // Second-order Butterworth low-pass, bilinear transform, run FORWARD THEN
    // BACKWARD so the result has zero phase lag.
    //
    // The zero-phase part is not a nicety. A single-pass IIR delays the signal,
    // and the whole product here is "at S = 310 m" — a filter that shifts a peak
    // half a second downstream reports the violation in the wrong place, on the
    // wrong piece of track, and an author goes and looks at innocent geometry.
    inline void ButterworthZeroPhase(std::vector<double>& X, double SampleHz, double CutoffHz)
    {
        if (X.size() < 3 || !(SampleHz > 0.0) || !(CutoffHz > 0.0)
            || CutoffHz >= 0.5 * SampleHz)
        {
            return;
        }

        const double Pi = 3.14159265358979323846;
        const double W = std::tan(Pi * CutoffHz / SampleHz);
        const double Root2 = std::sqrt(2.0);
        const double N = 1.0 / (1.0 + Root2 * W + W * W);

        const double B0 = W * W * N;
        const double B1 = 2.0 * B0;
        const double B2 = B0;
        const double A1 = 2.0 * (W * W - 1.0) * N;
        const double A2 = (1.0 - Root2 * W + W * W) * N;

        auto Pass = [&](std::vector<double>& V)
        {
            // Seeded with the first value rather than zero: a ride starts at 1 g
            // vertical, and a filter starting from rest rings for the first half
            // second of every profile. That transient would be reported as a jerk
            // finding at S = 0 on every track ever checked.
            double X1 = V[0], X2 = V[0], Y1 = V[0], Y2 = V[0];
            for (double& S : V)
            {
                const double In = S;
                const double Out = B0 * In + B1 * X1 + B2 * X2 - A1 * Y1 - A2 * Y2;
                X2 = X1; X1 = In;
                Y2 = Y1; Y1 = Out;
                S = Out;
            }
        };

        Pass(X);
        std::reverse(X.begin(), X.end());
        Pass(X);
        std::reverse(X.begin(), X.end());
    }

    // One uniformly-sampled channel, in TIME.
    //
    // RunRideProfile samples by ARC LENGTH, which is right for a graph against S
    // and wrong for every check here: a duration-dependent envelope needs a
    // uniform time base, and at 1 m spacing a sample is 0.025 s apart at 40 m/s
    // and 0.5 s apart at 2 m/s. Judging durations on that grid measures the
    // train's speed, not the rider's experience. The filter needs it too.
    struct FUniform
    {
        double Hz = 0.0;
        double T0 = 0.0;
        std::vector<double> Gz, Gy, Gx, S;
    };

    inline FUniform Resample(const FRideProfile& P, double Hz)
    {
        FUniform U;
        U.Hz = Hz;
        if (P.Samples.size() < 2 || !(Hz > 0.0))
        {
            return U;
        }

        U.T0 = P.Samples.front().Time;
        const double T1 = P.Samples.back().Time;
        if (!(T1 > U.T0))
        {
            return U;
        }

        const std::size_t N = static_cast<std::size_t>((T1 - U.T0) * Hz) + 1;
        U.Gz.reserve(N); U.Gy.reserve(N); U.Gx.reserve(N); U.S.reserve(N);

        std::size_t j = 0;
        for (std::size_t i = 0; i < N; ++i)
        {
            const double T = U.T0 + static_cast<double>(i) / Hz;
            while (j + 2 < P.Samples.size() && P.Samples[j + 1].Time < T) { ++j; }

            const FRideSample& A = P.Samples[j];
            const FRideSample& B = P.Samples[j + 1];
            const double Span = B.Time - A.Time;
            const double F = (Span > 1e-12) ? (T - A.Time) / Span : 0.0;
            const double Fc = (F < 0.0) ? 0.0 : (F > 1.0 ? 1.0 : F);

            U.Gz.push_back(A.VerticalG + Fc * (B.VerticalG - A.VerticalG));
            U.Gy.push_back(A.LateralG + Fc * (B.LateralG - A.LateralG));
            U.Gx.push_back(A.TangentialG + Fc * (B.TangentialG - A.TangentialG));
            U.S.push_back(A.S + Fc * (B.S - A.S));
        }
        return U;
    }

    // Every maximal run where Value(i) exceeds Threshold, judged against the
    // band's allowance. Signed: Sign +1 tests the channel, -1 tests its negation,
    // so one routine serves +Gz and -Gz without a second copy that can drift.
    inline void ScanBand(const FUniform& U, const std::vector<double>& Ch, double Sign,
                         const FGBand& Band, const char* Axis, double ImpactSeconds,
                         std::vector<FGFinding>& Out, std::vector<FGFinding>& Impacts)
    {
        const double Dt = 1.0 / U.Hz;
        std::size_t i = 0;
        while (i < Ch.size())
        {
            if (Sign * Ch[i] <= Band.Threshold)
            {
                ++i;
                continue;
            }

            const std::size_t Start = i;
            double Worst = Sign * Ch[i];
            std::size_t WorstAt = i;
            while (i < Ch.size() && Sign * Ch[i] > Band.Threshold)
            {
                if (Sign * Ch[i] > Worst) { Worst = Sign * Ch[i]; WorstAt = i; }
                ++i;
            }

            const double Duration = static_cast<double>(i - Start) * Dt;

            FGFinding F;
            F.Axis = Axis;
            F.AtS = U.S[WorstAt];
            F.AtTime = U.T0 + static_cast<double>(WorstAt) * Dt;
            F.Duration = Duration;
            F.Value = Worst;
            F.Limit = Band.Threshold;

            // Inside the band's allowance: nothing to say, at any duration.
            if (Duration <= Band.MaxSeconds)
            {
                continue;
            }

            // AN IMPACT IS NOT A SHORT VIOLATION. Under the reaction threshold
            // the sustained curve is the wrong instrument, so the event leaves
            // the pass/fail entirely. Without this a single-sample spike off a
            // curvature joint fails every track.
            //
            // AND IT IS REPORTED ONLY BECAUSE THE EXEMPTION IS WHAT SAVED IT.
            // The duration test sits AFTER the band test on purpose: an impact is
            // worth telling an author about when it would have been a violation
            // had it lasted, and is noise otherwise. Reporting every short
            // excursion instead gave the worked layout seven "impacts" that were
            // 1.02 g wobbles crossing the 1 g band on level track — the resting
            // value of every ride, so every ride tripped it.
            F.Kind = (Duration < ImpactSeconds) ? EGFindingKind::Impact
                                                : EGFindingKind::Sustained;
            if (F.Kind == EGFindingKind::Impact) { Impacts.push_back(F); }
            else                                 { Out.push_back(F); }
        }
    }

    // Least-squares slope over a centred window. Uniform spacing, so the
    // denominator is constant and hoisted.
    inline double SlopeAt(const std::vector<double>& Ch, std::size_t Centre,
                          std::size_t Half, double Dt)
    {
        const std::size_t A = (Centre > Half) ? Centre - Half : 0;
        const std::size_t B = std::min(Ch.size() - 1, Centre + Half);
        const std::size_t N = B - A + 1;
        if (N < 3) { return 0.0; }

        const double Mean = 0.5 * static_cast<double>(N - 1);
        double Num = 0.0, Den = 0.0, Ybar = 0.0;
        for (std::size_t i = A; i <= B; ++i) { Ybar += Ch[i]; }
        Ybar /= static_cast<double>(N);
        for (std::size_t i = A; i <= B; ++i)
        {
            const double X = static_cast<double>(i - A) - Mean;
            Num += X * (Ch[i] - Ybar);
            Den += X * X;
        }
        return (Den > 1e-12) ? (Num / Den) / Dt : 0.0;
    }
}

// The whole check. One pass, no state, nothing cached.
inline FGVerdict JudgeRideProfile(const FRideProfile& P,
                                  const FGEnvelope& E = FGEnvelope::Harmonised(),
                                  double JudgeHz = 100.0)
{
    using namespace GEnvelopeDetail;

    FGVerdict V;
    V.Standard = E.Standard;
    V.FilteredAtHz = E.FilterHz;

    // Carried in the result rather than left in a comment. A conformance verdict
    // that quietly omits what it could not check is worse than no verdict.
    V.MeasurementNote = (E.Standard == EGStandard::ASTM_F2291)
        ? "ASTM limits applied. Measured at the HEARTLINE, which is where this "
          "model computes felt G and where ASTM specifies."
        : "EN 13814 limits applied. EN measures nearer HEAD level; this model "
          "computes felt G at the heartline and does NOT offset for that. The "
          "difference is real and this verdict does not account for it.";

    V.NotApplied =
        "Push-pull (reduced +Gz allowance after >=5 s sustained -Gz) is NOT "
        "applied: the source gives the effect but no reduction factor, and "
        "inventing one would be worse than omitting it. Restraint-design "
        "judgement at 1 Hz is not run; this is the 5 Hz patron evaluation.";

    FUniform U = Resample(P, JudgeHz);
    if (U.Gz.size() < 3)
    {
        return V;
    }
    V.SamplesJudged = U.Gz.size();

    ButterworthZeroPhase(U.Gz, JudgeHz, E.FilterHz);
    ButterworthZeroPhase(U.Gy, JudgeHz, E.FilterHz);
    ButterworthZeroPhase(U.Gx, JudgeHz, E.FilterHz);

    for (const FGBand& B : E.PosGz)
    { ScanBand(U, U.Gz, +1.0, B, "+Gz", E.ImpactSeconds, V.Findings, V.Impacts); }
    for (const FGBand& B : E.NegGz)
    { ScanBand(U, U.Gz, -1.0, B, "-Gz", E.ImpactSeconds, V.Findings, V.Impacts); }
    for (const FGBand& B : E.AbsGy)
    {
        ScanBand(U, U.Gy, +1.0, B, "+Gy", E.ImpactSeconds, V.Findings, V.Impacts);
        ScanBand(U, U.Gy, -1.0, B, "-Gy", E.ImpactSeconds, V.Findings, V.Impacts);
    }
    for (const FGBand& B : E.PosGx)
    { ScanBand(U, U.Gx, +1.0, B, "+Gx", E.ImpactSeconds, V.Findings, V.Impacts); }
    for (const FGBand& B : E.NegGx)
    { ScanBand(U, U.Gx, -1.0, B, "-Gx", E.ImpactSeconds, V.Findings, V.Impacts); }

    // ONE EVENT IS ONE FINDING. The bands nest, so a single 5 g pull-out crosses
    // the 4 g, 3 g, 2 g and 1 g thresholds and arrives here as four findings for
    // one thing an author would fix once. Measured on a worked layout: seven
    // "impacts" that were two events.
    //
    // Nested runs share their peak SAMPLE exactly, so grouping on (axis, peak
    // time) is exact rather than a tolerance.
    //
    // Keep the highest THRESHOLD crossed, not the largest exceedance. For a
    // violation that names the highest G level whose allowance was blown — "5.0 g
    // held 3.2 s, and at that level you get 1 s" — which is the tightest true
    // constraint and the one to design against. Largest exceedance would instead
    // report every event against the 1 g band, which is true and useless.
    auto DedupeByEvent = [](std::vector<FGFinding>& List)
    {
        std::vector<FGFinding> Out;
        for (const FGFinding& F : List)
        {
            bool bMerged = false;
            for (FGFinding& O : Out)
            {
                if (std::string(O.Axis) == F.Axis
                    && std::fabs(O.AtTime - F.AtTime) < 1e-9)
                {
                    if (F.Limit > O.Limit) { O = F; }
                    bMerged = true;
                    break;
                }
            }
            if (!bMerged) { Out.push_back(F); }
        }
        List.swap(Out);
    };
    DedupeByEvent(V.Findings);
    DedupeByEvent(V.Impacts);

    std::sort(V.Impacts.begin(), V.Impacts.end(),
              [](const FGFinding& A, const FGFinding& B) { return A.Value > B.Value; });

    // COMBINED LOADING. You cannot sit at 100% of two axis limits at once, so the
    // check is a normalised ellipse rather than three independent tests — the
    // three axes above can each pass while the rider is outside the envelope.
    //
    // Evaluated against each axis's MOST PERMISSIVE band, which is the shortest
    // duration and therefore the loosest test this can make. Deliberately: a
    // per-window combined check wants a duration for the combination, which the
    // source does not define, and a stricter guess would manufacture violations.
    const double Dt = 1.0 / JudgeHz;
    auto Ceil = [](const std::vector<FGBand>& Bands) -> double
    { return Bands.empty() ? 0.0 : Bands.front().Threshold; };

    const double Lz = Ceil(E.PosGz), Lzn = Ceil(E.NegGz);
    const double Ly = Ceil(E.AbsGy);
    const double Lx = Ceil(E.PosGx), Lxn = Ceil(E.NegGx);

    double WorstE = 0.0;
    std::size_t WorstAt = 0;
    for (std::size_t i = 0; i < U.Gz.size(); ++i)
    {
        const double Rz = (U.Gz[i] >= 0.0) ? (Lz > 0.0 ? U.Gz[i] / Lz : 0.0)
                                           : (Lzn > 0.0 ? U.Gz[i] / Lzn : 0.0);
        const double Ry = (Ly > 0.0) ? U.Gy[i] / Ly : 0.0;
        const double Rx = (U.Gx[i] >= 0.0) ? (Lx > 0.0 ? U.Gx[i] / Lx : 0.0)
                                           : (Lxn > 0.0 ? U.Gx[i] / Lxn : 0.0);
        const double Ell = std::sqrt(Rz * Rz + Ry * Ry + Rx * Rx);
        if (Ell > WorstE) { WorstE = Ell; WorstAt = i; }
    }
    if (WorstE > 1.0)
    {
        FGFinding F;
        F.Kind = EGFindingKind::Combined;
        F.Axis = "combined";
        F.AtS = U.S[WorstAt];
        F.AtTime = U.T0 + static_cast<double>(WorstAt) * Dt;
        F.Value = WorstE;
        F.Limit = 1.0;
        V.Findings.push_back(F);
    }

    // JERK, over the least-squares window. Above the design rate is a NOTE and
    // above the allowable rate is a finding — two different statements, and
    // collapsing them would either cry wolf on every launch or miss a real snap.
    {
        const std::size_t Half =
            static_cast<std::size_t>(0.5 * E.JerkWindowSeconds * JudgeHz);
        struct FCh { const std::vector<double>* V; const char* Name; };
        const FCh Chs[] = {{&U.Gz, "+Gz"}, {&U.Gy, "+Gy"}, {&U.Gx, "+Gx"}};

        for (const FCh& C : Chs)
        {
            double Worst = 0.0;
            std::size_t At = 0;
            for (std::size_t i = 0; i < C.V->size(); ++i)
            {
                const double J = std::fabs(SlopeAt(*C.V, i, Half, Dt));
                if (J > Worst) { Worst = J; At = i; }
            }
            if (Worst <= E.JerkDesignGPerSec) { continue; }

            FGFinding F;
            F.Kind = EGFindingKind::Jerk;
            F.Axis = C.Name;
            F.AtS = U.S[At];
            F.AtTime = U.T0 + static_cast<double>(At) * Dt;
            F.Duration = E.JerkWindowSeconds;
            F.Value = Worst;
            F.Limit = (Worst > E.JerkAllowableGPerSec) ? E.JerkAllowableGPerSec
                                                       : E.JerkDesignGPerSec;
            if (Worst > E.JerkAllowableGPerSec) { V.Findings.push_back(F); }
            else                                { V.JerkNotes.push_back(F); }
        }
    }

    // Worst first: an author fixes the biggest exceedance, not the earliest.
    std::sort(V.Findings.begin(), V.Findings.end(),
              [](const FGFinding& A, const FGFinding& B)
              { return A.Exceedance() > B.Exceedance(); });

    V.bPasses = V.Findings.empty();
    return V;
}

// ponytail: reversal derating (FGEnvelope::ReversalMinSeconds/ReversalDerate) is
// carried in the envelope and NOT yet applied — it needs a sign-flip pass over
// each axis that halves the bands across the flip. Left as data rather than
// dropped, because the numbers are part of the table and losing them loses the
// intent. Add it when a layout is found that trips it; nothing measured so far
// reverses that fast.

// ===================== A SEAT OFF THE HEARTLINE =====================
//
// Felt G is computed at the heartline, and a rider who is not on it feels
// something else through every roll: the seat swings about the heartline at
// the roll rate, so it carries the heartline's acceleration PLUS the
// acceleration of that swing. For a seat a lateral distance y from the
// heartline (+y is the rider's left, the frame's own sign):
//
//   vertical:  alpha * y      the angular ACCELERATION lifts or drops the seat --
//                             the "snap" an outboard seat gets that the centre
//                             never does, and why a wing coaster feels the way it does
//   lateral:  -omega^2 * y    the centripetal pull back toward the heartline
//
// COASTER_TYPES.md argues this is worth building for every wide train rather
// than for wing coasters alone; what a four-across train's outer seat feels is
// the same term at a smaller y. Nothing in it is new data: RollRateDegPerSec
// is already on every sample, so this is a transform of a profile into the
// profile that seat would have recorded. Judge it with JudgeRideProfile like
// any other, which is the "run it once per seat" rule the facing sign follows.
//
// Signs follow the frame: Tangent x Lateral = Up, so a positive roll rate
// carries +Lateral toward +Up, and a seat at +y rises -- an upward seat
// acceleration presses the rider into it, which is +Gz. The centripetal term
// points at the heartline: for +y that is -Lateral, which the rider feels as a
// push toward their RIGHT (LateralG's positive sense), hence the sign below.
// The finite difference for alpha is per-sample and noisy; the 5 Hz filter
// the judge already applies is what makes it a measurement.
inline FRideProfile OffsetProfile(const FRideProfile& P, double LateralM)
{
    FRideProfile Out = P;
    if (Out.Samples.size() < 2 || LateralM == 0.0) { return Out; }
    const double G = 9.80665;
    const double ToRad = 3.14159265358979323846 / 180.0;
    for (std::size_t i = 0; i < Out.Samples.size(); ++i)
    {
        const std::size_t A = (i == 0) ? 0 : i - 1;
        const std::size_t B = (i + 1 < Out.Samples.size()) ? i + 1 : i;
        const double Dt = P.Samples[B].Time - P.Samples[A].Time;
        const double Omega = P.Samples[i].RollRateDegPerSec * ToRad;
        const double Alpha = (Dt > 1e-9)
            ? (P.Samples[B].RollRateDegPerSec - P.Samples[A].RollRateDegPerSec) * ToRad / Dt
            : 0.0;
        Out.Samples[i].VerticalG += Alpha * LateralM / G;
        Out.Samples[i].LateralG  += Omega * Omega * LateralM / G;
    }
    return Out;
}
