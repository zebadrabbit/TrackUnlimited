// Fit RollingResistance and DragK to a recorded NoLimits 2 ride.
//
// Build & run (Windows):
//   clang++ -std=c++17 -Wall -Wextra -O2 -o calibrate.exe calibrate.cpp
//   ./calibrate.exe telemetry.csv
//
// PHASE0_FINDINGS records both coefficients as "tuning knobs, not
// measurements", and records a first fit that put DragK 3.2x above its
// physically derived value. The suspicion attached to that number was that the
// model had no train LENGTH, so the fit had nowhere else to put the
// discrepancy. Train length exists now, so the suspicion is testable: refit at
// several lengths and watch where DragK goes.
//
// The formulation matters. An earlier attempt searched over the two
// coefficients by running the whole ride forward, which is both slow and where
// its two segfaults came from. It is not necessary: for a coasting train the
// model is
//
//     a_resist = RollingResistance * (N * g)  +  DragK * v^2
//
// which is LINEAR in both unknowns. Every sample gives one equation, and the
// whole fit is a 2x2 normal-equation solve in closed form. No search, no
// iteration, nothing to diverge.
//
// Coasting samples are selected by the data rather than by detecting the lift
// and the brakes geometrically — which is what the earlier attempt got wrong,
// leaving a brake marker at 1e9 and reading off the end of an array. An
// interval is coasting if the resistance it implies is small and positive.
// A lift implies a large negative one and a brake a large positive one, so both
// exclude themselves.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr double G = 9.80665;

struct FSample
{
    double Speed = 0.0;
    double X = 0.0, Y = 0.0, Z = 0.0;
    double GMag = 0.0; // magnitude of NL2's own felt-G vector
    double ArcS = 0.0; // cumulative, derived from position rather than trusted
};

// The recorded path IS the track, sampled. Height against arc length is all the
// energy accounting needs, and deriving it from the positions avoids depending
// on the imported CSV the recording was matched against.
std::vector<FSample> Read(const char* Path, std::string& OutError)
{
    std::vector<FSample> Out;
    std::ifstream File(Path);
    if (!File)
    {
        OutError = "cannot open ";
        OutError += Path;
        return Out;
    }

    std::string Line;
    bool bHeader = true;
    while (std::getline(File, Line))
    {
        if (bHeader)
        {
            bHeader = false;
            continue;
        }
        std::istringstream Row(Line);
        std::vector<std::string> Cell;
        std::string Field;
        while (std::getline(Row, Field, '\t'))
        {
            Cell.push_back(Field);
        }
        if (Cell.size() < 10)
        {
            continue;
        }
        // Column 2 is `onride`: samples taken while not on the ride carry no
        // usable state and their positions jump.
        if (std::atoi(Cell[2].c_str()) == 0)
        {
            continue;
        }

        FSample S;
        S.Speed = std::atof(Cell[3].c_str());
        S.X = std::atof(Cell[4].c_str());
        S.Y = std::atof(Cell[5].c_str());
        S.Z = std::atof(Cell[6].c_str());
        const double Gx = std::atof(Cell[7].c_str());
        const double Gy = std::atof(Cell[8].c_str());
        const double Gz = std::atof(Cell[9].c_str());
        S.GMag = std::sqrt(Gx * Gx + Gy * Gy + Gz * Gz);
        Out.push_back(S);
    }

    // Arc length from the chord between consecutive samples. At 60-odd Hz and
    // coaster speeds the chord under-measures a curve by well under a
    // millimetre per step, which is far below anything this fit can resolve.
    for (std::size_t i = 1; i < Out.size(); ++i)
    {
        const double Dx = Out[i].X - Out[i - 1].X;
        const double Dy = Out[i].Y - Out[i - 1].Y;
        const double Dz = Out[i].Z - Out[i - 1].Z;
        Out[i].ArcS = Out[i - 1].ArcS + std::sqrt(Dx * Dx + Dy * Dy + Dz * Dz);
    }
    return Out;
}

// Height of the train's centre of mass: the mean height over a window of
// TrainLength centred on this sample. With TrainLength = 0 it is the sample's
// own height and every number below collapses to the point-mass fit.
double MeanHeight(const std::vector<FSample>& S, std::size_t Index, double TrainLength)
{
    if (!(TrainLength > 0.0))
    {
        return S[Index].Z;
    }
    const double Half = TrainLength * 0.5;
    const double Lo = S[Index].ArcS - Half;
    const double Hi = S[Index].ArcS + Half;

    double Sum = 0.0;
    int Count = 0;
    for (std::size_t i = 0; i < S.size(); ++i)
    {
        if (S[i].ArcS >= Lo && S[i].ArcS <= Hi)
        {
            Sum += S[i].Z;
            ++Count;
        }
    }
    return Count > 0 ? Sum / Count : S[Index].Z;
}

struct FFit
{
    double RollingResistance = 0.0;
    double DragK = 0.0;
    int Samples = 0;
    double RmsAccelResidual = 0.0; // m/s^2
    double RmsSpeedError = 0.0;    // m/s, integrating the fit through each coasting run

    // Whether the two coefficients are separable AT ALL on this data. Rolling
    // resistance scales with normal load and drag with v^2; if those two move
    // together across the recording, no amount of fitting can say which one is
    // responsible for the loss, and the split between them is arbitrary.
    double PredictorCorrelation = 0.0;
    double ConditionNumber = 0.0;

    // Crr alone, with DragK pinned at its physically derived value. One
    // parameter against well-conditioned data, which is the answer this
    // recording can actually support.
    double CrrWithDragPinned = 0.0;
    double PinnedRmsSpeedError = 0.0;
};

FFit Fit(const std::vector<FSample>& S, double TrainLength, double MinSpeed, double MaxResist)
{
    FFit R;
    if (S.size() < 3)
    {
        return R;
    }

    // Precompute the centre-of-mass height once per sample: MeanHeight is a
    // linear scan, so doing it inside the interval loop would be quadratic
    // twice over.
    std::vector<double> MeanZ(S.size());
    for (std::size_t i = 0; i < S.size(); ++i)
    {
        MeanZ[i] = MeanHeight(S, i, TrainLength);
    }

    // a_resist = Crr*(N*g) + DragK*(v^2), one equation per interval.
    double Sxx = 0.0, Sxy = 0.0, Syy = 0.0, Sxa = 0.0, Sya = 0.0;
    std::vector<std::size_t> Kept;

    for (std::size_t i = 1; i < S.size(); ++i)
    {
        const double Ds = S[i].ArcS - S[i - 1].ArcS;
        if (!(Ds > 0.05))
        {
            continue;
        }
        const double V0 = S[i - 1].Speed;
        const double V1 = S[i].Speed;
        if (V0 < MinSpeed || V1 < MinSpeed)
        {
            continue;
        }

        // Energy over the interval, with gravity read off the centre of mass.
        const double DKinetic = 0.5 * (V1 * V1 - V0 * V0);
        const double DHeight = MeanZ[i] - MeanZ[i - 1];
        const double AResist = -(DKinetic + G * DHeight) / Ds;

        // Self-selecting: a lift implies a large NEGATIVE resistance (something
        // is adding energy) and a brake a large positive one. Only coasting
        // lands in between, so neither has to be detected geometrically.
        if (!(AResist > 0.0) || AResist > MaxResist)
        {
            continue;
        }

        const double VMean = 0.5 * (V0 + V1);
        const double X = 0.5 * (S[i - 1].GMag + S[i].GMag) * G; // normal load term
        const double Y = VMean * VMean;                         // drag term

        Sxx += X * X;
        Sxy += X * Y;
        Syy += Y * Y;
        Sxa += X * AResist;
        Sya += Y * AResist;
        Kept.push_back(i);
    }

    R.Samples = static_cast<int>(Kept.size());
    double Det = Sxx * Syy - Sxy * Sxy;
    if (R.Samples < 10 || std::fabs(Det) < 1e-12)
    {
        return R;
    }
    R.RollingResistance = (Sxa * Syy - Sya * Sxy) / Det;
    R.DragK = (Sxx * Sya - Sxy * Sxa) / Det;

    // ===================== REJECT THE BRAKE RAMP =====================
    //
    // MaxResist correctly throws out a brake at 8 m/s^2. It does NOT throw out
    // the ramp INTO one, which climbs through 0.4, 0.6, 1.2 on its way there and
    // is under the threshold for every one of those. Those samples sit at the LOW
    // end of the v^2 range carrying resistance several times the coast's, so they
    // lift the intercept and flatten the slope: measured on a 142 km/h run that
    // braked at the end, Crr came out 0.0277 against a true 0.02603 and DragK
    // 0.000086 against 0.000100. The pinned-DragK fit went NEGATIVE, which is the
    // symptom that should have given it away.
    //
    // A tighter fixed threshold is not the fix — it is the same fragility with a
    // different number in it, and the right ceiling depends on how fast the ride
    // is. So the data sets its own: fit, measure the spread of the residuals, and
    // drop whatever sits far outside it. A brake ramp is a huge outlier against a
    // coast that fits to 0.04%; ordinary coasting noise is not.
    //
    // MEDIAN ABSOLUTE DEVIATION rather than a standard deviation, because the
    // contamination being removed is exactly what would inflate a standard
    // deviation and hide itself.
    for (int Pass = 0; Pass < 3; ++Pass)
    {
        std::vector<double> Res;
        Res.reserve(Kept.size());
        for (std::size_t i : Kept)
        {
            const double Ds = S[i].ArcS - S[i - 1].ArcS;
            const double VMean = 0.5 * (S[i - 1].Speed + S[i].Speed);
            const double AResist =
                -(0.5 * (S[i].Speed * S[i].Speed - S[i - 1].Speed * S[i - 1].Speed)
                  + G * (MeanZ[i] - MeanZ[i - 1])) / Ds;
            const double X = 0.5 * (S[i - 1].GMag + S[i].GMag) * G;
            Res.push_back(std::fabs(AResist
                - (R.RollingResistance * X + R.DragK * VMean * VMean)));
        }

        std::vector<double> Sorted = Res;
        std::nth_element(Sorted.begin(), Sorted.begin() + Sorted.size() / 2, Sorted.end());
        const double Median = Sorted[Sorted.size() / 2];
        // 6 median deviations. Generous on purpose: this is here to remove a
        // brake, not to polish a fit, and a trim that reshapes clean data would
        // be manufacturing the answer rather than finding it.
        const double Limit = std::max(6.0 * Median, 1e-6);

        std::vector<std::size_t> Next;
        Sxx = Sxy = Syy = Sxa = Sya = 0.0;
        for (std::size_t k = 0; k < Kept.size(); ++k)
        {
            if (Res[k] > Limit) { continue; }
            const std::size_t i = Kept[k];
            const double Ds = S[i].ArcS - S[i - 1].ArcS;
            const double VMean = 0.5 * (S[i - 1].Speed + S[i].Speed);
            const double AResist =
                -(0.5 * (S[i].Speed * S[i].Speed - S[i - 1].Speed * S[i - 1].Speed)
                  + G * (MeanZ[i] - MeanZ[i - 1])) / Ds;
            const double X = 0.5 * (S[i - 1].GMag + S[i].GMag) * G;
            const double Y = VMean * VMean;
            Sxx += X * X; Sxy += X * Y; Syy += Y * Y;
            Sxa += X * AResist; Sya += Y * AResist;
            Next.push_back(i);
        }
        if (Next.size() == Kept.size() || Next.size() < 10) { break; }

        const double D2 = Sxx * Syy - Sxy * Sxy;
        if (std::fabs(D2) < 1e-12) { break; }
        Kept.swap(Next);
        R.Samples = static_cast<int>(Kept.size());
        R.RollingResistance = (Sxa * Syy - Sya * Sxy) / D2;
        R.DragK = (Sxx * Sya - Sxy * Sxa) / D2;
        Det = D2;
    }

    // Is the split between the two even meaningful on this recording?
    R.PredictorCorrelation = Sxy / std::sqrt(Sxx * Syy);
    // Eigenvalues of the symmetric 2x2 normal matrix; their ratio is how much
    // the fit amplifies noise along the badly determined direction.
    const double Trace = Sxx + Syy;
    const double Root = std::sqrt(std::fmax(0.0, Trace * Trace - 4.0 * Det));
    const double Lo = 0.5 * (Trace - Root);
    const double Hi = 0.5 * (Trace + Root);
    R.ConditionNumber = Lo > 0.0 ? Hi / Lo : 0.0;

    // The one-parameter fit: pin drag at the value the physics gives and let
    // rolling resistance take the rest.
    const double PhysicalDragK = 0.00045;
    double Num = 0.0, Den = 0.0;
    for (const std::size_t i : Kept)
    {
        const double Ds = S[i].ArcS - S[i - 1].ArcS;
        const double DKinetic = 0.5 * (S[i].Speed * S[i].Speed - S[i - 1].Speed * S[i - 1].Speed);
        const double AResist = -(DKinetic + G * (MeanZ[i] - MeanZ[i - 1])) / Ds;
        const double VMean = 0.5 * (S[i - 1].Speed + S[i].Speed);
        const double X = 0.5 * (S[i - 1].GMag + S[i].GMag) * G;
        Num += X * (AResist - PhysicalDragK * VMean * VMean);
        Den += X * X;
    }
    R.CrrWithDragPinned = Den > 0.0 ? Num / Den : 0.0;

    // Residual in the quantity actually fitted.
    double SumSq = 0.0;
    for (const std::size_t i : Kept)
    {
        const double Ds = S[i].ArcS - S[i - 1].ArcS;
        const double DKinetic = 0.5 * (S[i].Speed * S[i].Speed - S[i - 1].Speed * S[i - 1].Speed);
        const double AResist = -(DKinetic + G * (MeanZ[i] - MeanZ[i - 1])) / Ds;
        const double VMean = 0.5 * (S[i - 1].Speed + S[i].Speed);
        const double Predicted = R.RollingResistance * 0.5 * (S[i - 1].GMag + S[i].GMag) * G
                               + R.DragK * VMean * VMean;
        SumSq += (AResist - Predicted) * (AResist - Predicted);
    }
    R.RmsAccelResidual = std::sqrt(SumSq / R.Samples);

    // And the number a human can judge: integrate the fitted model through each
    // contiguous coasting run from its own recorded start speed, and see how far
    // the predicted speed drifts from the recorded one.
    double SpeedSq = 0.0;
    int SpeedCount = 0;
    double V = 0.0;
    std::size_t Prev = 0;
    for (const std::size_t i : Kept)
    {
        if (i != Prev + 1 || V <= 0.0)
        {
            V = S[i - 1].Speed; // start of a new run: reset to the recording
        }
        const double Ds = S[i].ArcS - S[i - 1].ArcS;
        const double A = R.RollingResistance * 0.5 * (S[i - 1].GMag + S[i].GMag) * G
                       + R.DragK * V * V;
        double Next = V * V - 2.0 * G * (MeanZ[i] - MeanZ[i - 1]) - 2.0 * A * Ds;
        V = Next > 0.0 ? std::sqrt(Next) : 0.0;
        SpeedSq += (V - S[i].Speed) * (V - S[i].Speed);
        ++SpeedCount;
        Prev = i;
    }
    R.RmsSpeedError = SpeedCount > 0 ? std::sqrt(SpeedSq / SpeedCount) : 0.0;
    return R;
}

} // namespace

int main(int argc, char** argv)
{
    const char* Path = argc > 1 ? argv[1] : "telemetry.csv";
    std::string Error;
    const std::vector<FSample> S = Read(Path, Error);
    if (!Error.empty())
    {
        std::printf("%s\n", Error.c_str());
        return 1;
    }
    if (S.size() < 100)
    {
        std::printf("only %zu on-ride samples in %s; not enough to fit\n", S.size(), Path);
        return 1;
    }

    double Lowest = S[0].Z, Highest = S[0].Z, TopSpeed = 0.0;
    for (const FSample& Sm : S)
    {
        Lowest = Sm.Z < Lowest ? Sm.Z : Lowest;
        Highest = Sm.Z > Highest ? Sm.Z : Highest;
        TopSpeed = Sm.Speed > TopSpeed ? Sm.Speed : TopSpeed;
    }
    std::printf("%s: %zu on-ride samples, %.1f m of track, %.1f m of drop, top %.1f km/h\n\n",
                Path, S.size(), S.back().ArcS, Highest - Lowest, TopSpeed * 3.6);

    std::printf("Fitting a_resist = Crr*(N*g) + DragK*v^2 over coasting intervals only.\n");
    std::printf("PHASE0_FINDINGS records the shipped defaults as Crr 0.006, DragK 0.00045,\n");
    std::printf("and DragK's physically derived value for a loaded 7-car steel train as\n");
    std::printf("0.00045. The question is whether train length moves the fit toward it.\n\n");

    std::printf("%12s %10s %11s %9s %13s %10s %8s\n", "train length", "Crr", "DragK",
                "samples", "resid m/s^2", "corr(x,y)", "cond");
    for (const double Length : {0.0, 5.0, 10.0, 15.0, 20.0, 25.0})
    {
        const FFit F = Fit(S, Length, 5.0, 2.0);
        if (F.Samples < 10)
        {
            std::printf("%10.1f m %10s %11s %9d %13s %10s %8s\n", Length, "-", "-", F.Samples,
                        "-", "-", "-");
            continue;
        }
        std::printf("%10.1f m %10.5f %11.6f %9d %13.4f %10.4f %8.0f\n", Length,
                    F.RollingResistance, F.DragK, F.Samples, F.RmsAccelResidual,
                    F.PredictorCorrelation, F.ConditionNumber);
    }

    const FFit Ref = Fit(S, 0.0, 5.0, 2.0);

    // THE VERDICT IS DERIVED, NOT PRINTED FROM A SCRIPT. This used to end with a
    // fixed paragraph saying the split was arbitrary and a faster ride was
    // needed — true of the 44.5 km/h recording it was written against, and a flat
    // lie the moment a 142 km/h one arrived and separated them cleanly. A tool
    // that states its conclusion regardless of its input is worse than one that
    // states nothing.
    //
    // The test that actually distinguishes the two cases is NOT corr(x,y): on a
    // dead-flat coast N*g is very nearly constant, so its correlation with
    // anything is numerically meaningless and reads near 1 whatever the data
    // says. What matters is how much of the loss DRAG accounts for, and whether
    // that share VARIES across the recording — a term stuck at 5% everywhere
    // cannot be told from the constant beside it, and one running 38% down to 22%
    // plainly can.
    const double VTop = TopSpeed;
    const double VLo = std::max(5.0, 0.4 * TopSpeed);
    const double DragTop = Ref.DragK * VTop * VTop;
    const double DragLo = Ref.DragK * VLo * VLo;
    const double LossTop = Ref.RollingResistance * G + DragTop;
    const double LossLo = Ref.RollingResistance * G + DragLo;
    const double ShareTop = LossTop > 0.0 ? DragTop / LossTop : 0.0;
    const double ShareLo = LossLo > 0.0 ? DragLo / LossLo : 0.0;
    const bool bSeparable = ShareTop > 0.15 && (ShareTop - ShareLo) > 0.05;

    std::printf("\nSeparability. Drag accounts for %.0f%% of the loss at %.0f km/h and %.0f%%\n"
                "at %.0f km/h.\n",
                ShareTop * 100.0, VTop * 3.6, ShareLo * 100.0, VLo * 3.6);

    if (bSeparable)
    {
        std::printf("That share is both LARGE and VARYING across this recording, so the fit\n");
        std::printf("can tell the two apart. Corroborate it two ways before believing it:\n");
        std::printf("DragK should hold steady down the train-length column (it has nowhere\n");
        std::printf("else to go if it is real), and fitting the fast and slow halves of the\n");
        std::printf("coast separately should give the same pair.\n");
        std::printf("    RollingResistance = %.5f\n", Ref.RollingResistance);
        std::printf("    DragK             = %.6f\n", Ref.DragK);
    }
    else
    {
        std::printf("That is too small a share, too flat across the run, for any fit to say\n");
        std::printf("which coefficient the loss belongs to — the split between them is a free\n");
        std::printf("parameter here, and the sign flips down the DragK column are noise along\n");
        std::printf("a direction the data does not constrain. Separating them needs a FASTER\n");
        std::printf("ride; drag scales with v^2 and there is very little of it here.\n");
        std::printf("\nWhat this recording CAN support: pin DragK and fit rolling resistance\n");
        std::printf("alone, which is one parameter against well-conditioned data.\n");
        std::printf("    RollingResistance = %.5f   (DragK pinned at 0.00045)\n",
                    Ref.CrrWithDragPinned);
    }

    std::printf("\nCoasting is selected by the data: an interval counts when the resistance\n");
    std::printf("it implies is positive and under 2 m/s^2, so lifts and brakes exclude\n");
    std::printf("themselves rather than being detected geometrically. THE RAMP INTO a brake\n");
    std::printf("does not — it climbs through that window on its way up — so a robust trim\n");
    std::printf("drops whatever sits more than six median deviations off the fit.\n");
    return 0;
}
