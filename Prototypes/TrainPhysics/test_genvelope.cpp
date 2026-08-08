// Asserts for GEnvelope.h — judging a ride profile against acceleration envelopes.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_genvelope test_genvelope.cpp && ./test_genvelope
//
// Most of these build a SYNTHETIC profile rather than running a train, because
// the thing under test is the judging and a synthetic profile can hold exactly
// 4.5 g for exactly 2.0 seconds. The reference layout is run at the end, which
// is the case that matters: a real ride, judged.

#include "GEnvelope.h"
#include "../TrackSpline/TrackSpline.h"

#include <string>

#include <cassert>
#include <cstdio>

namespace
{

// A profile holding fixed G values for a given time, at a nominal 20 m/s so arc
// length and time are related the way a real run relates them.
FRideProfile Hold(double Gz, double Gy, double Gx, double Seconds,
                  double SampleHz = 200.0)
{
    FRideProfile P;
    const int N = static_cast<int>(Seconds * SampleHz);
    for (int i = 0; i <= N; ++i)
    {
        FRideSample S;
        S.Time = static_cast<double>(i) / SampleHz;
        S.S = S.Time * 20.0;
        S.Speed = 20.0;
        S.VerticalG = Gz;
        S.LateralG = Gy;
        S.TangentialG = Gx;
        P.Samples.push_back(S);
    }
    P.bCompleted = true;
    P.Duration = Seconds;
    return P;
}

// Flat 1 g, a RAMPED rise to Gz, a plateau, a ramped fall, flat 1 g again.
//
// The ramp is not decoration. A square edge is an infinite jerk, and the judge
// is right to fail it — the first version of this fixture stepped 1 g to 5 g in
// one sample, which is 40 g/s against a 15 g/s allowable, and every "should
// pass" case failed on a jerk finding that had nothing to do with what was being
// tested. Real track cannot do that; a fixture that can is testing the fixture.
//
// Ramp defaults to 0.5 s, so a 4 g rise is 8 g/s — under the 10 g/s design rate.
// Pass 0.0 for a genuinely sharp event when that is the point (impacts).
FRideProfile Pulse(double Gz, double PlateauSeconds, double RampSeconds = 0.5,
                   double PadSeconds = 3.0, double SampleHz = 200.0)
{
    FRideProfile P;
    const double Total = PadSeconds * 2.0 + RampSeconds * 2.0 + PlateauSeconds;
    const int N = static_cast<int>(Total * SampleHz);
    const double R0 = PadSeconds;
    const double P0 = R0 + RampSeconds;
    const double P1 = P0 + PlateauSeconds;
    const double R1 = P1 + RampSeconds;

    for (int i = 0; i <= N; ++i)
    {
        const double T = static_cast<double>(i) / SampleHz;
        double G = 1.0;
        if (T >= P0 && T < P1)                       { G = Gz; }
        else if (RampSeconds > 0.0 && T >= R0 && T < P0)
        { G = 1.0 + (Gz - 1.0) * (T - R0) / RampSeconds; }
        else if (RampSeconds > 0.0 && T >= P1 && T < R1)
        { G = Gz - (Gz - 1.0) * (T - P1) / RampSeconds; }

        FRideSample S;
        S.Time = T;
        S.S = T * 20.0;
        S.Speed = 20.0;
        S.VerticalG = G;
        P.Samples.push_back(S);
    }
    P.bCompleted = true;
    P.Duration = Total;
    return P;
}

int CountKind(const std::vector<FGFinding>& F, EGFindingKind K)
{
    int N = 0;
    for (const FGFinding& X : F) { if (X.Kind == K) { ++N; } }
    return N;
}

bool HasAxis(const std::vector<FGFinding>& F, const char* Axis)
{
    for (const FGFinding& X : F)
    {
        if (std::string(X.Axis) == Axis) { return true; }
    }
    return false;
}

// One axis, one kind. Most tests below want THIS rather than bPasses: a test of
// the duration bands should not be able to fail because the fixture's jerk or
// its combined loading tripped something else. Assert the thing under test.
bool HasSustained(const FGVerdict& V, const char* Axis)
{
    for (const FGFinding& X : V.Findings)
    {
        if (X.Kind == EGFindingKind::Sustained && std::string(X.Axis) == Axis)
        {
            return true;
        }
    }
    return false;
}

const FGFinding& FirstSustained(const FGVerdict& V, const char* Axis)
{
    for (const FGFinding& X : V.Findings)
    {
        if (X.Kind == EGFindingKind::Sustained && std::string(X.Axis) == Axis)
        {
            return X;
        }
    }
    assert(false && "no sustained finding on that axis");
    return V.Findings.front();
}

// ---------------------------------------------------------------------------

// A ride that never leaves 1 g is the null case, and it has to come out clean —
// a checker that fails everything is as useless as one that passes everything.
void TestLevelRidePasses()
{
    const FGVerdict V = JudgeRideProfile(Hold(1.0, 0.0, 0.0, 10.0));
    assert(V.bPasses);
    assert(V.Findings.empty());
    assert(V.SamplesJudged > 0);
    std::printf("  level ride: %zu samples judged, clean\n", V.SamplesJudged);
}

// THE CENTRAL CLAIM: the same value passes or fails depending on how long it is
// held. If this pair does not split, the whole thing is a peak detector wearing
// a standard's name.
void TestDurationIsTheWholePoint()
{
    // 5 g. Held briefly, the run above the 4 g band is inside its 1 s allowance.
    const FGVerdict Short = JudgeRideProfile(Pulse(5.0, 0.3));
    assert(!HasSustained(Short, "+Gz"));

    // The same 5 g held longer is not. Same value, different verdict — if this
    // pair does not split, the whole thing is a peak detector.
    const FGVerdict Long = JudgeRideProfile(Pulse(5.0, 3.0));
    assert(HasSustained(Long, "+Gz"));

    const FGFinding& F = FirstSustained(Long, "+Gz");
    assert(F.Duration > 1.0);
    assert(F.Value > 4.9 && F.Value < 5.2);
    std::printf("  5 g briefly passes; 5 g held fails "
                "(%.2f g for %.2f s against a %.2f g band)\n",
                F.Value, F.Duration, F.Limit);
}

// The inverted band form is the part most likely to be "corrected" into
// something generous, so pin the case the header comment works through: at
// 5.9 g, a duration landing in the 1-3 s band must fail against the 4 g limit.
void TestInvertedBandsAgreeWithPublishedForm()
{
    const FGVerdict V = JudgeRideProfile(Pulse(5.9, 2.9));
    assert(HasSustained(V, "+Gz"));
    assert(FirstSustained(V, "+Gz").Duration > 1.0);

    // And the same magnitude inside the 1 s allowance does not.
    const FGVerdict Ok = JudgeRideProfile(Pulse(5.9, 0.3));
    assert(!HasSustained(Ok, "+Gz"));
    std::printf("  5.9 g: 2.9 s fails the 4 g band, 0.3 s does not — "
                "bands inverted correctly\n");
}

// An impact is NOT a short violation. A sharp spike has to leave the sustained
// pass/fail entirely, or every curvature joint in every layout fails.
void TestImpactsAreNotViolations()
{
    // Ramp 0, so it is genuinely sharp — which is what an impact is. Big enough
    // that it still clears the 6 g ceiling AFTER filtering: a 5 Hz low-pass takes
    // most of the height off anything this brief, which is the filter working.
    const FGVerdict V = JudgeRideProfile(Pulse(15.0, 0.15, 0.0));
    assert(!HasSustained(V, "+Gz"));         // not a sustained violation
    assert(!V.Impacts.empty());              // but it IS reported
    assert(V.Impacts.front().Kind == EGFindingKind::Impact);
    assert(V.Impacts.front().Duration < 0.20);
    std::printf("  15 g for 0.15 s: reported as an impact, not a sustained "
                "violation (%.2f g filtered, %.3f s)\n",
                V.Impacts.front().Value, V.Impacts.front().Duration);
}

// The other half of the impact rule, and the one that stops it being noise: a
// short excursion that would NOT have violated its band even if sustained is not
// worth reporting at all. Level track wobbling either side of 1 g is the case
// that matters, because it is every ride ever authored.
void TestSmallExcursionsAreNotImpacts()
{
    const FGVerdict V = JudgeRideProfile(Pulse(1.05, 0.10, 0.0));
    assert(V.Impacts.empty());
    assert(V.Findings.empty());
    std::printf("  1.05 g for 0.10 s: not reported at all — a wobble over the "
                "1 g band is not an impact\n");
}

// Airtime has its own, much tighter, curve — and it is on the MAGNITUDE of a
// negative number, which is the sign slip worth guarding.
void TestNegativeVerticalIsJudgedOnMagnitude()
{
    // -0.4 g of gentle airtime, held: legal.
    const FGVerdict Gentle = JudgeRideProfile(Hold(-0.4, 0.0, 0.0, 6.0));
    assert(Gentle.bPasses);

    // -1.5 g held for 6 s: past the 1.1 g / 3 s allowance.
    const FGVerdict Hard = JudgeRideProfile(Hold(-1.5, 0.0, 0.0, 6.0));
    assert(!Hard.bPasses);
    assert(HasAxis(Hard.Findings, "-Gz"));
    assert(Hard.Findings.front().Value > 1.4);   // magnitude, not the signed value
    std::printf("  -1.5 g for 6 s fails on -Gz at magnitude %.2f g\n",
                Hard.Findings.front().Value);
}

// Both lateral directions, because one routine serves both via a sign and a
// copy-paste that tested only the positive one would pass.
void TestLateralBothWays()
{
    const FGVerdict L = JudgeRideProfile(Hold(1.0, 2.8, 0.0, 6.0));
    const FGVerdict R = JudgeRideProfile(Hold(1.0, -2.8, 0.0, 6.0));
    assert(!L.bPasses && !R.bPasses);
    assert(HasAxis(L.Findings, "+Gy"));
    assert(HasAxis(R.Findings, "-Gy"));
    std::printf("  lateral fails both ways: +Gy and -Gy\n");
}

// Braking is -Gx and a launch is +Gx, and they have different allowances.
void TestLongitudinalAxis()
{
    const FGVerdict Brake = JudgeRideProfile(Hold(1.0, 0.0, -2.5, 6.0));
    assert(!Brake.bPasses);
    assert(HasAxis(Brake.Findings, "-Gx"));

    // A launch at 1.8 g held 6 s is inside the 2.0 g band.
    const FGVerdict Launch = JudgeRideProfile(Hold(1.0, 0.0, 1.8, 6.0));
    assert(Launch.bPasses);
    std::printf("  -2.5 g braking fails; +1.8 g launch passes\n");
}

// COMBINED LOADING: three axes each individually legal, together outside the
// envelope. This is the test that would pass if the ellipse were dropped and
// three independent checks kept, which is exactly why it is here.
void TestCombinedLoadingCatchesWhatAxesMiss()
{
    // 3.5 g vertical (under the 6 g ceiling), 2.4 g lateral (under 3.0),
    // 1.5 g fore-aft (under 6.0). Held briefly enough that no single band trips.
    const FGEnvelope E = FGEnvelope::Harmonised();
    const FRideProfile P = Hold(3.5, 2.4, 1.5, 0.6);
    const FGVerdict V = JudgeRideProfile(P, E);

    // No axis is individually in violation at this duration...
    assert(CountKind(V.Findings, EGFindingKind::Sustained) == 0);
    // ...and the combination still is.
    assert(CountKind(V.Findings, EGFindingKind::Combined) == 1);
    assert(!V.bPasses);

    for (const FGFinding& F : V.Findings)
    {
        if (F.Kind == EGFindingKind::Combined)
        {
            assert(F.Value > 1.0);
            std::printf("  combined loading: ellipse %.3f > 1 with every axis "
                        "individually legal\n", F.Value);
        }
    }
}

// The filter has to be zero-phase or the reported location is wrong, which is
// the whole product. Put a pulse at a known time and check it is found there.
void TestFilterDoesNotShiftTheLocation()
{
    const double Pad = 4.0;
    const double Ramp = 0.5;
    const FGVerdict V = JudgeRideProfile(Pulse(5.0, 2.0, Ramp, Pad));
    assert(HasSustained(V, "+Gz"));

    // The worst point must sit inside the event, not lag past its end. A
    // single-pass IIR would push it downstream by a good fraction of a second,
    // and the whole product is "go and look at S = this".
    const FGFinding& F = FirstSustained(V, "+Gz");
    const double Begin = Pad + Ramp;
    const double End = Begin + 2.0;
    assert(F.AtTime > Begin - 0.25);
    assert(F.AtTime < End + 0.25);
    std::printf("  plateau t=[%.2f, %.2f] reported at t=%.2f s (S=%.1f m) — "
                "no phase lag\n", Begin, End, F.AtTime, F.AtS);
}

// The filter is part of the measurement standard, not a smoothing nicety. Prove
// it is load-bearing: the same single-sample artefact must reach the verdict
// unfiltered and be gone once filtered.
void TestFilterIsLoadBearing()
{
    // A one-sample 12 g spike on an otherwise flat 1 g ride — an integrator
    // artefact, not something a rider could feel.
    FRideProfile P = Hold(1.0, 0.0, 0.0, 6.0);
    P.Samples[600].VerticalG = 12.0;

    const FGVerdict Filtered = JudgeRideProfile(P);

    FGEnvelope Off = FGEnvelope::Harmonised();
    Off.FilterHz = 45.0;              // effectively unfiltered at 100 Hz judging
    const FGVerdict Raw = JudgeRideProfile(P, Off);

    auto Worst = [](const std::vector<FGFinding>& F)
    {
        double W = 0.0;
        for (const FGFinding& X : F) { if (X.Value > W) { W = X.Value; } }
        return W;
    };

    // Unfiltered, the artefact reads as a large-G event. Filtered at 5 Hz it is
    // smeared out to a fraction of a g — still visible as a small impact, which
    // is honest, but nowhere near the bands it originally tripped.
    //
    // Note it does NOT vanish, and asserting that it did was wrong: a 12 g
    // one-sample spike carries real area, and a low-pass spreads it rather than
    // deleting it. What the filter buys is that the spread version cannot
    // masquerade as a 12 g event.
    assert(Worst(Raw.Impacts) > 6.0);
    assert(Worst(Filtered.Impacts) < 3.0);
    std::printf("  one-sample 12 g artefact: reads %.1f g unfiltered, "
                "%.2f g filtered at 5 Hz\n",
                Worst(Raw.Impacts), Worst(Filtered.Impacts));
}

// MUTATION: break the band rule and the suite must notice. A check that passes
// whatever the data says is worse than no check.
void TestBandsActuallyBite()
{
    FGEnvelope Loose = FGEnvelope::Harmonised();
    for (FGBand& B : Loose.PosGz) { B.MaxSeconds += 100.0; }

    const FRideProfile P = Pulse(5.0, 3.0);
    assert(HasSustained(JudgeRideProfile(P), "+Gz"));          // real table
    assert(!HasSustained(JudgeRideProfile(P, Loose), "+Gz"));  // loosened
    std::printf("  loosening the +Gz durations makes a failing ride pass — "
                "the bands are doing the work\n");
}

// The verdict must carry what it did NOT check. A conformance result that hides
// its own gaps is worse than no verdict, so this is asserted rather than trusted.
void TestVerdictDeclaresItsGaps()
{
    const FGVerdict A = JudgeRideProfile(Hold(1.0, 0.0, 0.0, 3.0),
                                         FGEnvelope::Harmonised(EGStandard::ASTM_F2291));
    const FGVerdict E = JudgeRideProfile(Hold(1.0, 0.0, 0.0, 3.0),
                                         FGEnvelope::Harmonised(EGStandard::EN_13814));

    assert(!A.NotApplied.empty() && !E.NotApplied.empty());
    assert(A.NotApplied.find("Push-pull") != std::string::npos);
    assert(!A.MeasurementNote.empty() && !E.MeasurementNote.empty());
    // EN measures nearer head level and this model does not offset for it. The
    // verdict has to say so, or it claims a conformance it did not check.
    assert(E.MeasurementNote.find("does NOT offset") != std::string::npos);
    assert(A.MeasurementNote != E.MeasurementNote);
    std::printf("  verdict declares its gaps, and the two standards say "
                "different things about where they measure\n");
}

// ---------------------------------------------------------------------------
// A REAL RIDE. The reference layout, judged. No assertion on the verdict itself
// — the layout was authored to be rideable, not to be conformant, and asserting
// either way would be asserting a design decision. What IS asserted is that the
// judge ran over real data and produced locatable findings.

// Pitch curvature does the hills; MakeStraight then edited, since the helpers
// only cover yaw.
FTrackSegment MakeHill(double Length, double PitchStart, double PitchEnd)
{
    FTrackSegment Seg = MakeStraight(Length);
    Seg.PitchCurvatureStart = PitchStart;
    Seg.PitchCurvatureEnd = PitchEnd;
    return Seg;
}

FTrack BuildReferenceish()
{
    // A short stand-in for the reference layout: lift, crest, drop, pull-out,
    // banked turn. Enough real curvature to exercise the judge without depending
    // on the full 23-segment list, which lives in the actor.
    //
    // EVERY PITCH CHANGE IS A PAIR — k ramps 0 -> k then k -> 0 — so the angle
    // arrives and then STOPS arriving. The first draft of this used single
    // segments and the track climbed monotonically to 104 m: pitch curvature is
    // an angular RATE, so a segment that ends at non-zero curvature is still
    // pitching up when the next one starts.
    FTrack Track;
    Track.AddSegment(MakeStraight(26.0));               // station
    Track.AddSegment(MakeHill(20.0, 0.0, 0.020));       // ease into the climb
    Track.AddSegment(MakeHill(20.0, 0.020, 0.0));       // ... arriving at +0.4 rad
    Track.AddSegment(MakeStraight(60.0));               // the climb itself, 23 deg
    Track.AddSegment(MakeHill(20.0, 0.0, -0.020));      // ease over
    Track.AddSegment(MakeHill(20.0, -0.020, 0.0));      // ... level at the crest
    Track.AddSegment(MakeHill(25.0, 0.0, -0.024));      // into the drop
    Track.AddSegment(MakeHill(25.0, -0.024, 0.0));      // ... -0.6 rad, 34 deg down
    Track.AddSegment(MakeStraight(20.0));               // the drop
    Track.AddSegment(MakeHill(25.0, 0.0, 0.024));       // pull out — the +Gz event
    Track.AddSegment(MakeHill(25.0, 0.024, 0.0));       // ... back to level
    Track.AddSegment(MakeArc(60.0, 50.0, 0.6));         // banked left turn
    Track.AddSegment(MakeStraight(60.0));               // run out
    return Track;
}

void TestRealRideIsJudgedAndLocatable()
{
    const FTrack Track = BuildReferenceish();

    FTrainConfig Cfg;
    Cfg.TrainLength = 15.0;
    FTrain Train(Track, Cfg);
    // 5 m/s^2 of chain, because gravity down a 23 deg grade is 3.8 and a lift
    // that cannot out-pull it stalls. The first version of this test used 1.0 and
    // the train stopped at S=46 m.
    assert(Train.AddZone(MakeLift(0.0, 166.0, 4.0, 5.0)));

    const FRideProfile P = RunRideProfile(Train, Track);
    const FGVerdict V = JudgeRideProfile(P);

    // THE ASSERTION THAT MAKES THE REST MEAN ANYTHING. A stalled train produces a
    // short, flat, entirely conformant profile and a clean verdict — which is a
    // VACUOUS PASS, and this suite reported one for its first three runs: 1.02 g
    // peak, zero findings, "within envelope", on a train that crawled 46 m up a
    // hill it could not climb and stopped. Judging a ride that did not happen is
    // the one failure mode a conformance tool must not have.
    assert(P.bCompleted);
    assert(P.TopSpeed > 15.0);          // it actually got moving
    assert(P.MaxVerticalG > 2.0);       // and the pull-out actually pulled
    assert(V.SamplesJudged > 100);

    std::printf("\n  --- reference-ish layout, %.1f m, %.1f s ---\n",
                Track.TotalLength(), P.Duration);
    std::printf("  completed, top speed %.1f m/s (%.0f km/h), %.1f m of drop\n",
                P.TopSpeed, P.TopSpeed * 3.6, P.HighestHeight - P.LowestHeight);
    std::printf("  peak +Gz %.2f g at S=%.1f m | min %.2f g | max |Gy| %.2f g\n",
                P.MaxVerticalG, P.MaxVerticalGS, P.MinVerticalG, P.MaxAbsLateralG);
    std::printf("  verdict: %s, %d finding(s), %d impact(s), %d jerk note(s)\n",
                V.bPasses ? "within envelope" : "OUTSIDE ENVELOPE",
                static_cast<int>(V.Findings.size()),
                static_cast<int>(V.Impacts.size()),
                static_cast<int>(V.JerkNotes.size()));

    // Every finding has to be somewhere an author can GO. That is the difference
    // between this and a number, and it is worth an assertion.
    for (const FGFinding& F : V.Findings)
    {
        assert(F.AtS >= 0.0 && F.AtS <= Track.TotalLength() + 1.0);
        std::printf("    %-9s %.2f g for %.2f s at S=%.1f m (t=%.2f s), "
                    "limit %.2f, over by %.2f\n",
                    F.Axis, F.Value, F.Duration, F.AtS, F.AtTime, F.Limit,
                    F.Exceedance());
    }
    for (const FGFinding& F : V.Impacts)
    {
        assert(F.AtS >= 0.0 && F.AtS <= Track.TotalLength() + 1.0);
    }
}

} // namespace

// ===================== A BACKWARD-FACING SEAT =====================
//
// The fore-aft bands are asymmetric 3:1 — 6 g eyeballs-in with a headrest
// against 2 g back-to-chest — so which way the seat points is not cosmetic. It
// decides which band a braking event is judged against.
void TestABackwardFacingSeatFlipsFOREAFTAndLATERALButNotVERTICAL()
{
    const FTrack Track = BuildReferenceish();

    FTrainConfig Cfg;
    Cfg.TrainLength = 15.0;

    FTrain Fwd(Track, Cfg);
    assert(Fwd.AddZone(MakeLift(0.0, 166.0, 4.0, 5.0)));
    const FRideProfile F = RunRideProfile(Fwd, Track, 1.0, 1.0 / 240.0, +1.0);

    FTrain Rev(Track, Cfg);
    assert(Rev.AddZone(MakeLift(0.0, 166.0, 4.0, 5.0)));
    const FRideProfile R = RunRideProfile(Rev, Track, 1.0, 1.0 / 240.0, -1.0);

    assert(F.bCompleted && R.bCompleted);
    assert(F.Samples.size() == R.Samples.size());

    // EXACTLY NEGATED, not approximately — it is a sign, not a model.
    for (std::size_t i = 0; i < F.Samples.size(); ++i)
    {
        assert(R.Samples[i].TangentialG == -F.Samples[i].TangentialG);
        assert(R.Samples[i].LateralG == -F.Samples[i].LateralG);
        // VERTICAL IS UNTOUCHED. Up is up either way, and flipping it would put
        // a reversed rider in permanent negative G — which would be caught by
        // the envelope instantly, so the failure worth asserting against is the
        // one that would NOT be.
        assert(R.Samples[i].VerticalG == F.Samples[i].VerticalG);
        assert(R.Samples[i].Speed == F.Samples[i].Speed);
        assert(R.Samples[i].Height == F.Samples[i].Height);
    }
    std::printf("\n  a reversed seat: Gx and Gy exactly negated over %zu samples, Gz identical\n",
                F.Samples.size());
}

// AND THE VERDICT CHANGES, which is the whole reason the flip matters.
//
// A synthetic profile rather than authored geometry, because the point is the
// JUDGEMENT — hunting for a layout that lands between the two verdicts would be
// a test about track design, and it would break the first time anybody retuned
// the drag coefficient.
//
// RAMPED, NOT STEPPED. The first version stepped from 0 to 3 g between samples,
// which is 27 g/s of jerk against a 15 g/s limit — so both facings failed on the
// edge and the test proved nothing about facing at all. The ramp is not a
// convenience; it is what makes the assertion about the thing it names.
void TestTheSAMEEventIsJudgedDifferentlyByFacing()
{
    auto Build = [](double Gx)
    {
        FRideProfile P;
        P.bCompleted = true;
        for (int i = 0; i <= 400; ++i)
        {
            FRideSample S;
            S.Time = i * 0.01;
            S.S = i * 0.1;
            S.Speed = 20.0;
            S.VerticalG = 1.0;      // resting on the seat, as a real ride is
            S.LateralG = 0.0;

            const double t = S.Time;
            double k = 0.0;
            if (t >= 0.5 && t < 1.0)       { k = (t - 0.5) / 0.5; }
            else if (t >= 1.0 && t <= 3.0) { k = 1.0; }
            else if (t > 3.0 && t < 3.5)   { k = 1.0 - (t - 3.0) / 0.5; }
            S.TangentialG = Gx * k;
            P.Samples.push_back(S);
        }
        P.Duration = P.Samples.back().Time;
        return P;
    };

    const FGVerdict In = JudgeRideProfile(Build(+3.0));
    const FGVerdict Out = JudgeRideProfile(Build(-3.0));

    std::printf("  3 g for 2 s alongside 1 g of seat load:"
                " eyeballs-in passes=%d, back-to-chest passes=%d\n",
                In.bPasses ? 1 : 0, Out.bPasses ? 1 : 0);
    assert(In.bPasses);
    assert(!Out.bPasses);

    // AND IT IS THE *COMBINED* BAND THAT BITES, not the fore-aft one, which is
    // worth recording because it is not what this test was written expecting.
    // The two fore-aft tables are only asymmetric ABOVE 6 g — `PosGx` carries a
    // hard ceiling there and `NegGx` carries none at all — so at 3 g they treat
    // both directions identically. What separates them is the combined-loading
    // check, where 3 g of back-to-chest alongside 1 g of seat load lands outside
    // the envelope and the same magnitude of eyeballs-in does not.
    bool bCombined = false;
    for (const FGFinding& F : Out.Findings)
    {
        if (std::string(F.Axis) == "combined") { bCombined = true; }
    }
    assert(bCombined);

    // NOTED, NOT FIXED: `NegGx` having no hard ceiling means a very large
    // back-to-chest spike under 4 s is currently unreported. That may be a gap
    // in the table or may be what the standard says — the tables are UNVERIFIED
    // research and this project does not author safety limits from guesswork.
    // Recorded here so a verified table has something to answer.
}

int main()
{
    std::printf("GEnvelope: judging a ride against acceleration envelopes\n");
    std::printf("NOTE: limit tables are UNVERIFIED research, not checked against\n"
                "      the published ASTM F2291 / EN 13814 documents.\n\n");

    TestLevelRidePasses();
    TestDurationIsTheWholePoint();
    TestInvertedBandsAgreeWithPublishedForm();
    TestImpactsAreNotViolations();
    TestSmallExcursionsAreNotImpacts();
    TestNegativeVerticalIsJudgedOnMagnitude();
    TestLateralBothWays();
    TestLongitudinalAxis();
    TestCombinedLoadingCatchesWhatAxesMiss();
    TestFilterDoesNotShiftTheLocation();
    TestFilterIsLoadBearing();
    TestBandsActuallyBite();
    TestVerdictDeclaresItsGaps();
    TestRealRideIsJudgedAndLocatable();
    TestABackwardFacingSeatFlipsFOREAFTAndLATERALButNotVERTICAL();
    TestTheSAMEEventIsJudgedDifferentlyByFacing();

    std::printf("\nAll GEnvelope assertions passed.\n");
    return 0;
}
