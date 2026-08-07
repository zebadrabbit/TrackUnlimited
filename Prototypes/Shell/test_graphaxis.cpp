// Asserts for GraphAxis.h — the ride-profile graph's arithmetic.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_graphaxis test_graphaxis.cpp && ./test_graphaxis

#include "GraphAxis.h"

#include <cassert>
#include <cstdio>
#include <vector>

namespace
{

bool IsNice(double Step)
{
    if (!(Step > 0.0)) { return false; }
    const double Exp = std::floor(std::log10(Step));
    const double F = Step / std::pow(10.0, Exp);
    return std::fabs(F - 1.0) < 1e-9 || std::fabs(F - 2.0) < 1e-9 || std::fabs(F - 5.0) < 1e-9;
}

void TestTicksAreNICENUMBERS()
{
    // Divide a range by a tick count and you get gridlines at 0.37, 0.74, 1.11.
    // A person cannot read a value off that.
    //
    // Every step must be 1, 2 or 5 times a power of ten, across ranges spanning
    // eleven orders of magnitude — because a graph is asked for speed in m/s and
    // curvature in 1/m on the same afternoon.
    for (double Span : {0.001, 0.037, 0.4, 1.0, 3.7, 26.4, 100.0, 543.7, 12000.0})
    {
        const FAxis A = MakeAxis(0.0, Span, 6);
        assert(IsNice(A.Step));
        assert(A.Max >= Span);                 // and the data still fits
        assert(A.TickCount() >= 2);
    }
    std::printf("  every axis step is 1, 2 or 5 times a power of ten\n");
}

void TestTheRANGEGrowsToAMultipleOfTheStep()
{
    // THE HALF PEOPLE LEAVE OUT. A nice step against a raw range puts the top
    // gridline somewhere arbitrary, which is exactly as unreadable as a nasty
    // step — the numbers down the side are 0, 2, 4, 6 and then the line at the
    // top has no label because it is at 6.8.
    const FAxis A = MakeAxis(0.0, 6.8, 5);
    assert(IsNice(A.Step));

    // Every tick, including the last, is an exact multiple of the step.
    for (std::size_t i = 0; i < A.TickCount(); ++i)
    {
        const double T = A.TickAt(i);
        const double Ratio = T / A.Step;
        assert(std::fabs(Ratio - std::floor(Ratio + 0.5)) < 1e-9);
    }
    assert(std::fabs(A.Max - A.TickAt(A.TickCount() - 1)) < 1e-9);
    std::printf("  the top gridline is a labelled multiple of the step, not the data's edge\n");
}

void TestZEROIsIncludedOnASIGNEDChannel()
{
    // On lateral G, roll rate or vertical G the zero line is what the trace means
    // anything against. A graph auto-scaled to 0.4..0.9 g looks like a mountain
    // range and is a train sitting almost still.
    const FAxis Signed = MakeAxis(0.4, 0.9, 6, /*bIncludeZero*/ true);
    assert(Signed.Min <= 0.0);
    assert(Signed.Max >= 0.9);

    // Negative data pulls the other way, and both ends are kept.
    const FAxis Both = MakeAxis(-2.1, 4.3, 6, true);
    assert(Both.Min <= -2.1 && Both.Max >= 4.3);
    assert(Both.Fraction(0.0) > 0.0 && Both.Fraction(0.0) < 1.0);   // zero is on screen

    // An UNSIGNED channel is allowed to crop. Speed over a lap sits between 8 and
    // 26 m/s and squashing it against a zero baseline throws away the detail the
    // graph exists for.
    const FAxis Speed = MakeAxis(8.0, 26.4, 6, /*bIncludeZero*/ false);
    assert(Speed.Min > 0.0);
    std::printf("  a signed channel keeps zero on screen; speed is allowed to crop\n");
}

void TestAFLATChannelDoesNotDivideByNothing()
{
    // Lateral G on a straight track is exactly zero for its whole length. A graph
    // that divided by that range would produce NaN — on the layout most likely to
    // be somebody's very first.
    const FAxis Flat = MakeAxis(0.0, 0.0, 6);
    assert(Flat.Span() > 0.0);
    assert(IsNice(Flat.Step));

    const double F = Flat.Fraction(0.0);
    assert(F == F);                                  // not NaN
    assert(F >= 0.0 && F <= 1.0);

    // And a non-zero flat channel — a train held at a constant 1 g — draws
    // through the middle rather than off the top.
    const FAxis Held = MakeAxis(1.0, 1.0, 6, false);
    assert(Held.Span() > 0.0);
    assert(Held.Fraction(1.0) > 0.1 && Held.Fraction(1.0) < 0.9);
    std::printf("  a dead-flat channel gets an axis rather than a division by zero\n");
}

void TestTheSCRUBBERRoundTrips()
{
    // The scrubber's own handle is drawn from the value it just produced, so a
    // mismatch between the two directions makes the handle drift away from the
    // cursor as you drag — which reads as the graph being laggy.
    FProfileGraph G;
    G.SetDomain(1288.0);

    for (double F : {0.0, 0.1, 0.25, 0.5, 0.75, 1.0})
    {
        const double S = G.ScrubToS(F);
        assert(std::fabs(G.SToScrub(S) - F) < 1e-12);
    }

    // CLAMPED, NOT WRAPPED. Dragging past the end of a circuit stops at the end;
    // wrapping would make the trace appear to teleport under the cursor.
    G.ScrubTo(-50.0);
    assert(G.ScrubbedS() == 0.0);
    G.ScrubTo(5000.0);
    assert(G.ScrubbedS() == 1288.0);
    assert(G.SToScrub(5000.0) == 1.0);

    // An empty document does not divide by its own length.
    FProfileGraph E;
    assert(E.SToScrub(10.0) == 0.0);
    assert(E.ScrubToS(0.5) == 0.0);
    std::printf("  the scrubber round-trips exactly and clamps at both ends\n");
}

void TestSamplingIsBYARCLENGTHAndInterpolated()
{
    // By arc length rather than by index, because an index lookup breaks the
    // moment the sample spacing changes — and the sample spacing is a setting.
    const std::vector<double> Trace{0.0, 10.0, 20.0, 30.0, 40.0};
    const double Total = 400.0;

    assert(FProfileGraph::SampleAt(Trace, Total, 0.0) == 0.0);
    assert(FProfileGraph::SampleAt(Trace, Total, 400.0) == 40.0);
    assert(std::fabs(FProfileGraph::SampleAt(Trace, Total, 200.0) - 20.0) < 1e-12);

    // INTERPOLATED, because the graph is continuous and the samples are not.
    // Nearest-neighbour makes the readout jump in steps as you drag, which reads
    // as the data being coarse rather than the lookup being lazy.
    assert(std::fabs(FProfileGraph::SampleAt(Trace, Total, 150.0) - 15.0) < 1e-12);
    assert(std::fabs(FProfileGraph::SampleAt(Trace, Total, 50.0) - 5.0) < 1e-12);

    // Past either end is the end value, not a wrap and not a crash.
    assert(FProfileGraph::SampleAt(Trace, Total, -10.0) == 0.0);
    assert(FProfileGraph::SampleAt(Trace, Total, 900.0) == 40.0);
    assert(FProfileGraph::SampleAt({}, Total, 100.0) == 0.0);
    assert(FProfileGraph::SampleAt({7.0}, Total, 100.0) == 7.0);
    std::printf("  sampling is by metres and interpolates, so dragging reads smoothly\n");
}

void TestEXTREMESCarryTheirLocations()
{
    // "4.25 g at S = 310 m" is somewhere to go and look. "4.25 g" is trivia —
    // the same rule the diagnostics panel runs on, and the reason both exist.
    std::vector<double> G(101, 1.0);
    G[20] = -0.3;                                    // airtime
    G[77] = 4.25;                                    // the pull-out
    const FChannelExtremes E = ExtremesOf(G, 500.0);

    assert(E.bValid);
    assert(std::fabs(E.Max - 4.25) < 1e-12);
    assert(std::fabs(E.MaxAtS - 385.0) < 1e-9);      // 77/100 of 500
    assert(std::fabs(E.Min + 0.3) < 1e-12);
    assert(std::fabs(E.MinAtS - 100.0) < 1e-9);

    assert(!ExtremesOf({}, 500.0).bValid);
    std::printf("  peak %.2f g at %.0f m, minimum %.2f g at %.0f m\n",
                E.Max, E.MaxAtS, E.Min, E.MinAtS);
}

void TestEachChannelGetsItsOWNAxis()
{
    // PER-CHANNEL SCALE, which the Phase 1 legibility card asked for. Speed in
    // m/s and roll rate in deg/s on one shared axis makes the speed trace a flat
    // line along the bottom.
    FGraphChannel Speed{"Speed", "m/s", false, MakeAxis(8.0, 26.4, 6, false)};
    FGraphChannel Roll{"Roll rate", "deg/s", true, MakeAxis(-33.2, 33.2, 6, true)};

    assert(Speed.Axis.Min > 0.0);                    // cropped, as it should be
    assert(Roll.Axis.Min < 0.0 && Roll.Axis.Max > 0.0);
    assert(Speed.Axis.Span() != Roll.Axis.Span());

    // Both traces use the full height of the plot, which is the point.
    assert(Speed.Axis.Fraction(26.4) > 0.8);
    assert(Roll.Axis.Fraction(33.2) > 0.8);
    std::printf("  speed and roll rate each fill the plot on their own axis\n");
}

} // namespace

int main()
{
    std::printf("The ride-profile graph, minus the drawing\n\n");

    TestTicksAreNICENUMBERS();
    TestTheRANGEGrowsToAMultipleOfTheStep();
    TestZEROIsIncludedOnASIGNEDChannel();
    TestAFLATChannelDoesNotDivideByNothing();
    TestTheSCRUBBERRoundTrips();
    TestSamplingIsBYARCLENGTHAndInterpolated();
    TestEXTREMESCarryTheirLocations();
    TestEachChannelGetsItsOWNAxis();

    std::printf("\ntest_graphaxis: all assertions passed.\n");
    return 0;
}
