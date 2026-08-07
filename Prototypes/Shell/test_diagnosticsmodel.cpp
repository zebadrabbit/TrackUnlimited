// Asserts for DiagnosticsModel.h — the diagnostics panel's rules.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_diagnosticsmodel test_diagnosticsmodel.cpp && ./test_diagnosticsmodel

#include "DiagnosticsModel.h"

#include <cassert>
#include <cstdio>

namespace
{

FDiagTarget At(double S) { FDiagTarget T; T.S = S; return T; }
FDiagTarget Seg(int I) { FDiagTarget T; T.Segment = I; return T; }

void TestOrderingIsSeverityTHENLocation()
{
    // Not insertion order and not alphabetical. Somebody working on the first drop
    // wants the findings about the first drop TOGETHER, and a list ordered by
    // whichever check happened to run first scatters them across the panel.
    FDiagnostics D;
    D.Add(EDiagSeverity::Warning, "Geometry", "late warning", At(300.0));
    D.Add(EDiagSeverity::Error, "Geometry", "late error", At(400.0));
    D.Add(EDiagSeverity::Warning, "Geometry", "early warning", At(10.0));
    D.Add(EDiagSeverity::Error, "Geometry", "early error", At(20.0));
    D.Sort();

    assert(D.At(0).Text == "early error");
    assert(D.At(1).Text == "late error");
    assert(D.At(2).Text == "early warning");
    assert(D.At(3).Text == "late warning");
    std::printf("  errors before warnings, and within each, near before far\n");
}

void TestARowWithNOPlaceSortsLASTWithinItsSeverity()
{
    // A statement about the whole ride is CONTEXT rather than a next thing to go
    // and fix, so it does not sit between two findings that are.
    FDiagnostics D;
    D.Add(EDiagSeverity::Error, "Closure", "the track does not close");   // no place
    D.Add(EDiagSeverity::Error, "Geometry", "bad radius", At(120.0));
    D.Sort();

    assert(D.At(0).Text == "bad radius");
    assert(D.At(0).Target.HasPlace());
    assert(!D.At(1).Target.HasPlace());
    std::printf("  a whole-ride statement sorts after the findings you can go to\n");
}

void TestSortIsSTABLEAtTheSamePlace()
{
    // Two findings at one place keep the order the checks produced, which is the
    // order they depend on each other in — a radius warning after the "not finite"
    // error that caused it reads correctly; the other way round does not.
    FDiagnostics D;
    D.Add(EDiagSeverity::Error, "Geometry", "first", Seg(3));
    D.Add(EDiagSeverity::Error, "Geometry", "second", Seg(3));
    D.Add(EDiagSeverity::Error, "Geometry", "third", Seg(3));
    D.Sort();
    assert(D.At(0).Text == "first");
    assert(D.At(1).Text == "second");
    assert(D.At(2).Text == "third");
    std::printf("  two findings at one place keep the order the checks produced\n");
}

void TestHEIGHTIsItsOWNRowAndSaysWHICHWay()
{
    // THE ONE THAT ALREADY COST THIS PROJECT A RELEASE. The vertical slice shipped
    // 8.5 m low because plan view looked closed, and a closure gap reported as one
    // number hides exactly that.
    //
    // And "8.5 m apart" versus "ends 8.5 m LOW" are the same number, but only one
    // of them tells you what to change.
    FDiagnostics D;
    D.AddClosure(0.01, 0.01, -8.5, 0.05);
    assert(D.Num() == 1);                            // plan is fine, height is not
    assert(D.At(0).Text.find("LOW") != std::string::npos);
    assert(D.At(0).Text.find("8.50 m") != std::string::npos);

    FDiagnostics H;
    H.AddClosure(0.01, 0.01, 3.0, 0.05);
    assert(H.At(0).Text.find("HIGH") != std::string::npos);

    // Both wrong is two rows, not one combined magnitude.
    FDiagnostics B;
    B.AddClosure(4.0, 3.0, -8.5, 0.05);
    assert(B.Num() == 2);
    assert(B.At(0).Text.find("in plan") != std::string::npos);
    assert(B.At(0).Text.find("5.00 m") != std::string::npos);   // sqrt(16+9)
    assert(B.At(1).Text.find("LOW") != std::string::npos);

    // A closed track says nothing at all.
    FDiagnostics C;
    C.AddClosure(0.001, 0.001, 0.001, 0.05);
    assert(C.Num() == 0);
    std::printf("  a plan-closed track 8.5 m low gets its own row, and it says LOW\n");
}

void TestARideThatDoesNotCompleteHidesEveryDerivedNumber()
{
    // THE FAILURE THE ENVELOPE SUITE ALREADY HAD. It reported "within envelope,
    // zero findings" three times over a train that stalled at 46 m — a conformance
    // verdict on a ride that did not happen.
    //
    // So a stall is an ERROR, not a warning: a ride the train cannot finish is not
    // a ride with a problem, it is not a ride. And NOTHING derived from the run is
    // shown alongside it, because a top speed from a ride that did not happen is
    // worse than no number — it reads as a result.
    FDiagnostics D;
    D.AddRideProfile(false, 46.0, 12.0, 1.9, 30.0, 0.0);

    assert(D.Num() == 1);
    assert(D.HasErrors());
    assert(D.At(0).Severity == EDiagSeverity::Error);
    assert(D.At(0).Text.find("stalls at 46.00 m") != std::string::npos);
    assert(D.At(0).Target.HasPlace());               // and it takes you there

    for (std::size_t i = 0; i < D.Num(); ++i)
    {
        assert(D.At(i).Text.find("top speed") == std::string::npos);
        assert(D.At(i).Text.find("peak") == std::string::npos);
    }

    // A ride that DID complete gets its numbers, as information rather than as a
    // complaint — a panel that only ever appears when something is broken is one
    // people learn to dread.
    FDiagnostics G;
    G.AddRideProfile(true, 0.0, 26.4, 3.8, 310.0, 100.2);
    assert(!G.HasErrors());
    assert(G.Count(EDiagSeverity::Info) == 2);
    assert(G.At(0).Text.find("3.80 g at 310.00 m") != std::string::npos);
    assert(G.At(1).Text.find("95.0 km/h") != std::string::npos);
    std::printf("  a stalled ride shows the stall and NO derived numbers; a good one shows both\n");
}

void TestThereIsNowhereToPutAFIXITButton()
{
    // REPORT, NEVER REPAIR — IN THE UI TOO, and it is structural rather than a
    // policy somebody has to remember: a row has a place to GO and no action to
    // TAKE, so there is nowhere to wire one.
    //
    // Measured rather than principled. PHASE0_FINDINGS records that clamping a
    // degenerate arc to a straight yields a plausible 1.00 g and a clean
    // continuity pass — worse than leaving it visibly broken, because the ride
    // then looks correct and is not.
    FDiagnostics D;
    D.Add(EDiagSeverity::Error, "Geometry", "curvature implies a radius under 2 m", Seg(4));

    const FDiagRow& R = D.At(0);
    assert(R.Target.Segment == 4);
    // The struct is the assertion: it has Severity, Group, Text and Target, and
    // nothing that could be invoked. A row is COPYABLE and comparable as data,
    // which is what it means for it to carry no behaviour — if somebody adds a
    // callback it stops being that, and this line is what they have to argue with.
    const FDiagRow Copy = R;
    assert(Copy.Target.Segment == R.Target.Segment);
    assert(Copy.Text == R.Text);
    std::printf("  a row has a place to go and no action to take, structurally\n");
}

void TestSAVINGIsNeverBlocked()
{
    // The format tolerates work in progress and so does this. A validator that
    // refused to let somebody save an unfinished ride would teach them to work
    // somewhere else, and the one thing worse than an invalid file is no file.
    //
    // Present as a function rather than simply omitted, because somebody WILL
    // eventually wire a save button to HasErrors() and this is what makes them
    // read a sentence first.
    FDiagnostics D;
    D.Add(EDiagSeverity::Error, "Geometry", "not finite", Seg(0));
    D.Add(EDiagSeverity::Error, "Closure", "does not close");
    assert(D.HasErrors());
    assert(!D.BlocksSaving());
    std::printf("  two errors, and saving is still allowed\n");
}

void TestTheSUMMARYIsWhatMakesSomebodyOpenThePanel()
{
    // The panel is not always open, so the status bar has to carry enough to make
    // opening it the obvious next thing.
    FDiagnostics Empty;
    assert(Empty.Summary() == "no findings");

    FDiagnostics One;
    One.Add(EDiagSeverity::Error, "G", "x");
    assert(One.Summary() == "1 error");             // singular, because it reads

    FDiagnostics Many;
    Many.Add(EDiagSeverity::Error, "G", "x");
    Many.Add(EDiagSeverity::Error, "G", "y");
    Many.Add(EDiagSeverity::Warning, "G", "z");
    assert(Many.Summary() == "2 errors, 1 warning");

    // Info does not appear: a summary that said "0 errors, 0 warnings, 2 info"
    // would make a healthy ride look like it needed attention.
    FDiagnostics Good;
    Good.AddRideProfile(true, 0.0, 26.4, 3.8, 310.0, 100.2);
    assert(Good.Summary() == "no findings");
    std::printf("  \"%s\" — and a healthy ride says \"%s\"\n",
                Many.Summary().c_str(), Good.Summary().c_str());
}

} // namespace

int main()
{
    std::printf("The diagnostics panel: what it shows, and what it refuses to offer\n\n");

    TestOrderingIsSeverityTHENLocation();
    TestARowWithNOPlaceSortsLASTWithinItsSeverity();
    TestSortIsSTABLEAtTheSamePlace();
    TestHEIGHTIsItsOWNRowAndSaysWHICHWay();
    TestARideThatDoesNotCompleteHidesEveryDerivedNumber();
    TestThereIsNowhereToPutAFIXITButton();
    TestSAVINGIsNeverBlocked();
    TestTheSUMMARYIsWhatMakesSomebodyOpenThePanel();

    std::printf("\ntest_diagnosticsmodel: all assertions passed.\n");
    return 0;
}
