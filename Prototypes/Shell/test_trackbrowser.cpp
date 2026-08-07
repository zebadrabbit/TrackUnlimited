// Asserts for TrackBrowser.h — the recent list and the browser's rules.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_trackbrowser test_trackbrowser.cpp && ./test_trackbrowser

#include "TrackBrowser.h"

#include <cassert>
#include <cstdio>

namespace
{

FTrackEntry Good(const std::string& Path, double L, double H)
{
    FTrackEntry E;
    E.Path = Path;
    E.Name = FTrackBrowser::FileNameOf(Path);
    E.LengthM = L;
    E.HeightM = H;
    return E;
}

void TestRecentIsMOSTRECENTFirstAndDeduplicated()
{
    FTrackBrowser B;
    B.Touch("C:/rides/a.tutrack");
    B.Touch("C:/rides/b.tutrack");
    B.Touch("C:/rides/c.tutrack");
    assert(B.NumRecent() == 3);
    assert(B.RecentAt(0) == "C:/rides/c.tutrack");

    // Re-opening promotes rather than duplicating.
    B.Touch("C:/rides/a.tutrack");
    assert(B.NumRecent() == 3);
    assert(B.RecentAt(0) == "C:/rides/a.tutrack");
    std::printf("  most recent first, and re-opening promotes rather than duplicating\n");
}

void TestDEDUPIsCaseAndSeparatorInsensitive()
{
    // Windows-first, and a path opened through a file dialog, a recent entry and a
    // command line can be three spellings of one file. Dedup on the raw string
    // shows it three times, and the list looks broken to the one person who would
    // have noticed.
    FTrackBrowser B;
    B.Touch("C:/Rides/Thing.tutrack");
    B.Touch("c:\\rides\\thing.tutrack");
    B.Touch("C:/Rides/Thing.tutrack/");
    assert(B.NumRecent() == 1);

    // AND THE STORED SPELLING IS THE ONE JUST USED. If somebody opened it through
    // a different route, that is the route they will recognise in the list.
    assert(B.RecentAt(0) == "C:/Rides/Thing.tutrack/");
    std::printf("  three spellings of one path are one entry, spelled the way it was last opened\n");
}

void TestDEDUPHappensBEFORETheCap()
{
    // The other order pushes a file off the end and then re-adds it, which loses
    // whatever was at the bottom for no reason at all.
    FTrackBrowser B;
    B.SetMaxRecent(3);
    B.Touch("a");
    B.Touch("b");
    B.Touch("c");
    B.Touch("a");                                  // already present: promote

    assert(B.NumRecent() == 3);
    assert(B.RecentAt(0) == "a");
    assert(B.RecentAt(1) == "c");
    assert(B.RecentAt(2) == "b");                  // still there, correctly

    // A genuinely new one does evict the oldest.
    B.Touch("d");
    assert(B.NumRecent() == 3);
    assert(B.RecentAt(2) == "c");
    std::printf("  promoting an entry never evicts anything; a new one evicts the oldest\n");
}

void TestAMISSINGFileSTAYSInTheList()
{
    // Dropping a recent entry because the path does not resolve is wrong on the
    // commonest case: an external drive that is not plugged in, a share that is
    // down, a folder that got renamed. Silently pruning it means somebody's
    // history disappears for reasons that have nothing to do with them.
    //
    // It stays, marked, and is still clickable — because "reconnect the drive and
    // click it again" only works if it is still there to click.
    FTrackBrowser B;
    B.Touch("D:/usb/holiday.tutrack");
    B.Touch("C:/rides/thing.tutrack");

    const std::vector<FTrackEntry> Known{Good("C:/rides/thing.tutrack", 1288.0, 46.0)};
    const std::vector<FTrackEntry> Rows = FTrackBrowser::Rows(Known, B.RecentList());

    // Rows follow the recent list's order, which is most-recent-first — so the
    // one touched second is on top and the missing one is below it.
    assert(Rows.size() == 2);                      // NOTHING is omitted
    assert(!Rows[0].bMissing && Rows[0].Name == "thing");
    assert(Rows[1].bMissing && Rows[1].Name == "holiday");
    assert(B.NumRecent() == 2);                    // and the list is untouched

    // The row says which kind of problem it is, because one is "plug the drive
    // back in" and the other is "line 12 is wrong".
    assert(FTrackBrowser::Subtitle(Rows[1]).find("not found") != std::string::npos);
    assert(FTrackBrowser::Subtitle(Rows[1]).find("disconnected") != std::string::npos);

    // Removing is EXPLICIT, and the only way an entry leaves.
    B.Forget("d:\\usb\\holiday.tutrack");          // and it normalises here too
    assert(B.NumRecent() == 1);
    std::printf("  a missing file stays in the list, marked, and only a person removes it\n");
}

void TestAFileThatWillNotLoadSTILLGetsARowWithItsREASON()
{
    // THE CARD'S EXPLICIT ASK. The format is hand-editable by design, so the
    // browser WILL meet a file somebody edited and got wrong — and the loader
    // already refuses an unknown segment kind WITH A REASON. The whole value of
    // that reason is lost if the UI silently returns to the menu.
    FTrackEntry Broken;
    Broken.Path = "C:/rides/typo.tutrack";
    Broken.Name = "typo";
    Broken.Error = "segment 12: unknown kind \"clothiod\"";

    const std::vector<FTrackEntry> Rows =
        FTrackBrowser::Rows({Broken}, {"C:/rides/typo.tutrack"});

    assert(Rows.size() == 1);
    assert(!Rows[0].bMissing);                     // it is there, it is just wrong
    assert(!Rows[0].IsUsable());
    assert(FTrackBrowser::Subtitle(Rows[0]) == "segment 12: unknown kind \"clothiod\"");

    // AND NO PLAUSIBLE NOTHING. A broken file showing "0.0 m, 0.0 m tall" reads
    // as a real empty track rather than as a file that would not parse.
    assert(FTrackBrowser::Subtitle(Rows[0]).find("0.0 m") == std::string::npos);
    std::printf("  a file that will not load says \"%s\"\n",
                FTrackBrowser::Subtitle(Rows[0]).c_str());
}

void TestAGoodRowShowsWhatSomebodyIsChoosingBetween()
{
    // Length and height, because that is what distinguishes two tracks in a list
    // at a glance and the name usually does not.
    const FTrackEntry E = Good("C:/rides/circuit.tutrack", 1288.0, 46.6);
    assert(E.IsUsable());
    assert(FTrackBrowser::Subtitle(E) == "1288.0 m, 46.6 m tall");
    std::printf("  a good row reads \"%s\"\n", FTrackBrowser::Subtitle(E).c_str());
}

void TestTheNameDropsTheFolderAndONEExtension()
{
    // "Reference" and not "C:/rides/Reference.tutrack" thirty times down the left
    // of the list.
    assert(FTrackBrowser::FileNameOf("C:/rides/Reference.tutrack") == "Reference");
    assert(FTrackBrowser::FileNameOf("C:\\rides\\Reference.tutrack") == "Reference");
    assert(FTrackBrowser::FileNameOf("Reference.tutrack") == "Reference");
    assert(FTrackBrowser::FileNameOf("Reference") == "Reference");

    // ONE extension. A name with a dot in it keeps the rest of itself.
    assert(FTrackBrowser::FileNameOf("v1.2 draft.tutrack") == "v1.2 draft");
    assert(FTrackBrowser::FileNameOf(".hidden") == ".hidden");
    std::printf("  the list shows \"Reference\", and \"v1.2 draft\" keeps its dot\n");
}

void TestTheCAPAppliesOnLOADAsWellAsOnTouch()
{
    // A recent file hand-edited to two hundred entries should not produce a menu
    // two hundred rows long — and somebody WILL edit it, because every file this
    // project writes is meant to be readable.
    FTrackBrowser B;
    B.SetMaxRecent(3);

    std::string File;
    for (int i = 0; i < 200; ++i) { File += "C:/rides/" + std::to_string(i) + ".tutrack\n"; }
    B.LoadRecent(File);
    assert(B.NumRecent() == 3);
    assert(B.RecentAt(0) == "C:/rides/0.tutrack");

    // Blank lines and stray whitespace do not become entries.
    B.LoadRecent("C:/a.tutrack\n\n   \nC:/b.tutrack  \n");
    assert(B.NumRecent() == 2);
    assert(B.RecentAt(1) == "C:/b.tutrack");

    // And it round-trips.
    FTrackBrowser C;
    C.LoadRecent(B.SaveRecent());
    assert(C.NumRecent() == 2);
    assert(C.RecentAt(0) == B.RecentAt(0));
    std::printf("  a 200-line recent file yields three rows, and the list round-trips\n");
}

} // namespace

int main()
{
    std::printf("The track browser: the recent list, and what a bad file looks like\n\n");

    TestRecentIsMOSTRECENTFirstAndDeduplicated();
    TestDEDUPIsCaseAndSeparatorInsensitive();
    TestDEDUPHappensBEFORETheCap();
    TestAMISSINGFileSTAYSInTheList();
    TestAFileThatWillNotLoadSTILLGetsARowWithItsREASON();
    TestAGoodRowShowsWhatSomebodyIsChoosingBetween();
    TestTheNameDropsTheFolderAndONEExtension();
    TestTheCAPAppliesOnLOADAsWellAsOnTouch();

    std::printf("\ntest_trackbrowser: all assertions passed.\n");
    return 0;
}
