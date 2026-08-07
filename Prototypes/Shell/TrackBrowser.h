// TrackUnlimited Phase 3.5: the track browser and the recent list.
// Plain C++17, no engine dependency. No filesystem either — paths and metadata
// come in from the caller, because WHERE the files are is the engine's business
// and WHAT to do about them is not.
//
// ===================== THE FILE FORMAT IS HAND-EDITABLE BY DESIGN =====================
//
// Which means the browser will meet a file somebody edited by hand and got wrong.
// The loader already refuses an unknown segment kind WITH A REASON, and the whole
// value of that reason is lost if the UI silently returns to the menu.
//
// So a file that will not load still gets a ROW, carrying its error. Anything
// else trains people to believe the application is broken rather than their file.
//
// ===================== A MISSING FILE IS NOT A GONE FILE =====================
//
// Dropping a recent entry because the path does not resolve is wrong on the
// commonest case: an external drive that is not plugged in, a network share that
// is down, a folder that got renamed. Silently pruning it means somebody's
// history disappears for reasons that have nothing to do with them.
//
// It stays, marked, and is still clickable — because "reconnect the drive and
// click it again" only works if it is still there to click.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

// What the browser knows about one track, however it found out.
struct FTrackEntry
{
    std::string Path;
    std::string Name;

    // Read from the file. Zero on anything that would not load, and the row says
    // why rather than showing a plausible nothing.
    double LengthM = 0.0;
    double HeightM = 0.0;
    long long ModifiedUnix = 0;

    // NOT FOUND is a state, not an absence. See above.
    bool bMissing = false;

    // WHY IT WILL NOT LOAD, verbatim from the loader. Empty means it loads.
    std::string Error;

    bool IsUsable() const { return !bMissing && Error.empty(); }
};

class FTrackBrowser
{
public:
    // ===================== THE RECENT LIST =====================
    //
    // Most recent first, deduplicated, capped. All three are obvious and the
    // combination is where it goes wrong: dedup by raw string means the same file
    // appears twice with different capitalisation, and capping before dedup drops
    // an entry that was about to be promoted.
    void Touch(const std::string& Path)
    {
        if (Path.empty()) { return; }
        const std::string Key = Normalise(Path);

        // DEDUP FIRST, THEN CAP. The other order can push a file off the end and
        // then re-add it, which loses whatever was at the bottom for no reason.
        for (std::size_t i = 0; i < Recent.size(); ++i)
        {
            if (Normalise(Recent[i]) == Key)
            {
                // PROMOTED, and the stored spelling is UPDATED to the one just
                // used — if somebody opened it through a different route, that is
                // the route they will recognise in the list.
                Recent.erase(Recent.begin() + static_cast<long>(i));
                break;
            }
        }
        Recent.insert(Recent.begin(), Path);
        while (Recent.size() > MaxRecent) { Recent.pop_back(); }
    }

    // Removing is EXPLICIT, and it is the only way an entry leaves the list.
    // Nothing prunes automatically — see the missing-file note above.
    void Forget(const std::string& Path)
    {
        const std::string Key = Normalise(Path);
        for (std::size_t i = 0; i < Recent.size(); ++i)
        {
            if (Normalise(Recent[i]) == Key)
            {
                Recent.erase(Recent.begin() + static_cast<long>(i));
                return;
            }
        }
    }
    void ForgetAll() { Recent.clear(); }

    std::size_t NumRecent() const { return Recent.size(); }
    const std::string& RecentAt(std::size_t i) const { return Recent[i]; }
    const std::vector<std::string>& RecentList() const { return Recent; }

    void SetMaxRecent(std::size_t N) { MaxRecent = N > 0 ? N : 1; }

    // Persisted as one path per line. Blank lines skipped, and the CAP IS APPLIED
    // ON LOAD as well — a settings file hand-edited to two hundred entries should
    // not produce a menu two hundred rows long.
    void LoadRecent(const std::string& Text)
    {
        Recent.clear();
        std::size_t Pos = 0;
        while (Pos < Text.size() && Recent.size() < MaxRecent)
        {
            std::size_t End = Text.find('\n', Pos);
            if (End == std::string::npos) { End = Text.size(); }
            std::string Line = Text.substr(Pos, End - Pos);
            Pos = End + 1;
            while (!Line.empty() && (Line.back() == '\r' || Line.back() == ' ')) { Line.pop_back(); }
            if (!Line.empty()) { Recent.push_back(Line); }
        }
    }

    std::string SaveRecent() const
    {
        std::string Out;
        for (const std::string& P : Recent) { Out += P + "\n"; }
        return Out;
    }

    // ===================== BUILDING THE VIEW =====================
    //
    // The caller supplies what it managed to read for each path — including the
    // failures. This decides what the list looks like, and the one rule is that
    // NOTHING IS OMITTED: a file that is missing or broken is a row that says so.
    static std::vector<FTrackEntry> Rows(const std::vector<FTrackEntry>& Known,
                                         const std::vector<std::string>& Paths)
    {
        std::vector<FTrackEntry> Out;
        for (const std::string& P : Paths)
        {
            bool bFound = false;
            for (const FTrackEntry& K : Known)
            {
                if (Normalise(K.Path) == Normalise(P))
                {
                    Out.push_back(K);
                    bFound = true;
                    break;
                }
            }
            if (!bFound)
            {
                // The caller could not read it at all. That is MISSING rather than
                // an error in the file, and the difference matters: one is "plug
                // the drive back in" and the other is "line 12 is wrong".
                FTrackEntry E;
                E.Path = P;
                E.Name = FileNameOf(P);
                E.bMissing = true;
                Out.push_back(E);
            }
        }
        return Out;
    }

    // What the row says under the name. Deliberately assembled here rather than in
    // the widget, so the missing and broken cases cannot be forgotten by whichever
    // template somebody writes.
    static std::string Subtitle(const FTrackEntry& E)
    {
        if (E.bMissing) { return "not found — the drive or folder may be disconnected"; }
        if (!E.Error.empty()) { return E.Error; }
        return Metres(E.LengthM) + ", " + Metres(E.HeightM) + " tall";
    }

    static std::string FileNameOf(const std::string& Path)
    {
        std::size_t Cut = std::string::npos;
        for (std::size_t i = 0; i < Path.size(); ++i)
        {
            if (Path[i] == '/' || Path[i] == '\\') { Cut = i; }
        }
        std::string Name = Cut == std::string::npos ? Path : Path.substr(Cut + 1);
        // Strip one extension, so the list shows "Reference" and not
        // "Reference.tutrack" thirty times down the left-hand side.
        const std::size_t Dot = Name.rfind('.');
        if (Dot != std::string::npos && Dot > 0) { Name = Name.substr(0, Dot); }
        return Name;
    }

private:
    // CASE AND SEPARATORS BOTH, because this is Windows-first and a path opened
    // through a file dialog, a recent entry and a command line can be three
    // spellings of one file. Dedup on the raw string shows it three times.
    static std::string Normalise(const std::string& In)
    {
        std::string Out = In;
        for (char& C : Out)
        {
            if (C == '\\') { C = '/'; }
            if (C >= 'A' && C <= 'Z') { C = static_cast<char>(C - 'A' + 'a'); }
        }
        while (!Out.empty() && (Out.back() == '/' || Out.back() == ' ')) { Out.pop_back(); }
        return Out;
    }

    static std::string Metres(double V)
    {
        const long long Tenths = static_cast<long long>(V * 10.0 + 0.5);
        return std::to_string(Tenths / 10) + "." + std::to_string(Tenths % 10) + " m";
    }

    std::vector<std::string> Recent;
    std::size_t MaxRecent = 10;
};

// ponytail: no thumbnails and no folder tree. A thumbnail wants a render on save,
// which is a real feature with a real cost and belongs on its own card; a folder
// tree is what the OS file dialog is for, and the browser's job is the handful of
// tracks somebody actually works on.
