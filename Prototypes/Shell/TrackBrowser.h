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

    // THE PLAN VIEW, interleaved x,y in [0,1], ready to draw. See PlanThumb, and
    // the note at the foot of this file about why it is not a rendered picture.
    // Empty on anything that would not load, which is the same rule the numbers
    // above follow — a row shows its reason rather than a plausible nothing.
    std::vector<float> Plan;

    bool IsUsable() const { return !bMissing && Error.empty(); }
};

// ===================== A TRACK IS ITS OWN THUMBNAIL =====================
//
// Takes a plan-view path as interleaved x,y in metres and returns it fitted to
// the unit box, ready for a widget to scale into whatever rectangle it has.
//
// ASPECT RATIO IS PRESERVED, which is the entire point rather than a nicety: an
// out-and-back stretched to fill a square reads as a completely different
// layout from the one that is there, and a browser whose pictures lie is worse
// than one with no pictures. The shorter axis is CENTRED in what is left over.
//
// A DEAD-STRAIGHT TRACK IS THE CASE THAT BREAKS IT. Its plan has zero extent
// across, so the obvious scale is a division by zero, and the obvious guard --
// bail out and draw nothing -- makes the commonest first track in the world the
// one with no picture. It is drawn down the middle of its own axis instead.
inline std::vector<float> PlanThumb(const std::vector<float>& XY,
                                    std::size_t MaxPoints = 64)
{
    std::vector<float> Out;
    const std::size_t N = XY.size() / 2;
    if (N == 0) { return Out; }

    float MinX = XY[0], MaxX = XY[0], MinY = XY[1], MaxY = XY[1];
    for (std::size_t i = 0; i < N; ++i)
    {
        MinX = XY[i * 2] < MinX ? XY[i * 2] : MinX;
        MaxX = XY[i * 2] > MaxX ? XY[i * 2] : MaxX;
        MinY = XY[i * 2 + 1] < MinY ? XY[i * 2 + 1] : MinY;
        MaxY = XY[i * 2 + 1] > MaxY ? XY[i * 2 + 1] : MaxY;
    }

    const float SpanX = MaxX - MinX;
    const float SpanY = MaxY - MinY;
    const float Span = SpanX > SpanY ? SpanX : SpanY;
    // Both axes flat is a single point -- a one-segment track of zero length, or
    // a track being built. One dot in the middle, and no division.
    const float Scale = Span > 1e-6f ? 1.0f / Span : 0.0f;
    const float PadX = (1.0f - SpanX * Scale) * 0.5f;
    const float PadY = (1.0f - SpanY * Scale) * 0.5f;

    // EVENLY SPACED, INCLUDING THE LAST POINT. Taking every Nth sample drops the
    // end of the track whenever the count does not divide, which on a circuit
    // leaves a visible gap exactly where the layout closes -- the one feature of
    // the picture somebody is looking for.
    const std::size_t Want = (MaxPoints < 2 ? 2 : MaxPoints);
    const std::size_t Take = N < Want ? N : Want;
    Out.reserve(Take * 2);
    for (std::size_t i = 0; i < Take; ++i)
    {
        const std::size_t Src = Take == 1 ? 0
            : static_cast<std::size_t>((static_cast<double>(i) * (N - 1)) / (Take - 1) + 0.5);
        Out.push_back(PadX + (XY[Src * 2] - MinX) * Scale);
        Out.push_back(PadY + (XY[Src * 2 + 1] - MinY) * Scale);
    }
    return Out;
}

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

// ponytail: no folder tree — that is what the OS file dialog is for, and the
// browser's job is the handful of tracks somebody actually works on.
//
// THUMBNAILS LANDED, AND THE DEFERRAL THAT STOOD HERE HAD THE WRONG PREMISE. It
// said a thumbnail "wants a render on save", which is what makes it expensive:
// a picture taken at one moment, stored beside the file, wrong the moment the
// track changes, missing for every track written by a build that did not take
// one, and a second thing to delete when a track is deleted.
//
// A track already carries its own picture. `PlanThumb` is the plan view, taken
// from the walk the browser ALREADY does to read length and height, so it costs
// nothing extra, cannot go stale, needs no file, and exists for a track that
// has never been opened. What it cannot show is theming or scenery — the day
// this project has either, a rendered thumbnail is worth its cost and this is
// what it replaces.
