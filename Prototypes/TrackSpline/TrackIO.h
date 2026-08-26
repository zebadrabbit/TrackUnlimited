// TrackUnlimited Phase 1: the authored data model, and a diffable save format.
// Plain C++17, no engine dependency, same conventions as TrackSpline.h.
//
// Two things live here because neither is worth much without the other.
//
// 1. FAuthoredSegment — what the AUTHOR typed, which is not what the
//    integrator runs. A helix is "radius 20, climbing 15 degrees, 2 turns",
//    not "length 261.6, curvature 0.0467, torsion 0.0125". Both describe the
//    same curve; only the first can be edited afterwards.
//
// 2. A JSON reader/writer over that model.
//
// The rule the format follows, and the reason both halves exist:
//
//   STORE WHAT WAS TYPED, NEVER WHAT WAS DERIVED.
//
// The alternative was measured and rejected while adding the helix. Approximate
// a helix with the straight/arc/clothoid vocabulary and it costs roughly one
// segment per metre — a 300 m helix becomes 300 segments. They ride identically;
// a rider cannot tell. What is lost is the radius, which exists nowhere in those
// 300 segments' 600 fitted curvature endpoints. Grouping them in the UI fixes
// the row count and not that: to edit the group you must regenerate it, so the
// group has to store radius/climb/turns anyway — at which point the 300
// segments are a cache of the authored parameters, and this file stores the
// authored parameters alone.
//
// Same argument decides the file format. Nudging a helix radius by 2 m changes
// ONE number in the diff here. Stored as an expansion it would change ~600, and
// only if the regeneration were bit-deterministic across compilers, which is
// not a promise worth making. "Diffable" is in PROJECT_PLAN.md for a solo dev
// working in git; a format that rewrites 600 floats when you touch one field is
// not diffable in any useful sense.
//
// Units: metres, and **degrees** for angles — the one place in this codebase
// that is not radians, and deliberately so. Nobody types 0.26179938779914941;
// they type 15. Angles are degrees in FAuthoredSegment and degrees in the file,
// and BuildSegment converts to radians on the way into the geometry, which is
// radians throughout as usual. Field and key names carry the unit so a hand
// edit cannot get it wrong.
//
// The conversion runs ONE direction only. That is what keeps the round trip
// exact: degrees -> radians -> degrees would lose an ulp, so it never happens.
// This is the same "convert at the boundary, never inside the math" rule
// CLAUDE.md already states for the UE port's units and handedness.

#pragma once

#include "TrackSpline.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

// ------------------------------------------------------------ authored model

enum class ESegmentKind
{
    Straight,
    Arc,
    Clothoid,
    Helix,

    // Everything the authored vocabulary cannot say: a hill (pitch curvature,
    // which no Make* helper builds yet), and every segment of an imported NL2
    // track, which is thousands of fitted micro-segments whose original
    // vocabulary is gone and cannot be recovered.
    //
    // Its presence in a file is information, not a wart. A track full of Raw is
    // a track nobody can meaningfully edit, and that is exactly what
    // CLAUDE.md's scope guard says an import is.
    Raw,

    // A vertical curve: how far the nose comes up or down, the tightest radius
    // in it, and whether it eases in, eases out or holds -- MakePitch. Appended
    // AFTER Raw because the editor's enum stores this by value in a level, and
    // a renumbered enumerator turns every stored hill into something else.
    Pitch,
};

// One row of the editor's segment list. Which fields matter depends on Kind —
// flat rather than a variant because every field is a real number a real kind
// needs, and a tagged union buys nothing at this size.
//
// ponytail: no per-kind subtype and no field-presence tracking. Build() reads
// only the fields its kind uses, and the writer only emits those, so an unused
// field is invisible rather than wrong.
// ===================== THE CONTROL LAYER IS AUTHORED TOO =====================
//
// A device is a property of a SEGMENT, exactly as a radius is: a lift is however
// many segments in a row say "lift", and the runtime zone is a contiguous run of
// them derived by a walk. So it belongs in the authored document, and until it
// was here a saved track was geometry with no brakes, no station and no lift —
// the ride's entire control layer dropped on the way to disc, silently, which is
// the worst shape a data loss can take.
//
// STORED BY NAME, never by index. An enumerator's number changes the day somebody
// inserts a value above it, and every stored track then means something different
// with nothing to notice. The same rule the segment kinds already follow.
enum class EAuthoredZone
{
    None,
    Lift,
    Launch,
    Brake,          // trim only: can remove energy, can never hold a train
    BlockBrake,     // friction pad AND drive tyres, so it can hold and release
    Station,
    StationUnload,
    StationLoad,
};

struct FAuthoredSegment
{
    ESegmentKind Kind = ESegmentKind::Straight;

    double Length = 0.0;            // straight, arc, clothoid. Helix derives it.
    double Radius = 0.0;            // arc, helix. +ve left, -ve right.
    // Clothoid endpoints, 1/m. NOT angles, so they stay 1/m — and not radii
    // either, deliberately. An easement out of a straight has an endpoint with
    // NO radius, and curvature 0 says that as an ordinary value where radius
    // would need infinity or a sentinel. It is the same reason the geometry
    // stores curvature at all (TrackSpline.h). The cost is that a transition
    // into R=30 reads as 0.033333333333333333 in the file rather than 30; the
    // editor should show radius with a "straight" option and convert.
    double CurvatureStart = 0.0;
    double CurvatureEnd = 0.0;
    double ClimbAngleDegrees = 0.0; // helix, +ve ascending
    double Turns = 0.0;             // helix, revolutions
    double PitchDeltaDegrees = 0.0; // pitch, +ve raises the nose; Radius is its tightest radius
    EPitchEase PitchEase = EPitchEase::EaseIn;

    double RollStartDegrees = 0.0; // every kind
    double RollEndDegrees = 0.0;
    ERollMode RollMode = ERollMode::PathRelative;

    // ---- The device on this segment, if any. Defaults match the editor's.
    //
    // A DEVICE IS ITS KIND AND ITS SPEED, and since a run ends where either
    // changes, both are authored per segment rather than per run. Acceleration and
    // deceleration are the device's own too — a chain hauling at 0.61 g is nothing
    // like a real chain, and one global figure for every device on every ride was
    // a number nobody typed standing in for one they should have.
    EAuthoredZone Zone = EAuthoredZone::None;
    double ZoneSpeed = 4.0;
    double ZoneAccel = 6.0;
    double ZoneDecel = 6.0;
    // The friction pad's rate, which is a SECOND MACHINE rather than a mode of the
    // first: a pad is a ceiling that can only remove energy, the tyres are a
    // setpoint driven toward from either side. Kept separate because either can
    // fail without the other.
    double ZoneBrakeDecel = 0.0;

    // NOT A ZONE, deliberately, and that is why it sits outside the block above: a
    // catch has no speed and nothing commanding it, and it overlaps zones freely
    // because a lift hill is both at once. It is written whatever the zone is.
    bool bAntiRollback = false;

    // Three loading positions are the same kind at the same speed, so nothing the
    // derivation walk can see tells them apart. This is the author saying so.
    bool bStartsNewDevice = false;

    FTrackSegment RawSegment; // Kind == Raw only. Its own fields stay radians.
};

constexpr double AuthoredDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double RadiansToAuthoredDegrees = 180.0 / 3.14159265358979323846;

// The authored row turned into the thing the integrator runs. One direction
// only, deliberately: there is no FromSegment, because deriving "radius 20,
// 15 degrees, 2 turns" back out of a curvature profile is the recovery problem
// this whole file exists to avoid needing to solve.
inline FTrackSegment BuildSegment(const FAuthoredSegment& A)
{
    FTrackSegment Seg;
    switch (A.Kind)
    {
    case ESegmentKind::Straight:
        Seg = MakeStraight(A.Length);
        break;
    case ESegmentKind::Arc:
        Seg = MakeArc(A.Length, A.Radius);
        break;
    case ESegmentKind::Clothoid:
        Seg = MakeClothoid(A.Length, A.CurvatureStart, A.CurvatureEnd);
        break;
    case ESegmentKind::Helix:
        Seg = MakeHelix(A.Radius, A.ClimbAngleDegrees * AuthoredDegreesToRadians, A.Turns);
        break;
    case ESegmentKind::Raw:
        Seg = A.RawSegment;
        break;
    case ESegmentKind::Pitch:
        Seg = MakePitch(A.PitchDeltaDegrees * AuthoredDegreesToRadians, A.Radius, A.PitchEase);
        break;
    }
    // Roll is applied here rather than through the Make* roll arguments so that
    // every kind carries it identically and Raw cannot disagree with the rest.
    // This is also the only place degrees become radians.
    Seg.RollStart = A.RollStartDegrees * AuthoredDegreesToRadians;
    Seg.RollEnd = A.RollEndDegrees * AuthoredDegreesToRadians;
    Seg.RollMode = A.RollMode;
    return Seg;
}

// Convenience constructors mirroring the Make* helpers. Note the roll and climb
// arguments are DEGREES here where the Make* equivalents take radians — these
// are the authoring side of the boundary. The names say so.
inline FAuthoredSegment AuthorStraight(double Length, double RollDegrees = 0.0)
{
    FAuthoredSegment A;
    A.Kind = ESegmentKind::Straight;
    A.Length = Length;
    A.RollStartDegrees = A.RollEndDegrees = RollDegrees;
    return A;
}

inline FAuthoredSegment AuthorArc(double Length, double Radius, double RollDegrees = 0.0)
{
    FAuthoredSegment A;
    A.Kind = ESegmentKind::Arc;
    A.Length = Length;
    A.Radius = Radius;
    A.RollStartDegrees = A.RollEndDegrees = RollDegrees;
    return A;
}

inline FAuthoredSegment AuthorClothoid(double Length, double CurvatureStart, double CurvatureEnd,
                                       double RollStartDegrees = 0.0, double RollEndDegrees = 0.0)
{
    FAuthoredSegment A;
    A.Kind = ESegmentKind::Clothoid;
    A.Length = Length;
    A.CurvatureStart = CurvatureStart;
    A.CurvatureEnd = CurvatureEnd;
    A.RollStartDegrees = RollStartDegrees;
    A.RollEndDegrees = RollEndDegrees;
    return A;
}

inline FAuthoredSegment AuthorHelix(double Radius, double ClimbAngleDegrees, double Turns,
                                    double RollDegrees = 0.0)
{
    FAuthoredSegment A;
    A.Kind = ESegmentKind::Helix;
    A.Radius = Radius;
    A.ClimbAngleDegrees = ClimbAngleDegrees;
    A.Turns = Turns;
    A.RollStartDegrees = A.RollEndDegrees = RollDegrees;
    return A;
}

inline FAuthoredSegment AuthorPitch(double PitchDeltaDegrees, double Radius, EPitchEase Ease,
                                    double RollStartDegrees = 0.0, double RollEndDegrees = 0.0)
{
    FAuthoredSegment A;
    A.Kind = ESegmentKind::Pitch;
    A.PitchDeltaDegrees = PitchDeltaDegrees;
    A.Radius = Radius;
    A.PitchEase = Ease;
    A.RollStartDegrees = RollStartDegrees;
    A.RollEndDegrees = RollEndDegrees;
    return A;
}

// The one place the conversion runs backwards, because the caller already holds
// radians. Confined to Raw, which is import and hand-tuned geometry — never a
// value a human typed, so there is no authored form to preserve exactly.
inline FAuthoredSegment AuthorRaw(const FTrackSegment& Seg)
{
    FAuthoredSegment A;
    A.Kind = ESegmentKind::Raw;
    A.RawSegment = Seg;
    A.RollStartDegrees = Seg.RollStart * RadiansToAuthoredDegrees;
    A.RollEndDegrees = Seg.RollEnd * RadiansToAuthoredDegrees;
    A.RollMode = Seg.RollMode;
    return A;
}

// -------------------------------------------------------------------- document

// Bumped only when a change would make an older reader misread a newer file.
// Adding an optional field does not qualify — omitted fields take their
// defaults, which is how Torsion and RollMode were both added to an existing
// model without touching a single stored track.
constexpr int TrackFormatVersion = 1;

struct FTrackDocument
{
    int Version = TrackFormatVersion;
    double HeartlineHeight = 1.1;
    std::vector<FAuthoredSegment> Segments;
};

inline FTrack BuildTrack(const FTrackDocument& Doc)
{
    FTrack Track(Doc.HeartlineHeight);
    for (const FAuthoredSegment& A : Doc.Segments)
    {
        Track.AddSegment(BuildSegment(A));
    }
    return Track;
}

inline std::vector<FTrackSegment> BuildSegments(const FTrackDocument& Doc)
{
    std::vector<FTrackSegment> Out;
    Out.reserve(Doc.Segments.size());
    for (const FAuthoredSegment& A : Doc.Segments)
    {
        Out.push_back(BuildSegment(A));
    }
    return Out;
}

namespace TrackIODetail
{

// The shortest decimal form that reads back as the identical double. %.17g
// always round-trips and always looks like 0.10000000000000001; %.15g is clean
// and nearly always exact. Try the readable one, keep it only if it survives.
// Costs one strtod per number written and makes the diffs human.
inline std::string Num(double V)
{
    char Buf[48];
    std::snprintf(Buf, sizeof(Buf), "%.15g", V);
    if (std::strtod(Buf, nullptr) != V)
    {
        std::snprintf(Buf, sizeof(Buf), "%.17g", V);
    }
    return std::string(Buf);
}

inline const char* KindName(ESegmentKind K)
{
    switch (K)
    {
    case ESegmentKind::Straight: return "straight";
    case ESegmentKind::Arc: return "arc";
    case ESegmentKind::Clothoid: return "clothoid";
    case ESegmentKind::Helix: return "helix";
    case ESegmentKind::Raw: return "raw";
    case ESegmentKind::Pitch: return "pitch";
    }
    return "raw";
}

inline bool KindFromName(const std::string& S, ESegmentKind& Out)
{
    if (S == "straight") { Out = ESegmentKind::Straight; return true; }
    if (S == "arc") { Out = ESegmentKind::Arc; return true; }
    if (S == "clothoid") { Out = ESegmentKind::Clothoid; return true; }
    if (S == "helix") { Out = ESegmentKind::Helix; return true; }
    if (S == "raw") { Out = ESegmentKind::Raw; return true; }
    if (S == "pitch") { Out = ESegmentKind::Pitch; return true; }
    return false;
}

// As permanent as the kind names: these land in files.
inline const char* PitchEaseName(EPitchEase E)
{
    switch (E)
    {
    case EPitchEase::EaseIn: return "in";
    case EPitchEase::EaseOut: return "out";
    case EPitchEase::Constant: return "constant";
    }
    return "in";
}

inline bool PitchEaseFromName(const std::string& S, EPitchEase& Out)
{
    if (S == "in") { Out = EPitchEase::EaseIn; return true; }
    if (S == "out") { Out = EPitchEase::EaseOut; return true; }
    if (S == "constant") { Out = EPitchEase::Constant; return true; }
    return false;
}

// THESE STRINGS ARE AS PERMANENT AS THE KEYS ARE. They land in stored tracks, so
// renaming one silently turns every ride that used that device into unpowered
// track — which builds, runs, and simply never dispatches.
inline const char* ZoneName(EAuthoredZone Z)
{
    switch (Z)
    {
    case EAuthoredZone::None: return "none";
    case EAuthoredZone::Lift: return "lift";
    case EAuthoredZone::Launch: return "launch";
    case EAuthoredZone::Brake: return "brake";
    case EAuthoredZone::BlockBrake: return "blockBrake";
    case EAuthoredZone::Station: return "station";
    case EAuthoredZone::StationUnload: return "stationUnload";
    case EAuthoredZone::StationLoad: return "stationLoad";
    }
    return "none";
}

// REFUSED RATHER THAN DEFAULTED. An unrecognised device name is a file this build
// cannot honour, and quietly reading it as unpowered track would open somebody's
// ride with its brakes missing and no complaint — the exact silent loss this
// whole block exists to end. The parser turns a false here into a stated error.
inline bool ZoneFromName(const std::string& S, EAuthoredZone& Out)
{
    if (S == "none") { Out = EAuthoredZone::None; return true; }
    if (S == "lift") { Out = EAuthoredZone::Lift; return true; }
    if (S == "launch") { Out = EAuthoredZone::Launch; return true; }
    if (S == "brake") { Out = EAuthoredZone::Brake; return true; }
    if (S == "blockBrake") { Out = EAuthoredZone::BlockBrake; return true; }
    if (S == "station") { Out = EAuthoredZone::Station; return true; }
    if (S == "stationUnload") { Out = EAuthoredZone::StationUnload; return true; }
    if (S == "stationLoad") { Out = EAuthoredZone::StationLoad; return true; }
    return false;
}

// A key/value pair as it appeared in the file: value kept as text, interpreted
// later by whoever knows what the key means. Enough for a schema with no
// nesting inside a segment, and it makes key ORDER irrelevant, which a
// hand-edited file will eventually depend on.
//
// NOT called FField, and not because the prefix is ugly. UE declares a global
// `FField` in UObject/Field.h, so inside the engine build that name wins the
// lookup here and every use silently resolves to the wrong type. Standalone
// clang never sees it, so it compiles clean right up until the port. This is
// the second time — `FFrame` collided with the Blueprint VM's and became
// `FTrackFrame`. These headers are engine-free but they are COMPILED INTO the
// engine, so a name only needs to be unique against all of UE.
struct FTrackIOField
{
    std::string Key;
    std::string Value;
};

inline const std::string* Find(const std::vector<FTrackIOField>& Fields, const char* Key)
{
    for (const FTrackIOField& F : Fields)
    {
        if (F.Key == Key)
        {
            return &F.Value;
        }
    }
    return nullptr;
}

// A trust boundary, so: bounds-checked everywhere, no asserts on input, and an
// error string rather than a thrown exception or a silently absorbed default.
// This parses OUR schema, not arbitrary JSON — no nested objects inside a
// segment, no arrays other than "segments", no \u escapes.
//
// ponytail: hand-rolled rather than a JSON dependency. It is ~120 lines against
// a fixed schema, and at the UE port boundary this is replaced wholesale by
// FJsonObjectConverter anyway — the durable artefact here is the FORMAT, not
// the parser. Reach for a library the day the schema stops being flat.
struct FCursor
{
    const std::string& S;
    std::size_t I = 0;
    std::string Error;

    explicit FCursor(const std::string& InS) : S(InS) {}

    bool Fail(const std::string& Msg)
    {
        if (Error.empty())
        {
            char Buf[64];
            std::snprintf(Buf, sizeof(Buf), " (at byte %zu)", I);
            Error = Msg + Buf;
        }
        return false;
    }

    void SkipWs()
    {
        while (I < S.size() && (S[I] == ' ' || S[I] == '\t' || S[I] == '\n' || S[I] == '\r'))
        {
            ++I;
        }
    }

    bool Peek(char C)
    {
        SkipWs();
        return I < S.size() && S[I] == C;
    }

    bool Take(char C)
    {
        SkipWs();
        if (I < S.size() && S[I] == C)
        {
            ++I;
            return true;
        }
        return false;
    }

    bool Expect(char C)
    {
        if (Take(C))
        {
            return true;
        }
        char Buf[48];
        std::snprintf(Buf, sizeof(Buf), "expected '%c'", C);
        return Fail(Buf);
    }

    bool ReadString(std::string& Out)
    {
        SkipWs();
        if (I >= S.size() || S[I] != '"')
        {
            return Fail("expected a string");
        }
        ++I;
        Out.clear();
        while (I < S.size() && S[I] != '"')
        {
            if (S[I] == '\\')
            {
                // Our writer never emits an escape, so accepting only the two
                // that could ever appear keeps a hand-edited file honest rather
                // than half-supporting a spec we do not implement.
                if (I + 1 >= S.size() || (S[I + 1] != '"' && S[I + 1] != '\\'))
                {
                    return Fail("unsupported string escape");
                }
                Out.push_back(S[I + 1]);
                I += 2;
                continue;
            }
            Out.push_back(S[I]);
            ++I;
        }
        if (I >= S.size())
        {
            return Fail("unterminated string");
        }
        ++I;
        return true;
    }

    // A scalar: string, number, or bare true/false/null. Kept as text.
    bool ReadScalar(std::string& Out)
    {
        SkipWs();
        if (I < S.size() && S[I] == '"')
        {
            return ReadString(Out);
        }
        const std::size_t Start = I;
        while (I < S.size() && S[I] != ',' && S[I] != '}' && S[I] != ']' && S[I] != ' '
               && S[I] != '\t' && S[I] != '\n' && S[I] != '\r')
        {
            ++I;
        }
        if (I == Start)
        {
            return Fail("expected a value");
        }
        Out = S.substr(Start, I - Start);
        return true;
    }

    // A flat object: {"k": scalar, ...}. Rejects nesting rather than skipping
    // it, so a file this parser cannot fully represent is refused, not
    // half-read.
    bool ReadFlatObject(std::vector<FTrackIOField>& Out)
    {
        if (!Expect('{'))
        {
            return false;
        }
        Out.clear();
        if (Take('}'))
        {
            return true;
        }
        for (;;)
        {
            FTrackIOField F;
            if (!ReadString(F.Key) || !Expect(':'))
            {
                return false;
            }
            if (Peek('{') || Peek('['))
            {
                return Fail("nested values are not part of this schema");
            }
            if (!ReadScalar(F.Value))
            {
                return false;
            }
            Out.push_back(F);
            if (Take(','))
            {
                continue;
            }
            return Expect('}');
        }
    }
};

// Reject anything that is not a finite number, rather than letting strtod's
// zero-on-failure become a plausible-looking value. A "length": "abc" that
// silently reads as 0 is exactly the class of bug PHASE0_FINDINGS records for
// MakeArc(L, 0): visibly broken beats plausibly wrong.
inline bool ReadNumber(const std::vector<FTrackIOField>& Fields, const char* Key, double& Out,
                       bool bRequired, std::string& OutError)
{
    const std::string* Text = Find(Fields, Key);
    if (Text == nullptr)
    {
        if (bRequired)
        {
            OutError = std::string("missing required field \"") + Key + "\"";
            return false;
        }
        return true; // absent optional field keeps the caller's default
    }
    char* End = nullptr;
    const double V = std::strtod(Text->c_str(), &End);
    if (End == Text->c_str() || *End != '\0' || !std::isfinite(V))
    {
        OutError = std::string("field \"") + Key + "\" is not a finite number: " + *Text;
        return false;
    }
    Out = V;
    return true;
}

} // namespace TrackIODetail

// ----------------------------------------------------------------- write

// Emits only the fields the segment's kind uses, and omits anything sitting at
// its default. Both rules serve the same goal: a track file should contain the
// author's decisions and nothing else, so a diff shows what changed rather than
// what the struct happens to have. It is also what lets a new field be added
// without rewriting every stored track — see TrackFormatVersion.
inline bool WriteTrackJson(const FTrackDocument& Doc, std::string& Out, std::string& OutError)
{
    using namespace TrackIODetail;
    OutError.clear();
    Out.clear();

    // Refuse rather than emit "nan", which is not JSON and which no reader
    // could take back in. ValidateTrack should have caught this first; this is
    // the backstop that keeps an unreadable file off disk.
    auto Check = [&OutError](double V, const char* What, std::size_t Index) {
        if (std::isfinite(V))
        {
            return true;
        }
        char Buf[128];
        std::snprintf(Buf, sizeof(Buf), "segment %zu field %s is not finite; refusing to write",
                      Index, What);
        OutError = Buf;
        return false;
    };

    Out += "{\n";
    Out += "  \"format\": \"trackunlimited.track\",\n";
    Out += "  \"version\": " + Num(Doc.Version) + ",\n";
    Out += "  \"heartlineHeight\": " + Num(Doc.HeartlineHeight) + ",\n";
    Out += "  \"segments\": [\n";

    for (std::size_t i = 0; i < Doc.Segments.size(); ++i)
    {
        const FAuthoredSegment& A = Doc.Segments[i];
        std::string Row = "    {\"kind\": \"";
        Row += KindName(A.Kind);
        Row += "\"";

        auto Add = [&](const char* Key, double V) { Row += std::string(", \"") + Key + "\": " + Num(V); };

        switch (A.Kind)
        {
        case ESegmentKind::Straight:
            if (!Check(A.Length, "length", i)) { return false; }
            Add("length", A.Length);
            break;
        case ESegmentKind::Arc:
            if (!Check(A.Length, "length", i) || !Check(A.Radius, "radius", i)) { return false; }
            Add("length", A.Length);
            Add("radius", A.Radius);
            break;
        case ESegmentKind::Clothoid:
            if (!Check(A.Length, "length", i) || !Check(A.CurvatureStart, "curvatureStart", i)
                || !Check(A.CurvatureEnd, "curvatureEnd", i)) { return false; }
            Add("length", A.Length);
            Add("curvatureStart", A.CurvatureStart);
            Add("curvatureEnd", A.CurvatureEnd);
            break;
        case ESegmentKind::Helix:
            if (!Check(A.Radius, "radius", i) || !Check(A.ClimbAngleDegrees, "climbAngleDeg", i)
                || !Check(A.Turns, "turns", i)) { return false; }
            // No length: it is 2*pi*R*turns/cos(climb), and a derived value in
            // a file is a value that can disagree with its own source.
            Add("radius", A.Radius);
            Add("climbAngleDeg", A.ClimbAngleDegrees);
            Add("turns", A.Turns);
            break;
        case ESegmentKind::Pitch:
            if (!Check(A.Radius, "radius", i) || !Check(A.PitchDeltaDegrees, "pitchDeg", i)) { return false; }
            // No length, for the helix's reason: it is derived from these.
            Add("radius", A.Radius);
            Add("pitchDeg", A.PitchDeltaDegrees);
            Row += std::string(", \"pitchEase\": \"") + PitchEaseName(A.PitchEase) + "\"";
            break;
        case ESegmentKind::Raw:
        {
            const FTrackSegment& S = A.RawSegment;
            if (!Check(S.Length, "length", i) || !Check(S.YawCurvatureStart, "yawStart", i)
                || !Check(S.YawCurvatureEnd, "yawEnd", i)
                || !Check(S.PitchCurvatureStart, "pitchStart", i)
                || !Check(S.PitchCurvatureEnd, "pitchEnd", i) || !Check(S.Torsion, "torsion", i)
                || !Check(S.TorsionRatio, "torsionRatio", i))
            {
                return false;
            }
            Add("length", S.Length);
            if (S.YawCurvatureStart != 0.0 || S.YawCurvatureEnd != 0.0)
            {
                Add("yawStart", S.YawCurvatureStart);
                Add("yawEnd", S.YawCurvatureEnd);
            }
            if (S.PitchCurvatureStart != 0.0 || S.PitchCurvatureEnd != 0.0)
            {
                Add("pitchStart", S.PitchCurvatureStart);
                Add("pitchEnd", S.PitchCurvatureEnd);
            }
            if (S.Torsion != 0.0)
            {
                Add("torsion", S.Torsion);
            }
            if (S.TorsionRatio != 0.0)
            {
                Add("torsionRatio", S.TorsionRatio);
            }
            break;
        }
        }

        if (!Check(A.RollStartDegrees, "rollStartDeg", i)
            || !Check(A.RollEndDegrees, "rollEndDeg", i))
        {
            return false;
        }
        // One field when the roll is constant, two when it ramps. The common
        // case is constant, and it should not cost two lines of noise.
        if (A.RollStartDegrees == A.RollEndDegrees)
        {
            if (A.RollStartDegrees != 0.0)
            {
                Add("rollDeg", A.RollStartDegrees);
            }
        }
        else
        {
            Add("rollStartDeg", A.RollStartDegrees);
            Add("rollEndDeg", A.RollEndDegrees);
        }
        if (A.RollMode == ERollMode::WorldBank)
        {
            Row += ", \"rollMode\": \"worldBank\"";
        }
        else if (A.RollMode == ERollMode::FollowsTorsion)
        {
            Row += ", \"rollMode\": \"followsTorsion\"";
        }

        // ---- THE DEVICE.
        //
        // OMITTED ONLY WHEN AT ITS DEFAULT, and never because the zone happens to
        // be None. The editor hides a device's speed on unpowered track and
        // HIDDEN IS NOT DELETED — flipping a segment to plain track and back keeps
        // the number somebody typed. A writer that dropped those fields would undo
        // that rule at the one moment it costs real work, which is the save.
        if (A.Zone != EAuthoredZone::None)
        {
            Row += std::string(", \"zone\": \"") + ZoneName(A.Zone) + "\"";
        }
        if (!Check(A.ZoneSpeed, "zoneSpeed", i) || !Check(A.ZoneAccel, "zoneAccel", i)
            || !Check(A.ZoneDecel, "zoneDecel", i)
            || !Check(A.ZoneBrakeDecel, "zoneBrakeDecel", i))
        {
            return false;
        }
        if (A.ZoneSpeed != 4.0) { Add("zoneSpeed", A.ZoneSpeed); }
        if (A.ZoneAccel != 6.0) { Add("zoneAccel", A.ZoneAccel); }
        if (A.ZoneDecel != 6.0) { Add("zoneDecel", A.ZoneDecel); }
        if (A.ZoneBrakeDecel != 0.0) { Add("zoneBrakeDecel", A.ZoneBrakeDecel); }
        if (A.bAntiRollback) { Row += ", \"antiRollback\": true"; }
        if (A.bStartsNewDevice) { Row += ", \"startsNewDevice\": true"; }

        Row += "}";
        if (i + 1 < Doc.Segments.size())
        {
            Row += ",";
        }
        Out += Row + "\n";
    }

    Out += "  ]\n}\n";
    return true;
}

// ------------------------------------------------------------------ read

inline bool ParseTrackJson(const std::string& Text, FTrackDocument& Out, std::string& OutError)
{
    using namespace TrackIODetail;
    OutError.clear();
    Out = FTrackDocument();

    FCursor C(Text);
    if (!C.Expect('{'))
    {
        OutError = C.Error;
        return false;
    }

    std::vector<std::vector<FTrackIOField>> SegmentFields;
    bool bSawSegments = false;

    for (;;)
    {
        if (C.Take('}'))
        {
            break;
        }
        std::string Key;
        if (!C.ReadString(Key) || !C.Expect(':'))
        {
            OutError = C.Error;
            return false;
        }

        if (Key == "segments")
        {
            bSawSegments = true;
            if (!C.Expect('['))
            {
                OutError = C.Error;
                return false;
            }
            if (!C.Take(']'))
            {
                for (;;)
                {
                    std::vector<FTrackIOField> Fields;
                    if (!C.ReadFlatObject(Fields))
                    {
                        OutError = C.Error;
                        return false;
                    }
                    SegmentFields.push_back(Fields);
                    if (C.Take(','))
                    {
                        continue;
                    }
                    if (!C.Expect(']'))
                    {
                        OutError = C.Error;
                        return false;
                    }
                    break;
                }
            }
        }
        else
        {
            // Unknown top-level keys are read and ignored, so a file written by
            // a later version still loads. Unknown SEGMENT keys are treated the
            // same way, for the same reason.
            std::string Value;
            if (C.Peek('{') || C.Peek('['))
            {
                C.Fail("nested values are not part of this schema");
                OutError = C.Error;
                return false;
            }
            if (!C.ReadScalar(Value))
            {
                OutError = C.Error;
                return false;
            }
            if (Key == "version")
            {
                char* End = nullptr;
                const long V = std::strtol(Value.c_str(), &End, 10);
                if (End == Value.c_str() || *End != '\0')
                {
                    OutError = "version is not an integer: " + Value;
                    return false;
                }
                if (V > TrackFormatVersion)
                {
                    char Buf[160];
                    std::snprintf(Buf, sizeof(Buf),
                                  "file is format version %ld; this build understands up to %d. "
                                  "Refusing rather than guessing at fields it does not know.",
                                  V, TrackFormatVersion);
                    OutError = Buf;
                    return false;
                }
                Out.Version = static_cast<int>(V);
            }
            else if (Key == "heartlineHeight")
            {
                char* End = nullptr;
                const double V = std::strtod(Value.c_str(), &End);
                if (End == Value.c_str() || *End != '\0' || !std::isfinite(V))
                {
                    OutError = "heartlineHeight is not a finite number: " + Value;
                    return false;
                }
                Out.HeartlineHeight = V;
            }
        }

        if (C.Take(','))
        {
            continue;
        }
        if (!C.Expect('}'))
        {
            OutError = C.Error;
            return false;
        }
        break;
    }

    if (!bSawSegments)
    {
        OutError = "no \"segments\" array; this is not a track file";
        return false;
    }

    for (std::size_t i = 0; i < SegmentFields.size(); ++i)
    {
        const std::vector<FTrackIOField>& F = SegmentFields[i];
        const std::string* KindText = Find(F, "kind");
        if (KindText == nullptr)
        {
            char Buf[96];
            std::snprintf(Buf, sizeof(Buf), "segment %zu has no \"kind\"", i);
            OutError = Buf;
            return false;
        }

        FAuthoredSegment A;
        if (!KindFromName(*KindText, A.Kind))
        {
            // Not skipped. A kind we do not understand is geometry we would
            // silently drop out of the middle of a track, leaving a file that
            // loads, looks fine, and is a different ride.
            char Buf[160];
            std::snprintf(Buf, sizeof(Buf),
                          "segment %zu has unknown kind \"%s\". Refusing: skipping it would "
                          "silently produce a different track that still looks valid.",
                          i, KindText->c_str());
            OutError = Buf;
            return false;
        }

        std::string FieldError;
        bool bOk = true;
        switch (A.Kind)
        {
        case ESegmentKind::Straight:
            bOk = ReadNumber(F, "length", A.Length, true, FieldError);
            break;
        case ESegmentKind::Arc:
            bOk = ReadNumber(F, "length", A.Length, true, FieldError)
                  && ReadNumber(F, "radius", A.Radius, true, FieldError);
            break;
        case ESegmentKind::Clothoid:
            bOk = ReadNumber(F, "length", A.Length, true, FieldError)
                  && ReadNumber(F, "curvatureStart", A.CurvatureStart, true, FieldError)
                  && ReadNumber(F, "curvatureEnd", A.CurvatureEnd, true, FieldError);
            break;
        case ESegmentKind::Helix:
            bOk = ReadNumber(F, "radius", A.Radius, true, FieldError)
                  && ReadNumber(F, "climbAngleDeg", A.ClimbAngleDegrees, true, FieldError)
                  && ReadNumber(F, "turns", A.Turns, true, FieldError);
            break;
        case ESegmentKind::Raw:
            bOk = ReadNumber(F, "length", A.RawSegment.Length, true, FieldError)
                  && ReadNumber(F, "yawStart", A.RawSegment.YawCurvatureStart, false, FieldError)
                  && ReadNumber(F, "yawEnd", A.RawSegment.YawCurvatureEnd, false, FieldError)
                  && ReadNumber(F, "pitchStart", A.RawSegment.PitchCurvatureStart, false, FieldError)
                  && ReadNumber(F, "pitchEnd", A.RawSegment.PitchCurvatureEnd, false, FieldError)
                  && ReadNumber(F, "torsion", A.RawSegment.Torsion, false, FieldError)
                  && ReadNumber(F, "torsionRatio", A.RawSegment.TorsionRatio, false, FieldError);
            break;
        case ESegmentKind::Pitch:
            bOk = ReadNumber(F, "radius", A.Radius, true, FieldError)
                  && ReadNumber(F, "pitchDeg", A.PitchDeltaDegrees, true, FieldError);
            if (bOk)
            {
                // REQUIRED, and an unknown one is refused: a hill read with the
                // wrong ease is a different hill that still looks valid.
                const std::string* E = Find(F, "pitchEase");
                if (E == nullptr)
                {
                    FieldError = "pitch needs \"pitchEase\" (in, out or constant)";
                    bOk = false;
                }
                else if (!PitchEaseFromName(*E, A.PitchEase))
                {
                    FieldError = "unknown pitchEase \"" + *E + "\"; expected in, out or constant";
                    bOk = false;
                }
            }
            break;
        }

        double Roll = 0.0;
        bOk = bOk && ReadNumber(F, "rollDeg", Roll, false, FieldError);
        A.RollStartDegrees = A.RollEndDegrees = Roll;
        bOk = bOk && ReadNumber(F, "rollStartDeg", A.RollStartDegrees, false, FieldError)
              && ReadNumber(F, "rollEndDeg", A.RollEndDegrees, false, FieldError);

        if (!bOk)
        {
            OutError = "segment " + std::to_string(i) + ": " + FieldError;
            return false;
        }

        if (const std::string* Mode = Find(F, "rollMode"))
        {
            if (*Mode == "worldBank")
            {
                A.RollMode = ERollMode::WorldBank;
            }
            else if (*Mode == "followsTorsion")
            {
                A.RollMode = ERollMode::FollowsTorsion;
            }
            else if (*Mode != "pathRelative")
            {
                OutError = "segment " + std::to_string(i) + " has unknown rollMode \"" + *Mode
                           + "\"; roll would be measured from the wrong reference";
                return false;
            }
        }

        // ---- THE DEVICE. Every field optional, so a track stored before the
        // control layer existed still loads as the unpowered geometry it was.
        if (const std::string* Zone = Find(F, "zone"))
        {
            if (!ZoneFromName(*Zone, A.Zone))
            {
                // REFUSED, exactly as an unknown kind is. Reading it as unpowered
                // would open somebody's ride with a brake missing from the middle
                // of it — a file that loads, looks right, and cannot stop a train.
                OutError = "segment " + std::to_string(i) + " has unknown zone \"" + *Zone
                           + "\". Refusing: reading it as plain track would silently "
                             "remove a device from the ride.";
                return false;
            }
        }
        bOk = ReadNumber(F, "zoneSpeed", A.ZoneSpeed, false, FieldError)
              && ReadNumber(F, "zoneAccel", A.ZoneAccel, false, FieldError)
              && ReadNumber(F, "zoneDecel", A.ZoneDecel, false, FieldError)
              && ReadNumber(F, "zoneBrakeDecel", A.ZoneBrakeDecel, false, FieldError);
        if (!bOk)
        {
            OutError = "segment " + std::to_string(i) + ": " + FieldError;
            return false;
        }

        // A FLAG IS TRUE OR IT IS ABSENT. "false" never gets written, so anything
        // other than "true" here is a file saying something this build does not
        // understand — and a mistyped flag read as false is a catch that is not
        // there, which is the one direction a safety device must not fail in.
        auto ReadFlag = [&](const char* Key, bool& Field) {
            const std::string* V = Find(F, Key);
            if (V == nullptr) { return true; }
            if (*V == "true") { Field = true; return true; }
            if (*V == "false") { Field = false; return true; }
            OutError = "segment " + std::to_string(i) + ": \"" + Key + "\" is \"" + *V
                       + "\", which is not a flag";
            return false;
        };
        if (!ReadFlag("antiRollback", A.bAntiRollback)
            || !ReadFlag("startsNewDevice", A.bStartsNewDevice))
        {
            return false;
        }

        Out.Segments.push_back(A);
    }

    return true;
}
