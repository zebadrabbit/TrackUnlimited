// TrackUnlimited Tier 2: the PROCESS IMAGE.
// Plain C++17, no dependencies.
//
// The named snapshot the scan already takes. `CLAUDE.md` constraint 7 decided
// that a Tier 2 override is a pure expression over this image — and the image is
// THE LARGER HALF of that work, needed whichever language ever wins, so none of
// it is speculative.
//
// ===================== WHAT A PROCESS IMAGE IS =====================
//
// A real PLC does not read a sensor when your program mentions it. It copies
// every input into memory at the top of the scan, executes against that copy,
// and writes every output at the bottom. IEC 61131-3 calls those %I and %Q, and
// the reason is not speed: it is that EVERY RUNG SEES THE SAME WORLD. A program
// that re-read an input halfway through could decide two contradictory things
// about one train in one scan.
//
// This project already scans inputs once at the top of the frame for exactly that
// reason. What was missing was a NAME for the snapshot, and that name is the
// whole interface a script ever gets.
//
// ===================== BIND ONCE, INDEX THEREAFTER =====================
//
// Names are resolved to SLOT NUMBERS when an expression is parsed, and the scan
// only ever writes slots. That is how a real PLC works — the symbol table is a
// compile-time artefact and the runtime addresses memory — and it is why filling
// this 240 times a second costs a vector store per field rather than a hash of a
// string.
//
// A slot's index is arithmetic: base of the domain, plus the element, times the
// field count. So the caller writes `SetZone(3, EZoneField::Output, v)` and never
// sees an index, and the parser sees `zone[3].output` and never sees a value.
//
// ===================== THE SHAPE IS GENERATED, NOT MAINTAINED =====================
//
// `Describe()` enumerates every slot with its name and type at runtime. Editor
// autocomplete, the variable reference and the assert suite all read that rather
// than a hand-kept list, because a hand-kept list drifts the first time somebody
// adds a field — which is the on-ramp requirement from `CLAUDE.md` § "The control
// layer is a LAYER OF CHOICE".
//
// Units: metres, seconds, m/s, as everywhere else here.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// BOOL and REAL only. IEC 61131-3 has a dozen more — SINT through LREAL, TIME,
// STRING — and every one of them is a conversion rule and a promotion table for
// no behaviour a permissive needs. A permissive is a boolean over comparisons of
// measurements, and those are the two types it takes.
//
// ponytail: no INT, so MOD is fmod. Named as a limit rather than hidden, and the
// day something wants bit masks or an integer counter is the day to add it.
enum class EPlcType
{
    Bool,
    Real,
};

// One value in the image. Deliberately not a variant: two words and no
// allocation, so a whole image is one contiguous block a scan can stride through.
struct FPlcValue
{
    EPlcType Type = EPlcType::Bool;
    bool bBool = false;
    double Real = 0.0;

    static FPlcValue Boolean(bool V) { FPlcValue O; O.Type = EPlcType::Bool; O.bBool = V; return O; }
    static FPlcValue Number(double V) { FPlcValue O; O.Type = EPlcType::Real; O.Real = V; return O; }

    // For a caller that has already checked the type, or does not care which of
    // the two carried the truth. Zero is FALSE, everything else is TRUE — the
    // ST rule, not C's, and it is the same rule either way.
    bool AsBool() const { return Type == EPlcType::Bool ? bBool : Real != 0.0; }
    double AsReal() const { return Type == EPlcType::Real ? Real : (bBool ? 1.0 : 0.0); }
};

// ===================== THE DOMAINS =====================
//
// Every field below is something the model ALREADY computes every scan. Nothing
// here is a new measurement, and nothing here is a position: Tier 2 gets what the
// controller gets, which is switch state, drive readings and derived block state.
//
// A train's arc length is deliberately ABSENT. The interlocking is still handed a
// span internally and `SIGNALLING.md` records that as a cheat; exposing it to
// scripts would make the cheat load-bearing and permanent.

enum class ERideField
{
    Trains,           // REAL: how many are on the ride
    EStop,            // BOOL: latched emergency stop
    OutputsEnabled,   // BOOL: the controller is permitting commands
    Scan,             // REAL: scan counter, for a script that wants to see time pass
    Count
};

enum class EBlockField
{
    Clear,            // BOOL
    Occupied,         // BOOL
    Buffer,           // BOOL: the overlap held after a train physically exits
    Count
};

enum class EZoneField
{
    Commanded,        // REAL m/s: what the PLC wrote
    Output,           // REAL m/s: what the drive ramped to
    Actual,           // REAL m/s: what the motor is turning at
    Load,             // REAL 0..1: torque
    Ready,            // BOOL: output has reached command and nothing is faulted
    Faulted,          // BOOL
    Holding,          // BOOL: this device can hold a train — pads AND tyres
    Count
};

enum class EPlatformField
{
    Ready,            // BOOL: the whole station sequence says go
    InPosition,       // BOOL: a train is stopped on the mark
    RestraintsLocked, // BOOL: every group reports locked
    GatesClosed,      // BOOL
    Phase,            // REAL: EStationPhase as a number
    Count
};

enum class ETrainField
{
    Speed,            // REAL m/s
    Moving,           // BOOL
    Count
};

// The image itself: a shape declared once when the layout is built, and a block
// of values the scan fills.
class FProcessImage
{
public:
    // DECLARED FROM THE LAYOUT, which is what makes the names match the ride. A
    // layout with six blocks has `block[0]` through `block[5]` and nothing else,
    // so an override written against a different track fails to BIND rather than
    // silently reading a slot that means something else now — the same argument
    // as the PLC's program identity refusing to run against the wrong layout.
    void Declare(std::size_t Blocks, std::size_t Zones, std::size_t Platforms, std::size_t Trains)
    {
        NumBlock = Blocks;
        NumZone = Zones;
        NumPlatform = Platforms;
        NumTrain = Trains;

        RideBase = 0;
        BlockBase = RideBase + F(ERideField::Count);
        ZoneBase = BlockBase + NumBlock * F(EBlockField::Count);
        PlatformBase = ZoneBase + NumZone * F(EZoneField::Count);
        TrainBase = PlatformBase + NumPlatform * F(EPlatformField::Count);
        const std::size_t Total = TrainBase + NumTrain * F(ETrainField::Count);

        Slot.assign(Total, FPlcValue());
        // Types are a property of the SHAPE, not of whatever was last written, so
        // they are set here once. A slot that changed type when the scan wrote it
        // would make a bound expression's type-check meaningless.
        for (std::size_t i = 0; i < Total; ++i)
        {
            Slot[i] = TypeOfSlot(i) == EPlcType::Bool ? FPlcValue::Boolean(false)
                                                      : FPlcValue::Number(0.0);
        }
    }

    std::size_t NumSlots() const { return Slot.size(); }
    std::size_t NumBlocks() const { return NumBlock; }
    std::size_t NumZones() const { return NumZone; }
    std::size_t NumPlatforms() const { return NumPlatform; }
    std::size_t NumTrains() const { return NumTrain; }

    // ---- Writing, by domain. The scan calls these; nothing computes an index.

    void SetRide(ERideField Fld, FPlcValue V) { Put(RideBase + F(Fld), V); }
    void SetBlock(std::size_t i, EBlockField Fld, FPlcValue V)
    {
        if (i < NumBlock) { Put(BlockBase + i * F(EBlockField::Count) + F(Fld), V); }
    }
    void SetZone(std::size_t i, EZoneField Fld, FPlcValue V)
    {
        if (i < NumZone) { Put(ZoneBase + i * F(EZoneField::Count) + F(Fld), V); }
    }
    void SetPlatform(std::size_t i, EPlatformField Fld, FPlcValue V)
    {
        if (i < NumPlatform) { Put(PlatformBase + i * F(EPlatformField::Count) + F(Fld), V); }
    }
    void SetTrain(std::size_t i, ETrainField Fld, FPlcValue V)
    {
        if (i < NumTrain) { Put(TrainBase + i * F(ETrainField::Count) + F(Fld), V); }
    }

    // ---- Reading, by slot. What a bound expression does, and all it can do.

    const FPlcValue& At(std::size_t SlotIndex) const { return Slot[SlotIndex]; }

    // ---- Binding: name to slot, ONCE, at parse time.
    //
    // Case-insensitive, because IEC 61131-3 identifiers are. Returns false for an
    // unknown name rather than inventing a slot — an override that misspells a
    // block must fail to load, not read FALSE for ever.
    bool Find(const std::string& Name, std::size_t& OutSlot, EPlcType& OutType) const
    {
        const std::string Key = Lower(Name);
        for (std::size_t i = 0; i < Slot.size(); ++i)
        {
            if (Lower(NameOfSlot(i)) == Key)
            {
                OutSlot = i;
                OutType = TypeOfSlot(i);
                return true;
            }
        }
        return false;
    }

    // ---- The shape, ENUMERATED rather than documented.
    //
    // Autocomplete, the variable reference and the assert suite all read this.
    // A hand-kept list drifts the first time a field is added, and the drift is
    // silent — which is why there is no hand-kept list anywhere in this project.
    struct FSlotInfo
    {
        std::size_t Index = 0;
        std::string Name;
        EPlcType Type = EPlcType::Bool;
    };

    std::vector<FSlotInfo> Describe() const
    {
        std::vector<FSlotInfo> Out;
        Out.reserve(Slot.size());
        for (std::size_t i = 0; i < Slot.size(); ++i)
        {
            Out.push_back({i, NameOfSlot(i), TypeOfSlot(i)});
        }
        return Out;
    }

    std::string NameOfSlot(std::size_t i) const
    {
        if (i < BlockBase)
        {
            return std::string("ride.") + RideFieldName(static_cast<ERideField>(i - RideBase));
        }
        if (i < ZoneBase)
        {
            const std::size_t k = i - BlockBase;
            return Element("block", k / F(EBlockField::Count))
                 + BlockFieldName(static_cast<EBlockField>(k % F(EBlockField::Count)));
        }
        if (i < PlatformBase)
        {
            const std::size_t k = i - ZoneBase;
            return Element("zone", k / F(EZoneField::Count))
                 + ZoneFieldName(static_cast<EZoneField>(k % F(EZoneField::Count)));
        }
        if (i < TrainBase)
        {
            const std::size_t k = i - PlatformBase;
            return Element("platform", k / F(EPlatformField::Count))
                 + PlatformFieldName(static_cast<EPlatformField>(k % F(EPlatformField::Count)));
        }
        const std::size_t k = i - TrainBase;
        return Element("train", k / F(ETrainField::Count))
             + TrainFieldName(static_cast<ETrainField>(k % F(ETrainField::Count)));
    }

    EPlcType TypeOfSlot(std::size_t i) const
    {
        if (i < BlockBase)
        {
            return static_cast<ERideField>(i - RideBase) == ERideField::Trains
                || static_cast<ERideField>(i - RideBase) == ERideField::Scan
                ? EPlcType::Real : EPlcType::Bool;
        }
        if (i < ZoneBase)
        {
            return EPlcType::Bool;                       // every block field is a lamp
        }
        if (i < PlatformBase)
        {
            const EZoneField Fld = static_cast<EZoneField>((i - ZoneBase) % F(EZoneField::Count));
            return Fld == EZoneField::Ready || Fld == EZoneField::Faulted
                || Fld == EZoneField::Holding ? EPlcType::Bool : EPlcType::Real;
        }
        if (i < TrainBase)
        {
            const EPlatformField Fld =
                static_cast<EPlatformField>((i - PlatformBase) % F(EPlatformField::Count));
            return Fld == EPlatformField::Phase ? EPlcType::Real : EPlcType::Bool;
        }
        const ETrainField Fld = static_cast<ETrainField>((i - TrainBase) % F(ETrainField::Count));
        return Fld == ETrainField::Speed ? EPlcType::Real : EPlcType::Bool;
    }

private:
    template <typename E> static std::size_t F(E V) { return static_cast<std::size_t>(V); }

    // A write that disagrees with the slot's declared type is REFUSED rather than
    // coerced. A permissive that silently read 1.0 where it expected TRUE would be
    // the kind of quiet type confusion this whole design exists to avoid.
    void Put(std::size_t i, const FPlcValue& V)
    {
        if (i < Slot.size() && V.Type == Slot[i].Type)
        {
            Slot[i] = V;
        }
    }

    static std::string Element(const char* Domain, std::size_t Index)
    {
        return std::string(Domain) + "[" + std::to_string(Index) + "].";
    }

    static std::string Lower(const std::string& In)
    {
        std::string Out = In;
        for (char& C : Out)
        {
            if (C >= 'A' && C <= 'Z') { C = static_cast<char>(C - 'A' + 'a'); }
        }
        return Out;
    }

    static const char* RideFieldName(ERideField F2)
    {
        switch (F2)
        {
        case ERideField::Trains:         return "trains";
        case ERideField::EStop:          return "estop";
        case ERideField::OutputsEnabled: return "outputs_enabled";
        default:                         return "scan";
        }
    }
    static const char* BlockFieldName(EBlockField F2)
    {
        switch (F2)
        {
        case EBlockField::Clear:    return "clear";
        case EBlockField::Occupied: return "occupied";
        default:                    return "buffer";
        }
    }
    static const char* ZoneFieldName(EZoneField F2)
    {
        switch (F2)
        {
        case EZoneField::Commanded: return "commanded";
        case EZoneField::Output:    return "output";
        case EZoneField::Actual:    return "actual";
        case EZoneField::Load:      return "load";
        case EZoneField::Ready:     return "ready";
        case EZoneField::Faulted:   return "faulted";
        default:                    return "holding";
        }
    }
    static const char* PlatformFieldName(EPlatformField F2)
    {
        switch (F2)
        {
        case EPlatformField::Ready:            return "ready";
        case EPlatformField::InPosition:       return "in_position";
        case EPlatformField::RestraintsLocked: return "restraints_locked";
        case EPlatformField::GatesClosed:      return "gates_closed";
        default:                               return "phase";
        }
    }
    static const char* TrainFieldName(ETrainField F2)
    {
        return F2 == ETrainField::Speed ? "speed" : "moving";
    }

    std::vector<FPlcValue> Slot;
    std::size_t NumBlock = 0, NumZone = 0, NumPlatform = 0, NumTrain = 0;
    std::size_t RideBase = 0, BlockBase = 0, ZoneBase = 0, PlatformBase = 0, TrainBase = 0;
};

// ponytail: Find() is a linear scan over the slot names. It runs at PARSE time,
// once per identifier in an override, on a list of a few hundred — so a map would
// be a cache for a cost nothing has measured. If a track ever has enough blocks
// for this to matter, build the map inside Declare where the names are already
// being generated.
