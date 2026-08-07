// TrackUnlimited Tier 3: the effect device layer.
// Plain C++17, no dependencies.
//
// `FShowBus` decides WHAT fires. This is what it fires INTO — the actual devices,
// modelled as the protocol they speak.
//
// ===================== DMX512, MODELLED LITERALLY AND INTERNALLY =====================
//
// Universes, 512 slots, a start code, fixtures at addresses with channel maps.
// This is the vocabulary a real lighting programmer already speaks, and adopting
// it costs almost nothing — but there is NO NETWORK I/O here and there is not
// going to be. See `CONTROL_ARCHITECTURE.md` § "what this is not": a simulator
// that drove real fixtures would be a lighting console with a coaster attached,
// and the failure modes of that are somebody else's to own.
//
// ===================== DMX HAS NO FADES =====================
//
// The single fact people get wrong. A DMX packet is a set of ABSOLUTE VALUES, and
// there is no "fade to 50% over 2 seconds" on the wire. The console computes every
// intermediate value and streams them at frame rate; any smoothing beyond that is
// the fixture's own slew behaviour.
//
// A full 513-slot packet is about 22.7 ms, so roughly 44 packets a second is the
// hard ceiling, and shorter packets go proportionally faster. That means a cue's
// resolution is a frame, not a millisecond — which is exactly the same discipline
// the control scan already runs on, so it needs no new machinery.
//
// ===================== ARM PLUS LEVEL =====================
//
// The characteristic shape of effect gear, and it exists precisely so that one
// stray channel cannot start an effect. The MDG ATMe hazer — an industry standard
// unit — is three channels: unit power, haze output 0-255, and a separate haze
// ENABLE that must be held above half. Two of the three have to be right before
// anything comes out.
//
// And FLAME IS NOT FIRED BY THE SHOW CONTROLLER AT ALL. NFPA 160 puts a
// safety-rated flame effect controller and a fail-safe POSITIVE MANUAL ENABLE — a
// human-held enable, actively asserted — between the request and the gas. The show
// controller sends its cue into a circuit that may simply be open, and does not
// know. That is the same shape as `FShowBus`'s hazard permissive, one layer down.
//
// Units: seconds.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// One DMX universe: 512 slots after the start code.
//
// Slot 0 is the START CODE and is not a channel — it is 0x00 for dimmer data and
// other values for other protocols riding the same wire. Modelled rather than
// skipped, because addressing is 1-based against slots and the off-by-one that
// results is the classic DMX mistake.
class FDmxUniverse
{
public:
    static constexpr std::size_t Slots = 512;

    void Clear() { Slot.assign(Slots, 0); }
    FDmxUniverse() { Clear(); }

    // ADDRESSES ARE 1-BASED, as every fixture's DIP switches and menu are. A
    // fixture patched at 1 owns slot 1, and slot 0 is the start code.
    void Set(std::size_t Address, std::uint8_t Value)
    {
        if (Address >= 1 && Address <= Slots) { Slot[Address - 1] = Value; }
    }
    std::uint8_t Get(std::size_t Address) const
    {
        return Address >= 1 && Address <= Slots ? Slot[Address - 1] : 0;
    }

    std::uint8_t StartCode() const { return 0x00; }   // dimmer data

    // How long this many slots takes on the wire, at 250 kbit/s with the start
    // code, start and stop bits and the break. A full packet is ~22.7 ms, so a
    // universe cannot be refreshed faster than about 44 times a second.
    //
    // Here so a cue can be told it is asking for something the wire cannot do,
    // rather than being quietly ignored.
    static double PacketSeconds(std::size_t SlotsSent = Slots)
    {
        const std::size_t Bytes = SlotsSent + 1;             // plus the start code
        return 0.000176 + static_cast<double>(Bytes) * 44e-6;   // break + 44 us/slot
    }
    static double MaxRefreshHz(std::size_t SlotsSent = Slots)
    {
        return 1.0 / PacketSeconds(SlotsSent);
    }

private:
    std::vector<std::uint8_t> Slot;
};

// What a fixture does when the data stops — a cable pulled, a console crashed, a
// switch rebooted. REAL UNITS MAKE THIS SELECTABLE and both choices are defensible,
// which is why both are modelled rather than one being assumed.
enum class EDmxDataLoss
{
    // Everything to zero. Right for haze and light: the effect stops.
    Shutdown,
    // Keep the last values received. Right for architectural fixtures nobody
    // wants going dark, and WRONG for anything hazardous — a flame fixture
    // holding its last value is a flame nobody is commanding.
    HoldLast,
};

// A fixture: a name, an address, a channel count, and a policy.
//
// `Channel` is 1-based WITHIN the fixture, so channel 1 of a fixture at address
// 100 is slot 100. That doubled 1-based-ness is exactly what real patching is and
// exactly where people slip.
struct FDmxFixture
{
    std::string Name;
    std::size_t Address = 1;
    std::size_t Channels = 1;
    EDmxDataLoss OnDataLoss = EDmxDataLoss::Shutdown;

    std::size_t SlotOf(std::size_t Channel) const { return Address + Channel - 1; }
    std::size_t LastSlot() const { return Address + Channels - 1; }

    // Two fixtures whose channel ranges overlap is the most common patching error
    // there is, and it produces a light that flickers when a completely different
    // one is cued.
    bool Overlaps(const FDmxFixture& Other) const
    {
        return Address <= Other.LastSlot() && Other.Address <= LastSlot();
    }
};

// ===================== THE ONE PREDICATE =====================
//
// Every hazardous effect satisfies this and nothing else fires one:
//
//   fire  <=>  showRequest AND safetyPermissive AND armed AND NOT faulted
//              AND withinDutyCycle
//
// Most of what makes a park feel like a park rather than a light show is in the
// last three terms — the ones about the device rather than about the cue.
struct FEffectRequest
{
    bool bShowRequest = false;     // the cue said fire
    bool bSafetyPermissive = false;// the hard interlock, which show cannot see
    bool bArmed = false;           // the arm channel, held above threshold
    double Level = 0.0;            // 0..1, the level channel
};

// An effect device with a DUTY CYCLE, which is the thing a real programmer works
// around and which nothing in this project could previously express.
//
// A cheap fogger stops accepting fire commands while its heater recovers. A
// Le Maitre Salamander flame projector is two channels — a hot surface igniter
// that only energises above 99%, and fire above 50% — with a mandatory ~10 s
// igniter heat-up during which firing is inhibited, a 30 s maximum continuous
// solenoid, and recommended 0.5-1 s bursts.
//
// So an effect is not a lamp. It has three states nothing else here has: WARMING,
// READY, and RECOVERING, and a cue that arrives in the wrong one is simply not
// honoured.
class FEffectDevice
{
public:
    struct FSpec
    {
        double WarmupSeconds = 0.0;      // igniter heat-up; firing inhibited
        double MaxBurstSeconds = 1e9;    // solenoid duty limit
        double RecoverySeconds = 0.0;    // before it will fire again
        bool bHazardous = false;         // needs the permissive and the arm
    };

    explicit FEffectDevice(const FSpec& In) : Spec(In)
    {
        Warming = Spec.WarmupSeconds;
    }

    // THE PREDICATE, once per scan. Returns whether the device is actually
    // producing anything, which is not the same as whether it was asked to.
    bool Scan(double DeltaSeconds, const FEffectRequest& In)
    {
        if (Warming > 0.0)
        {
            // A REAL IGNITER TAKES TEN SECONDS AND CANNOT BE HURRIED. A cue during
            // warm-up is not queued and not honoured late — it is dropped, because
            // firing a flame effect ten seconds after the music wanted it is worse
            // than not firing it.
            Warming -= DeltaSeconds;
            ++Dropped;
            Firing = false;
            return false;
        }
        if (Recovering > 0.0)
        {
            Recovering -= DeltaSeconds;
            if (In.bShowRequest) { ++Dropped; }
            Firing = false;
            return false;
        }

        const bool bWanted = In.bShowRequest && In.Level > 0.0;
        const bool bAllowed = !Spec.bHazardous
            || (In.bSafetyPermissive && In.bArmed);

        if (bWanted && bAllowed)
        {
            Firing = true;
            Burst += DeltaSeconds;
            // THE SOLENOID DUTY LIMIT, and it is the DEVICE that enforces it. A
            // console operator holding a fader down does not get 30 seconds of
            // flame because they asked for it; the fixture stops and needs its
            // recovery. Modelling this in the cue layer would let a bad cue
            // destroy hardware, which is the wrong place for the rule.
            if (Burst >= Spec.MaxBurstSeconds)
            {
                Firing = false;
                Burst = 0.0;
                Recovering = Spec.RecoverySeconds;
                ++DutyCutoffs;
            }
        }
        else
        {
            if (Firing)
            {
                // Released cleanly. The recovery still applies — a fogger's heater
                // does not care that you let go politely.
                Recovering = Spec.RecoverySeconds;
            }
            if (bWanted && !bAllowed) { ++Inhibited; }
            Firing = false;
            Burst = 0.0;
        }
        return Firing;
    }

    bool IsFiring() const { return Firing; }
    bool IsWarming() const { return Warming > 0.0; }
    bool IsRecovering() const { return Recovering > 0.0; }
    bool IsReady() const { return Warming <= 0.0 && Recovering <= 0.0; }

    // Counted, never acted on — the same rule as the drives' fault reporting.
    // A show that keeps asking for something the device will not do is a show
    // worth fixing, and the count is how anybody would know.
    std::size_t TimesInhibited() const { return Inhibited; }
    std::size_t TimesDropped() const { return Dropped; }
    std::size_t TimesDutyCut() const { return DutyCutoffs; }

private:
    FSpec Spec;
    double Warming = 0.0;
    double Recovering = 0.0;
    double Burst = 0.0;
    bool Firing = false;
    std::size_t Inhibited = 0, Dropped = 0, DutyCutoffs = 0;
};

// A patch: every fixture in a universe, and the rig's own sanity.
class FDmxPatch
{
public:
    // REFUSES an overlap rather than accepting it. Two fixtures sharing a slot is
    // the most common patching error there is, and it produces a light that
    // flickers when a different one is cued — a symptom nobody traces back to
    // addressing. Report, never repair: the fix is a different address.
    bool Add(const FDmxFixture& F)
    {
        if (F.Address < 1 || F.Channels < 1 || F.LastSlot() > FDmxUniverse::Slots)
        {
            return false;
        }
        for (const FDmxFixture& Other : Fixture)
        {
            if (F.Overlaps(Other)) { return false; }
        }
        Fixture.push_back(F);
        return true;
    }

    std::size_t Num() const { return Fixture.size(); }
    const FDmxFixture& At(std::size_t i) const { return Fixture[i]; }

    // The highest slot anybody is using, which is what a real console sends
    // rather than always transmitting 512. Shorter packets refresh faster, and on
    // a small rig that is the difference between 44 Hz and 200.
    std::size_t HighestSlotUsed() const
    {
        std::size_t Highest = 0;
        for (const FDmxFixture& F : Fixture)
        {
            if (F.LastSlot() > Highest) { Highest = F.LastSlot(); }
        }
        return Highest;
    }

    // Apply the data-loss policy. Called when the stream stops, which a simulator
    // can do exactly and a real rig discovers the hard way.
    void OnDataLoss(FDmxUniverse& U) const
    {
        for (const FDmxFixture& F : Fixture)
        {
            if (F.OnDataLoss != EDmxDataLoss::Shutdown) { continue; }
            for (std::size_t c = 1; c <= F.Channels; ++c) { U.Set(F.SlotOf(c), 0); }
        }
    }

private:
    std::vector<FDmxFixture> Fixture;
};

// The arm-plus-level idiom, as the two thresholds real gear uses. Above half is
// "on" for an enable channel; the Salamander's igniter wants above 99%, which is
// deliberately almost the top of the range so a stray value cannot reach it.
inline bool DmxAbove(std::uint8_t Value, double Fraction)
{
    return static_cast<double>(Value) / 255.0 > Fraction;
}
inline bool DmxEnabled(std::uint8_t Value) { return Value >= 128; }

// ponytail: no colour mixing, no pan/tilt, no fixture personality files, no
// RDM. Those are how you make a LIGHT do something and they belong with the light
// props in Phase 5.5. What is here is the part that had to be right before any of
// that: addressing, the refresh ceiling, data loss, and the predicate that decides
// whether a hazardous effect fires.
