// TrackUnlimited Phase 3.5: the ride-profile graph, minus the drawing.
// Plain C++17, no engine dependency.
//
// The 2D graph is the view that makes a spike DIAGNOSABLE — speed, felt G on all
// three axes, roll rate and height against arc length, with a scrubber that moves
// the camera to that S. This is everything about it that is arithmetic, and it
// also absorbs the Phase 1 legibility deferral rather than accreting more
// booleans onto the debug draw.
//
// ===================== NICE NUMBERS =====================
//
// Divide a range by a tick count and you get gridlines at 0.37, 0.74, 1.11. A
// person cannot read a value off that, so the axis has to snap to 1, 2 or 5 times
// a power of ten — which is what every plotting library does and what a graph
// drawn by hand does without thinking about it.
//
// The consequence people forget: SNAPPING THE STEP MEANS THE RANGE MUST GROW to a
// multiple of it. An axis with a nice step and a raw range puts its top gridline
// somewhere arbitrary, which is exactly as unreadable.
//
// ===================== ZERO IS NOT OPTIONAL =====================
//
// On a SIGNED channel — lateral G, roll rate, vertical G — the zero line is what
// the trace means anything against. A graph auto-scaled to 0.4..0.9 g looks like
// a mountain range and is a train sitting almost still.
//
// ===================== A FLAT CHANNEL IS NOT AN ERROR =====================
//
// Lateral G on a straight track is exactly zero for its whole length. That is a
// range of nothing, and a graph that divided by it would produce NaN on the one
// layout most likely to be somebody's first.
//
// Units: whatever the channel is in. This layer never converts.

#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

// Snap to 1, 2 or 5 times a power of ten. The classic answer, and it is classic
// because the alternatives are all unreadable.
inline double NiceStep(double Rough)
{
    if (!(Rough > 0.0)) { return 1.0; }
    const double Exp = std::floor(std::log10(Rough));
    const double Pow = std::pow(10.0, Exp);
    const double F = Rough / Pow;              // 1 .. 10
    double Nice;
    if (F <= 1.0)      { Nice = 1.0; }
    else if (F <= 2.0) { Nice = 2.0; }
    else if (F <= 5.0) { Nice = 5.0; }
    else               { Nice = 10.0; }
    return Nice * Pow;
}

struct FAxis
{
    double Min = 0.0;
    double Max = 1.0;
    double Step = 1.0;

    double Span() const { return Max - Min; }
    std::size_t TickCount() const
    {
        return Step > 0.0 ? static_cast<std::size_t>(std::floor(Span() / Step + 0.5)) + 1 : 2;
    }
    double TickAt(std::size_t i) const { return Min + static_cast<double>(i) * Step; }

    // Value to a 0..1 fraction of the axis, which is all a renderer needs. The
    // pixel conversion belongs with the widget, because only it knows how tall it
    // is — and putting it here would bake a coordinate system into the model.
    double Fraction(double Value) const
    {
        return Span() > 0.0 ? (Value - Min) / Span() : 0.5;
    }
    double ValueAt(double Fraction01) const { return Min + Fraction01 * Span(); }
};

// Build an axis over a measured range.
//
// `bIncludeZero` is for SIGNED channels, where the zero line is what the trace
// means anything against. A lateral-G graph auto-scaled to 0.4..0.9 looks like a
// mountain range and is a train sitting almost still.
inline FAxis MakeAxis(double DataMin, double DataMax, int TargetTicks = 6,
                      bool bIncludeZero = true)
{
    FAxis A;
    if (bIncludeZero)
    {
        if (DataMin > 0.0) { DataMin = 0.0; }
        if (DataMax < 0.0) { DataMax = 0.0; }
    }

    // A FLAT CHANNEL IS NOT AN ERROR. Lateral G on a straight track is exactly
    // zero for its whole length — a range of nothing, and dividing by it would
    // produce NaN on the layout most likely to be somebody's first. Give it a
    // unit of room and draw a flat line through the middle, which is the truth.
    if (!(DataMax > DataMin))
    {
        const double Mid = DataMax;
        DataMin = Mid - 0.5;
        DataMax = Mid + 0.5;
    }

    const int N = TargetTicks > 1 ? TargetTicks : 2;
    A.Step = NiceStep((DataMax - DataMin) / static_cast<double>(N - 1));

    // AND THE RANGE GROWS TO A MULTIPLE OF THE STEP. A nice step against a raw
    // range puts the top gridline somewhere arbitrary, which is exactly as
    // unreadable as a nasty step — this is the half people leave out.
    A.Min = std::floor(DataMin / A.Step) * A.Step;
    A.Max = std::ceil(DataMax / A.Step) * A.Step;
    if (!(A.Max > A.Min)) { A.Max = A.Min + A.Step; }
    return A;
}

// One plotted channel. PER-CHANNEL SCALE, which the Phase 1 legibility card
// asked for: speed in m/s and roll rate in deg/s on one shared axis makes the
// speed trace a flat line at the bottom.
struct FGraphChannel
{
    const char* Name = "";
    const char* Unit = "";
    bool bSigned = true;        // does the zero line mean something
    FAxis Axis;

    // ROLL RATE IS A FIRST-CLASS CHANNEL WITH ITS OWN AXIS. It is the one thing
    // no G trace can ever show — felt G models the rider as a point at the
    // heartline, so spinning that point costs exactly nothing.
};

// The horizontal axis is arc length, and the scrubber lives on it.
class FProfileGraph
{
public:
    void SetDomain(double TotalLengthM)
    {
        Total = TotalLengthM > 0.0 ? TotalLengthM : 0.0;
        if (ScrubS > Total) { ScrubS = Total; }
    }
    double TotalLength() const { return Total; }

    // ===================== THE SCRUBBER =====================
    //
    // Screen fraction to arc length and back, and they MUST round-trip: the
    // scrubber's own handle is drawn from the value it just produced, so a
    // mismatch makes the handle drift away from the cursor as you drag.
    double ScrubToS(double Fraction01) const
    {
        const double F = Clamp01(Fraction01);
        return F * Total;
    }
    double SToScrub(double S) const
    {
        return Total > 0.0 ? Clamp01(S / Total) : 0.0;
    }

    // CLAMPED, NOT WRAPPED. Dragging past the end of a circuit must stop at the
    // end rather than jumping to the start — the graph is a plot of one lap, and
    // wrapping would make the trace appear to teleport under the cursor.
    void ScrubTo(double S)
    {
        ScrubS = S < 0.0 ? 0.0 : (S > Total ? Total : S);
    }
    double ScrubbedS() const { return ScrubS; }

    // Sample a trace at the scrubber, by arc length rather than by index. An
    // index-based lookup breaks the moment the sample spacing changes, and the
    // sample spacing is a setting.
    static double SampleAt(const std::vector<double>& Values, double TotalM, double S)
    {
        if (Values.empty() || !(TotalM > 0.0)) { return 0.0; }
        if (Values.size() == 1) { return Values[0]; }
        const double F = Clamp01(S / TotalM) * static_cast<double>(Values.size() - 1);
        const std::size_t I = static_cast<std::size_t>(F);
        if (I + 1 >= Values.size()) { return Values.back(); }
        const double T = F - static_cast<double>(I);
        // INTERPOLATED, because the graph is continuous and the samples are not.
        // Nearest-neighbour makes the readout jump in steps as you drag, which
        // reads as the data being coarse rather than the lookup being lazy.
        return Values[I] * (1.0 - T) + Values[I + 1] * T;
    }

private:
    static double Clamp01(double V) { return V < 0.0 ? 0.0 : (V > 1.0 ? 1.0 : V); }

    double Total = 0.0;
    double ScrubS = 0.0;
};

// The extremes, WITH THEIR LOCATIONS. "4.25 g at S = 310 m" is somewhere to go
// and look; "4.25 g" is trivia — which is the same rule the diagnostics panel
// runs on and the reason both exist.
struct FChannelExtremes
{
    double Min = 0.0, Max = 0.0;
    double MinAtS = 0.0, MaxAtS = 0.0;
    bool bValid = false;
};

inline FChannelExtremes ExtremesOf(const std::vector<double>& Values, double TotalM)
{
    FChannelExtremes E;
    if (Values.empty()) { return E; }
    E.bValid = true;
    E.Min = E.Max = Values[0];
    const double Step = Values.size() > 1
        ? TotalM / static_cast<double>(Values.size() - 1) : 0.0;
    for (std::size_t i = 0; i < Values.size(); ++i)
    {
        const double S = static_cast<double>(i) * Step;
        if (Values[i] < E.Min) { E.Min = Values[i]; E.MinAtS = S; }
        if (Values[i] > E.Max) { E.Max = Values[i]; E.MaxAtS = S; }
    }
    return E;
}

// ponytail: no log axes, no dual-Y, no channel groups. A ride profile is a
// handful of channels over a few hundred metres and every one of those is a
// feature for a data set this does not have.
