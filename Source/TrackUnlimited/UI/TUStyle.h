// TrackUnlimited: the design language, as a Slate style set.
//
// ===================== WHY THIS IS C++ AND NOT AN ASSET =====================
//
// `UI_CONVENTIONS.md` decided UMG-primary, and this does not contradict that:
// UMG widgets take their look from `FButtonStyle`, `FTextBlockStyle` and friends
// exactly as Slate widgets do. Putting the palette here rather than in a
// `.uasset` means the design language is TEXT — it diffs, it reviews, and a
// contributor to an MIT project can see what changed. The layout assets stay
// assets; the language they are drawn in does not have to be.
//
// It is also what a skin would swap, if one is ever wanted: the reskinning
// section of that document is explicit that a style SET is the mechanism and
// that a parallel widget tree is not.
//
// ===================== EVERY NUMBER HERE IS A DECISION ALREADY MADE =====================
//
// Nothing in this file is invented. The surface ramp, the two palettes and the
// rule that separates them are all from `UI_CONVENTIONS.md` § 3, and the reason
// there are TWO palettes is worth keeping in front of whoever edits this:
//
//   THE OPERATOR PANEL KEEPS HARDWARE COLOURS. Red, green and amber as
//   photographed on three real consoles. "Fixing" them for accessibility would
//   make the panel LESS faithful, which is the entire point of it.
//
//   ANALYSIS TRACES USE OKABE-ITO, the standard colourblind-safe qualitative
//   palette. Do not invent one; this exists and is designed for this.
//
// What makes both safe is the rule they share, which is stronger than either
// palette: HUE IS NEVER THE ONLY CHANNEL. Position, legend and text carry the
// meaning, and colour is the fast path to something already stated another way.

#pragma once

#include "CoreMinimal.h"
#include "Styling/SlateStyle.h"
#include "Styling/SlateTypes.h"

class FTUStyle
{
public:
	/** Registers the style set. Idempotent, so a hot reload cannot double-register. */
	static void Initialise();
	static void Shutdown();

	static const ISlateStyle& Get();
	static FName StyleSetName();

	// ===================== THE SURFACE RAMP =====================
	//
	// Five steps, and no more. A ramp with a dozen surfaces is a ramp nobody can
	// keep consistent, and the failure shows up as panels that are almost but not
	// quite the same grey.
	static const FLinearColor Background;      // #14171A
	static const FLinearColor Panel;           // #1E2226
	static const FLinearColor Border;          // #2E343A
	static const FLinearColor TextPrimary;     // #E6E9EC
	static const FLinearColor TextSecondary;   // #9AA3AB

	// ===================== THE CONSOLE PALETTE =====================
	//
	// Hardware colours. Photographed, not chosen.
	static const FLinearColor LampClear;       // green   — block clear, drive running
	static const FLinearColor LampOccupied;    // amber   — occupied, held, waiting
	static const FLinearColor LampFault;       // red     — fault, violation, E-stop
	static const FLinearColor LampMeasured;    // cyan    — a measured number, not a state

	// ===================== OKABE-ITO, FOR ANALYSIS =====================
	//
	// Blue #0072B2 is deliberately absent: about 3:1 against the background,
	// which is too weak for a 2 px line. Vermillion and yellow are the next two
	// if a fifth channel ever appears.
	static const FLinearColor TraceVerticalG;  // sky blue      #56B4E9
	static const FLinearColor TraceLateralG;   // orange        #E69F00
	static const FLinearColor TraceSpeed;      // bluish green  #009E73
	static const FLinearColor TraceRollRate;   // reddish purple #CC79A7

	/**
	 * WCAG AA, checked rather than asserted.
	 *
	 * The convention document says to run a contrast check on any new colour
	 * before it ships, and a rule nobody can run is a rule that decays. This is
	 * the relative-luminance ratio from WCAG 2.1 — 4.5:1 for body text, 3:1 for
	 * large text and graphical elements.
	 *
	 * Public because the assert suite uses it: every pairing this file ships is
	 * checked in `Prototypes/Shell/test_uistyle.cpp`, so a palette edit that
	 * breaks contrast fails a build rather than somebody's eyes.
	 */
	static double ContrastRatio(const FLinearColor& A, const FLinearColor& B);

	/**
	 * A 4K MONITOR SHOWS MORE ROWS, NOT BIGGER ONES.
	 *
	 * The failure mode for a data-dense tool is a 2160-tall display scaling to
	 * 2.0 and fitting exactly as many segment rows as the 1080p laptop did, with
	 * all that resolution spent on larger glyphs. So the curve clamps near 1.5
	 * and the surplus pixels become rows.
	 *
	 * Shortest side, per the decision — and the corollary that makes it work is
	 * in the widgets rather than here: NO LIST HARDCODES A ROW COUNT.
	 */
	static float DpiScaleForShortestSide(int32 ShortestSidePixels);

private:
	static TSharedPtr<FSlateStyleSet> Instance;
	static TSharedRef<FSlateStyleSet> Create();
};
