#include "TUStyle.h"

#include "Styling/SlateStyleRegistry.h"
#include "Styling/CoreStyle.h"
#include "Fonts/SlateFontInfo.h"

TSharedPtr<FSlateStyleSet> FTUStyle::Instance = nullptr;

// ===================== THE PALETTE =====================
//
// FLinearColor::FromSRGBColor, not the raw constructor. The hex in
// UI_CONVENTIONS.md is an sRGB value as a designer or a browser would read it,
// and Slate works in linear — writing 0x14/255 into a linear channel produces a
// noticeably different, washed-out colour, which is the classic way a palette
// arrives on screen not matching the document that specifies it.

namespace
{
	FLinearColor Hex(uint8 R, uint8 G, uint8 B, uint8 A = 255)
	{
		return FLinearColor::FromSRGBColor(FColor(R, G, B, A));
	}
}

const FLinearColor FTUStyle::Background    = Hex(0x14, 0x17, 0x1A);
const FLinearColor FTUStyle::Panel         = Hex(0x1E, 0x22, 0x26);
const FLinearColor FTUStyle::Border        = Hex(0x2E, 0x34, 0x3A);
const FLinearColor FTUStyle::TextPrimary   = Hex(0xE6, 0xE9, 0xEC);
const FLinearColor FTUStyle::TextSecondary = Hex(0x9A, 0xA3, 0xAB);

// Hardware colours, from the console photographs. See SIGNALLING.md
// § "What three real consoles taught it".
const FLinearColor FTUStyle::LampClear     = Hex(89, 209, 115);
const FLinearColor FTUStyle::LampOccupied  = Hex(250, 158, 41);
const FLinearColor FTUStyle::LampFault     = Hex(215, 60, 50);
const FLinearColor FTUStyle::LampMeasured  = Hex(89, 189, 255);

// Okabe-Ito.
const FLinearColor FTUStyle::TraceVerticalG = Hex(0x56, 0xB4, 0xE9);
const FLinearColor FTUStyle::TraceLateralG  = Hex(0xE6, 0x9F, 0x00);
const FLinearColor FTUStyle::TraceSpeed     = Hex(0x00, 0x9E, 0x73);
const FLinearColor FTUStyle::TraceRollRate  = Hex(0xCC, 0x79, 0xA7);

FName FTUStyle::StyleSetName()
{
	return FName(TEXT("TrackUnlimitedStyle"));
}

void FTUStyle::Initialise()
{
	// IDEMPOTENT. A hot reload or a second module startup must not register a
	// second style set under the same name — Slate keeps both and which one a
	// widget gets is undefined.
	if (Instance.IsValid())
	{
		return;
	}
	Instance = Create();
	FSlateStyleRegistry::RegisterSlateStyle(*Instance);
}

void FTUStyle::Shutdown()
{
	if (Instance.IsValid())
	{
		FSlateStyleRegistry::UnRegisterSlateStyle(*Instance);
		Instance.Reset();
	}
}

const ISlateStyle& FTUStyle::Get()
{
	// Created on demand rather than asserting, because a widget constructed
	// before module startup is a real ordering hazard in UE and the useful
	// behaviour is to work rather than to explain why it did not.
	if (!Instance.IsValid())
	{
		Initialise();
	}
	return *Instance;
}

double FTUStyle::ContrastRatio(const FLinearColor& A, const FLinearColor& B)
{
	// WCAG 2.1 relative luminance. The channel values here are ALREADY linear,
	// which is exactly why the palette above goes through FromSRGBColor: the
	// WCAG formula is defined on linearised sRGB, so a colour built with raw
	// /255 values would compute a ratio for a colour nobody is looking at.
	auto Luminance = [](const FLinearColor& C)
	{
		return 0.2126 * C.R + 0.7152 * C.G + 0.0722 * C.B;
	};
	const double La = Luminance(A);
	const double Lb = Luminance(B);
	const double Hi = FMath::Max(La, Lb);
	const double Lo = FMath::Min(La, Lb);
	return (Hi + 0.05) / (Lo + 0.05);
}

float FTUStyle::DpiScaleForShortestSide(int32 ShortestSidePixels)
{
	// Below the design target, scale DOWN proportionally so the 1600x900 floor
	// still fits a designed 1080p layout. Above it, rise gently and clamp at
	// 1.5 — which is the whole point: a 2160-tall display gets more rows, not
	// larger glyphs.
	const float S = static_cast<float>(ShortestSidePixels);
	if (S <= 0.f)
	{
		return 1.f;
	}
	if (S < 1080.f)
	{
		// 900 -> 0.833, and no lower than that: the 1600x900 floor is supported,
		// not merely tolerated, and text below about 0.8 stops being legible.
		return FMath::Max(0.8f, S / 1080.f);
	}
	// 1080 -> 1.0, 1440 -> 1.19, 2160 -> 1.5 and held there.
	const float Raw = 1.f + (S - 1080.f) / (2160.f - 1080.f) * 0.5f;
	return FMath::Min(1.5f, Raw);
}

TSharedRef<FSlateStyleSet> FTUStyle::Create()
{
	TSharedRef<FSlateStyleSet> S = MakeShared<FSlateStyleSet>(StyleSetName());

	// ===================== TYPE =====================
	//
	// TABULAR FIGURES ARE A REQUIREMENT AND ARE NOT MET HERE, which
	// UI_CONVENTIONS.md leaves open as "not decided: font (needs seeing at 4K;
	// must have tabular figures)".
	//
	// Every panel in this project is a column of numbers that CHANGE every
	// frame — commanded against output against motor, block counts, arc lengths.
	// With proportional figures a 1 is narrower than a 7, so the column jitters
	// as the values move, and a readout that dances is one people stop reading.
	//
	// Roboto ships with the engine and its digits are proportional. This uses it
	// so there is something on screen, and the gap is recorded rather than
	// hidden: the fix is a font asset with `tnum` (JetBrains Mono, IBM Plex Mono
	// and Roboto Mono are all open-licensed and would do), which needs the
	// developer at the editor because a font is an asset.
	const FSlateFontInfo Body = FCoreStyle::GetDefaultFontStyle("Regular", 11);
	const FSlateFontInfo Bold = FCoreStyle::GetDefaultFontStyle("Bold", 11);
	const FSlateFontInfo Small = FCoreStyle::GetDefaultFontStyle("Regular", 9);
	const FSlateFontInfo Heading = FCoreStyle::GetDefaultFontStyle("Bold", 14);

	S->Set("Text.Body", FTextBlockStyle()
		.SetFont(Body).SetColorAndOpacity(FSlateColor(TextPrimary)));
	S->Set("Text.Secondary", FTextBlockStyle()
		.SetFont(Body).SetColorAndOpacity(FSlateColor(TextSecondary)));
	S->Set("Text.Small", FTextBlockStyle()
		.SetFont(Small).SetColorAndOpacity(FSlateColor(TextSecondary)));
	S->Set("Text.Heading", FTextBlockStyle()
		.SetFont(Heading).SetColorAndOpacity(FSlateColor(TextPrimary)));

	// A readout is a number somebody reads off a moving panel, so it is the one
	// role that will most obviously want the tabular font once there is one.
	S->Set("Text.Readout", FTextBlockStyle()
		.SetFont(Bold).SetColorAndOpacity(FSlateColor(TextPrimary)));

	// ===================== SURFACES =====================
	//
	// Flat colour brushes rather than image assets, so the whole language stays
	// text. A gradient or a rounded corner wants a brush asset and can be added
	// per-role later without moving anything else.
	S->Set("Brush.Background", new FSlateColorBrush(Background));
	S->Set("Brush.Panel", new FSlateColorBrush(Panel));
	S->Set("Brush.Border", new FSlateColorBrush(Border));

	// ===================== THE LAMPS =====================
	S->Set("Brush.Lamp.Clear", new FSlateColorBrush(LampClear));
	S->Set("Brush.Lamp.Occupied", new FSlateColorBrush(LampOccupied));
	S->Set("Brush.Lamp.Fault", new FSlateColorBrush(LampFault));
	S->Set("Brush.Lamp.Measured", new FSlateColorBrush(LampMeasured));

	// ===================== METRICS =====================
	//
	// Named rather than typed at each call site, because "8" appearing in forty
	// places is how a layout drifts. NO SIZES HERE, only spacing: the layout
	// decision says no panel hardcodes its own dimensions, and a style set
	// offering a panel width would be the first thing to break it.
	S->Set("Pad.Tight", 4.f);
	S->Set("Pad.Row", 8.f);
	S->Set("Pad.Panel", 12.f);
	S->Set("Pad.Section", 20.f);
	S->Set("Size.Divider", 1.f);

	return S;
}
