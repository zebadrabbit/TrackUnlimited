// TrackUnlimited: the PAINTED panels -- the console and the profile graph --
// off the debug canvas and onto Slate's OnPaint.
//
// `UI_CONVENTIONS.md` carved these two out as the custom-Slate cases: a block
// strip, a lamp bank and a trace are painted, not composed, and an asset full
// of widgets would be the wrong tool. What this does is the least that meets
// that: the panel code still calls the same four primitives (tile, label, big
// label, line) it always did, but while this widget is painting they RECORD
// into a command list instead of drawing, and OnPaint replays the list. The
// thousand lines of console logic move renderers without moving at all.
//
// Immediate-mode is kept exactly: the list is rebuilt every paint from live
// reads, so nothing is cached that can drift. Hit-testing stays where it was
// (the rect lists the draw fills, consumed by PressConsole/PressGraph on the
// viewport click), so this widget is HIT-TEST INVISIBLE and the draw order of
// the things above it -- confirm, drag answer, recovery -- is untouched because
// those still draw on the canvas, which is above Slate.
//
// ponytail: a replayed command list rather than a composed console. The upgrade,
// if somebody wants a themed lamp, is to compose the lamp bank as widgets and
// leave the strip and the trace painted.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "TUPaintedPanelWidget.generated.h"

class ATUCoasterRide;

struct FTUPanelCmd
{
	enum EKind : uint8 { Tile, Label, BigLabel, Line };
	EKind Kind = Tile;
	FVector2D A = FVector2D::ZeroVector;   // position, or line start
	FVector2D B = FVector2D::ZeroVector;   // size, or line end
	FLinearColor Col = FLinearColor::White;
	FString Text;
	float Thickness = 1.f;
};

UCLASS()
class TRACKUNLIMITED_API UTUPaintedPanelWidget : public UUserWidget
{
	GENERATED_BODY()
public:
	TWeakObjectPtr<ATUCoasterRide> Ride;

protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;
};
