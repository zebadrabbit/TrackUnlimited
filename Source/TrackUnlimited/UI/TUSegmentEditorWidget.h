// TrackUnlimited: the runtime segment editor, as UMG.
//
// Second piece of the shell port. Same shape as the menu: built in code from
// the widget tree, no asset, every row a live read. What differs is that this
// panel's state changes on every keystroke (the typed buffer, the focused
// field, the selection), so it is rebuilt whenever a SIGNATURE of that state
// changes rather than on a mode change -- which keeps the immediate-mode rule
// ("nothing cached that can drift") at the cost of one string compare a frame.
//
// Constraint 1 holds exactly as it did on the canvas: numeric entry, click a
// field and type, Enter to apply. Nothing here is dragged.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "TUSegmentEditorWidget.generated.h"

class ATUCoasterRide;
class UVerticalBox;
class UTextBlock;
class UButton;

UCLASS()
class UTUEditorRow : public UObject
{
	GENERATED_BODY()
public:
	TWeakObjectPtr<ATUCoasterRide> Ride;
	int32 Action = 0;   // EEditField as int, or -1000-n for a segment row
	UFUNCTION() void OnClicked();
};

UCLASS()
class TRACKUNLIMITED_API UTUSegmentEditorWidget : public UUserWidget
{
	GENERATED_BODY()
public:
	TWeakObjectPtr<ATUCoasterRide> Ride;

protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;
	virtual void NativeTick(const FGeometry& Geometry, float DeltaSeconds) override;

private:
	UPROPERTY() TObjectPtr<UVerticalBox> Rows;
	UPROPERTY() TArray<TObjectPtr<UTUEditorRow>> Bindings;
	FString LastSignature;

	void Rebuild();
	UButton* AddRow(int32 Action, bool bHighlight);
	UTextBlock* MakeText(const FString& S, const FLinearColor& C, const char* Font = "Font.Body");
	void AddLine(const FString& S, const FLinearColor& C);
};
