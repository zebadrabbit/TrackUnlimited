#include "TUSegmentEditorWidget.h"

#include "TUStyle.h"
#include "../TUCoasterRide.h"
#include "Shell/FirstRun.h"
#include "Shell/SegmentEditorModel.h"

#include "Blueprint/WidgetTree.h"
#include "Components/Border.h"
#include "Components/Button.h"
#include "Components/ButtonSlot.h"
#include "Components/CanvasPanel.h"
#include "Components/CanvasPanelSlot.h"
#include "Components/HorizontalBox.h"
#include "Components/HorizontalBoxSlot.h"
#include "Components/SizeBox.h"
#include "Components/TextBlock.h"
#include "Components/VerticalBox.h"
#include "Components/VerticalBoxSlot.h"
#include "Framework/Application/SlateApplication.h"

void UTUEditorRow::OnClicked()
{
	if (ATUCoasterRide* R = Ride.Get())
	{
		// SHIFT EXTENDS, read off the modifier state at the click rather than
		// off the camera-boost flag: the widget already knows which key it was.
		const bool bShift = FSlateApplication::IsInitialized()
			&& FSlateApplication::Get().GetModifierKeys().IsShiftDown();
		R->EditorAction(Action, bShift);
	}
}

TSharedRef<SWidget> UTUSegmentEditorWidget::RebuildWidget()
{
	if (!WidgetTree->RootWidget)
	{
		// LOWER RIGHT, because it is the only corner nothing else wants -- the
		// canvas panel's reason, kept. Anchored rather than positioned so it
		// stays there at any resolution.
		UCanvasPanel* Root = WidgetTree->ConstructWidget<UCanvasPanel>();
		UBorder* Panel = WidgetTree->ConstructWidget<UBorder>();
		Panel->SetBrushColor(FTUStyle::Panel);
		Panel->SetPadding(FMargin(10.f, 8.f));
		Rows = WidgetTree->ConstructWidget<UVerticalBox>();
		USizeBox* Wide = WidgetTree->ConstructWidget<USizeBox>();
		Wide->SetWidthOverride(400.f);
		Wide->AddChild(Rows);
		Panel->AddChild(Wide);
		Root->AddChild(Panel);
		if (UCanvasPanelSlot* S = Cast<UCanvasPanelSlot>(Panel->Slot))
		{
			S->SetAnchors(FAnchors(1.f, 1.f));
			S->SetAlignment(FVector2D(1.f, 1.f));
			S->SetAutoSize(true);
			S->SetPosition(FVector2D(-20.f, -20.f));
		}
		WidgetTree->RootWidget = Root;
	}
	return Super::RebuildWidget();
}

void UTUSegmentEditorWidget::NativeTick(const FGeometry& Geometry, float DeltaSeconds)
{
	Super::NativeTick(Geometry, DeltaSeconds);
	ATUCoasterRide* R = Ride.Get();
	if (!R) { return; }
	// THE SIGNATURE: everything the panel reads that can change between frames.
	// SegmentsRevision stands in for the segment values themselves, because
	// every edit goes through RebuildFromSegments and that bumps it.
	const FString Sig = FString::Printf(TEXT("%d|%d|%d|%d|%s|%d|%d"),
		R->SegmentsRevision, R->SelectedSegment, R->Selection.Num(),
		static_cast<int32>(R->FocusedField), *R->FieldBuffer,
		R->Session.EditsAllowed() ? 1 : 0, R->Segments.Num());
	if (Sig != LastSignature)
	{
		LastSignature = Sig;
		Rebuild();
	}
}

UTextBlock* UTUSegmentEditorWidget::MakeText(const FString& S, const FLinearColor& C, const char* Font)
{
	UTextBlock* T = WidgetTree->ConstructWidget<UTextBlock>();
	T->SetText(FText::FromString(S));
	T->SetColorAndOpacity(FSlateColor(C));
	T->SetFont(FTUStyle::Get().GetFontStyle(Font));
	return T;
}

void UTUSegmentEditorWidget::AddLine(const FString& S, const FLinearColor& C)
{
	UTextBlock* T = MakeText(S, C, "Font.Small");
	T->SetAutoWrapText(true);
	Rows->AddChildToVerticalBox(T);
}

UButton* UTUSegmentEditorWidget::AddRow(int32 Action, bool bHighlight)
{
	UButton* B = WidgetTree->ConstructWidget<UButton>();
	FButtonStyle St = B->GetStyle();
	St.Normal.TintColor = bHighlight ? FTUStyle::Border : FLinearColor::Transparent;
	St.Hovered.TintColor = FTUStyle::Border;
	St.Pressed.TintColor = FTUStyle::Border;
	St.NormalPadding = FMargin(2.f, 1.f);
	St.PressedPadding = FMargin(2.f, 1.f);
	B->SetStyle(St);
	Rows->AddChildToVerticalBox(B);
	UTUEditorRow* R = NewObject<UTUEditorRow>(this);
	R->Ride = Ride;
	R->Action = Action;
	Bindings.Add(R);
	B->OnClicked.AddDynamic(R, &UTUEditorRow::OnClicked);
	return B;
}

void UTUSegmentEditorWidget::Rebuild()
{
	ATUCoasterRide* R = Ride.Get();
	if (!R || !Rows) { return; }
	Rows->ClearChildren();
	Bindings.Reset();

	const TArray<FTUTrackSegment>& Segments = R->Segments;
	const FLinearColor Dim = FTUStyle::TextSecondary, Text = FTUStyle::TextPrimary;
	const FLinearColor Cyan = FTUStyle::LampMeasured, Amber = FTUStyle::LampOccupied;

	// Columns, because the font is proportional and "%-8s" is not a column in it.
	auto Cols = [&](UButton* B, std::initializer_list<TPair<float, FString>> Cells, const FLinearColor& C)
	{
		UHorizontalBox* H = WidgetTree->ConstructWidget<UHorizontalBox>();
		for (const TPair<float, FString>& Cell : Cells)
		{
			USizeBox* Box = WidgetTree->ConstructWidget<USizeBox>();
			if (Cell.Key > 0.f) { Box->SetWidthOverride(Cell.Key); }
			Box->AddChild(MakeText(Cell.Value, C));
			H->AddChildToHorizontalBox(Box);
		}
		B->AddChild(H);
		if (UButtonSlot* Sl = Cast<UButtonSlot>(H->Slot)) { Sl->SetHorizontalAlignment(HAlign_Left); }
	};

	if (Segments.Num() == 0)
	{
		AddLine(TEXT("SEGMENTS   [I] insert  ·  [B] hide"), Dim);
		AddLine(UTF8_TO_TCHAR(EmptyStateFor(EPanelKind::SegmentList)), Dim);
		return;
	}

	const bool bEditsAllowed = R->Session.EditsAllowed();
	AddLine(bEditsAllowed
		? TEXT("SEGMENTS   [B] hide   click a field, type, Enter")
		: TEXT("SEGMENTS   read-only while the ride runs   [Tab] to BUILD"),
		bEditsAllowed ? Dim : Amber);

	// Windowed around the selection: a CSV import of four thousand is not a panel.
	const int32 Window = 8;
	const int32 First = FMath::Max(0, FMath::Min(R->SelectedSegment - Window / 2, Segments.Num() - Window));
	const int32 Last = FMath::Min(Segments.Num(), First + Window);
	double SAt = 0.0;
	for (int32 i = 0; i < First; ++i) { SAt += Segments[i].Length; }
	for (int32 i = First; i < Last; ++i)
	{
		const bool bSel = i == R->SelectedSegment;
		const bool bDevice = Segments[i].Zone != ETUSegmentZone::None;
		UButton* B = AddRow(-1000 - i, bSel);
		Cols(B, {
			{28.f, FString::Printf(TEXT("%d"), i)},
			{72.f, FString(ATUCoasterRide::KindNameOf(Segments[i].Kind))},
			{64.f, FString::Printf(TEXT("%.1f m"), Segments[i].Length)},
			{56.f, FString::Printf(TEXT("@%.0f"), SAt)},
			{0.f, bDevice ? FString(ATUCoasterRide::ZoneNameOf(Segments[i].Zone)) : FString()}},
			bSel ? Cyan : (bDevice ? Amber : Text));
		SAt += Segments[i].Length;
	}
	if (Segments.Num() > Window)
	{
		AddLine(FString::Printf(TEXT("   %d of %d"), Last - First, Segments.Num()), Dim);
	}

	if (!Segments.IsValidIndex(R->SelectedSegment)) { return; }
	const FTUTrackSegment& Seg = Segments[R->SelectedSegment];

	// The fields, per kind -- EditConditionHides from the tested model.
	const EEditKind Kind = ATUCoasterRide::KindOf(Seg.Kind);
	const bool bMulti = R->IsMultiSelect();
	const std::vector<FFieldView> MultiFields =
		bMulti ? R->BuildSelectionEditor().Fields() : std::vector<FFieldView>();
	bool bFirstField = true;
	for (std::size_t f = 0; f < static_cast<std::size_t>(EEditField::Count); ++f)
	{
		const EEditField F = static_cast<EEditField>(f);
		if (!KindUsesField(Kind, F)) { continue; }
		const bool bDeviceField = F == EEditField::ZoneSpeed
			|| F == EEditField::ZoneAccel || F == EEditField::ZoneDecel
			|| F == EEditField::ZoneBrakeDecel || F == EEditField::StartsNewDevice;
		if (bDeviceField && Seg.Zone == ETUSegmentZone::None) { continue; }
		if (bMulti && (f >= MultiFields.size() || !MultiFields[f].bVisible)) { continue; }

		const bool bFocus = R->FocusedField == F;
		const bool bChoice = IsChoiceField(F);
		FString Value;
		if (F == EEditField::Kind) { Value = ATUCoasterRide::KindNameOf(Seg.Kind); }
		else if (F == EEditField::ZoneKind)
		{
			Value = Seg.Zone == ETUSegmentZone::None ? FString(TEXT("none")) : FString(ATUCoasterRide::ZoneNameOf(Seg.Zone));
		}
		else if (F == EEditField::StartsNewDevice) { Value = Seg.bStartsNewDevice ? TEXT("yes") : TEXT("no"); }
		else if (F == EEditField::PitchEase) { Value = ATUCoasterRide::PitchEaseNameOf(Seg.PitchEase); }
		else
		{
			const bool bDiffers = bMulti && f < MultiFields.size() && MultiFields[f].bDiffers;
			Value = bFocus ? R->FieldBuffer + TEXT("_")
				: (bDiffers ? FString(TEXT("--  (differs)"))
							: FString::Printf(TEXT("%.4g"), R->ReadField(Seg, F)));
		}

		UButton* B = AddRow(static_cast<int32>(f), bFocus);
		if (UVerticalBoxSlot* Sl = Cast<UVerticalBoxSlot>(B->Slot))
		{
			if (bFirstField) { Sl->SetPadding(FMargin(0.f, 8.f, 0.f, 0.f)); bFirstField = false; }
		}
		UHorizontalBox* H = WidgetTree->ConstructWidget<UHorizontalBox>();
		USizeBox* NameBox = WidgetTree->ConstructWidget<USizeBox>();
		NameBox->SetWidthOverride(146.f);
		NameBox->AddChild(MakeText(UTF8_TO_TCHAR(FieldName(F)), Dim));
		H->AddChildToHorizontalBox(NameBox);
		H->AddChildToHorizontalBox(MakeText(
			bChoice ? FString::Printf(TEXT("%s  <click"), *Value)
					: FString::Printf(TEXT("%s %s"), *Value, UTF8_TO_TCHAR(FieldUnit(F))),
			bFocus ? Cyan : (bChoice ? Amber : Text)));
		B->AddChild(H);
		if (UButtonSlot* Sl = Cast<UButtonSlot>(H->Slot)) { Sl->SetHorizontalAlignment(HAlign_Left); }
	}

	// The tooltip for whatever is focused.
	if (R->FocusedField != EEditField::Count)
	{
		const FFieldHelp Help = HelpFor(R->FocusedField);
		AddLine(UTF8_TO_TCHAR(Help.Tooltip), Dim);
		if (Help.bHasRange)
		{
			AddLine(FString::Printf(TEXT("typically %.4g to %.4g %s   ·  Enter to apply, Esc to cancel"),
				Help.TypicalMin, Help.TypicalMax, UTF8_TO_TCHAR(FieldUnit(R->FocusedField))), Dim);
		}
	}
}
