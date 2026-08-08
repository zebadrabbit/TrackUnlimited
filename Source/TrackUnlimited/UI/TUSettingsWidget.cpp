#include "TUSettingsWidget.h"

#include "TUStyle.h"
#include "../TUCoasterRide.h"

#include "Blueprint/WidgetTree.h"
#include "Components/VerticalBox.h"
#include "Components/VerticalBoxSlot.h"
#include "Components/HorizontalBox.h"
#include "Components/HorizontalBoxSlot.h"
#include "Components/TextBlock.h"
#include "Components/Button.h"
#include "Components/CheckBox.h"
#include "Components/Slider.h"
#include "Components/ComboBoxString.h"
#include "GameFramework/GameUserSettings.h"

// ===================== APPLYING AN ENGINE SETTING NEEDS A MAP =====================
//
// The schema says each engine-owned entry names its `UGameUserSettings`
// property, and that is true of `bUseVSync` and `FrameRateLimit`. It is NOT true
// of the scalability groups: UE exposes those as SetShadowQuality(int32) and
// friends rather than as uniform properties, so there is nothing to set by name.
//
// So the coupling the table was meant to remove is reduced rather than removed,
// and it is honest to say so here rather than let the schema's comment stand
// unqualified. What the table still buys is real: it decides what appears on
// screen, in what order, with what label, range and default. This function is
// the one place that knows how the engine wants to be told, and it is a switch
// over names rather than logic spread across the UI.
namespace
{
	int32 QualityFromName(const FString& Name)
	{
		if (Name == TEXT("Low"))    { return 0; }
		if (Name == TEXT("Medium")) { return 1; }
		if (Name == TEXT("High"))   { return 2; }
		if (Name == TEXT("Epic"))   { return 3; }
		return 4;   // Cinematic
	}

	FString NameFromQuality(int32 Q)
	{
		static const TCHAR* Names[] = {TEXT("Low"), TEXT("Medium"), TEXT("High"),
									   TEXT("Epic"), TEXT("Cinematic")};
		return Names[FMath::Clamp(Q, 0, 4)];
	}

	EWindowMode::Type WindowModeFromName(const FString& Name)
	{
		if (Name == TEXT("Fullscreen")) { return EWindowMode::Fullscreen; }
		if (Name == TEXT("Windowed"))   { return EWindowMode::Windowed; }
		return EWindowMode::WindowedFullscreen;
	}

	FString NameFromWindowMode(EWindowMode::Type M)
	{
		if (M == EWindowMode::Fullscreen) { return TEXT("Fullscreen"); }
		if (M == EWindowMode::Windowed)   { return TEXT("Windowed"); }
		return TEXT("Windowed Fullscreen");
	}
}

// The per-row handlers UMG's dynamic delegates insist on. Each one forwards to
// the screen with the entry it was made for, which is the whole reason it exists.
void UTUSettingBinding::OnBool(bool bOn)
{
	if (Owner.IsValid()) { Owner->WriteValue(Entry, bOn ? TEXT("true") : TEXT("false")); }
}

void UTUSettingBinding::OnScalar(float Value)
{
	if (Owner.IsValid()) { Owner->WriteValue(Entry, FString::SanitizeFloat(Value)); }
}

void UTUSettingBinding::OnChoice(FString Selected, ESelectInfo::Type)
{
	if (Owner.IsValid()) { Owner->WriteValue(Entry, Selected); }
}

void UTUSettingBinding::OnPageClicked()
{
	if (Owner.IsValid()) { Owner->ShowPage(Page); }
}

void UTUSettingsWidget::AttachTo(ATUCoasterRide* InRide)
{
	Ride = InRide;
}

void UTUSettingsWidget::NativeConstruct()
{
	Super::NativeConstruct();

	if (ResetPageButton)
	{
		ResetPageButton->OnClicked.AddDynamic(this, &UTUSettingsWidget::OnResetPage);
	}
	BuildPageTabs();
	ShowPage(ESettingPage::Video);
}

void UTUSettingsWidget::BuildPageTabs()
{
	if (!PageTabs || !WidgetTree)
	{
		return;
	}
	PageTabs->ClearChildren();

	// THE PAGE LIST IS THE ENUM. Five buttons hardcoded in the asset is a sixth
	// page waiting to be forgotten — the same failure as a hardcoded row list,
	// one level up.
	const ESettingPage Pages[] = {ESettingPage::Video, ESettingPage::Graphics,
								  ESettingPage::Audio, ESettingPage::Controls,
								  ESettingPage::Simulation};
	for (ESettingPage P : Pages)
	{
		UButton* Tab = WidgetTree->ConstructWidget<UButton>();
		UTextBlock* Label = WidgetTree->ConstructWidget<UTextBlock>();
		Label->SetText(FText::FromString(UTF8_TO_TCHAR(SettingPageName(P))));
		Label->SetColorAndOpacity(FSlateColor(FTUStyle::TextPrimary));
		Tab->AddChild(Label);

		if (UHorizontalBoxSlot* S = PageTabs->AddChildToHorizontalBox(Tab))
		{
			S->SetPadding(FMargin(4.f, 6.f));
		}

		UTUSettingBinding* B = NewObject<UTUSettingBinding>(this);
		B->Owner = this;
		B->Page = P;
		TabBindings.Add(B);
		Tab->OnClicked.AddDynamic(B, &UTUSettingBinding::OnPageClicked);
	}
}

void UTUSettingsWidget::ShowPage(ESettingPage Page)
{
	Current = Page;
	if (!RowBox)
	{
		return;
	}

	// REBUILT, NOT HIDDEN. Keeping every page's rows alive and toggling
	// visibility means five pages of widgets ticking for one page of use, and it
	// is the kind of cost that is invisible until the settings screen is the
	// slowest thing in the application.
	RowBox->ClearChildren();
	// The row bindings go with the rows they belonged to. Left behind they would
	// be live handlers on widgets that no longer exist. The tab bindings are a
	// separate array precisely so this line cannot take them too.
	RowBindings.Reset();

	for (const FSettingEntry& E : SettingsSchema())
	{
		if (E.Page == Page)
		{
			BuildRow(E);
		}
	}

	if (PageHelp)
	{
		PageHelp->SetText(FText::FromString(FString::Printf(
			TEXT("%s — changes apply immediately unless a row says otherwise."),
			UTF8_TO_TCHAR(SettingPageName(Page)))));
		PageHelp->SetColorAndOpacity(FSlateColor(FTUStyle::TextSecondary));
	}
}

UTUSettingBinding* UTUSettingsWidget::Bind(const FSettingEntry& Entry)
{
	UTUSettingBinding* B = NewObject<UTUSettingBinding>(this);
	B->Owner = this;
	B->Entry = Entry;
	RowBindings.Add(B);
	return B;
}

void UTUSettingsWidget::BuildRow(const FSettingEntry& Entry)
{
	if (!WidgetTree || !RowBox)
	{
		return;
	}

	UHorizontalBox* Row = WidgetTree->ConstructWidget<UHorizontalBox>();

	UTextBlock* Label = WidgetTree->ConstructWidget<UTextBlock>();
	Label->SetText(FText::FromString(UTF8_TO_TCHAR(Entry.Label.c_str())));
	Label->SetColorAndOpacity(FSlateColor(FTUStyle::TextPrimary));
	if (UHorizontalBoxSlot* S = Row->AddChildToHorizontalBox(Label))
	{
		S->SetSize(ESlateSizeRule::Fill);
		S->SetPadding(FMargin(0.f, 4.f, 12.f, 4.f));
	}

	const FString Value = ReadValue(Entry);

	switch (Entry.Kind)
	{
	case ESettingKind::Bool:
	{
		UCheckBox* Box = WidgetTree->ConstructWidget<UCheckBox>();
		Box->SetIsChecked(Value == TEXT("true"));
		Row->AddChildToHorizontalBox(Box);
		Box->OnCheckStateChanged.AddDynamic(Bind(Entry), &UTUSettingBinding::OnBool);
		break;
	}
	case ESettingKind::Scalar:
	{
		USlider* S = WidgetTree->ConstructWidget<USlider>();
		S->SetMinValue(static_cast<float>(Entry.Min));
		S->SetMaxValue(static_cast<float>(Entry.Max));
		S->SetStepSize(static_cast<float>(Entry.Step));
		S->SetValue(FCString::Atof(*Value));
		Row->AddChildToHorizontalBox(S);
		S->OnValueChanged.AddDynamic(Bind(Entry), &UTUSettingBinding::OnScalar);
		break;
	}
	case ESettingKind::Choice:
	{
		UComboBoxString* C = WidgetTree->ConstructWidget<UComboBoxString>();
		for (const std::string& O : Entry.Options)
		{
			C->AddOption(UTF8_TO_TCHAR(O.c_str()));
		}
		C->SetSelectedOption(Value);
		Row->AddChildToHorizontalBox(C);
		C->OnSelectionChanged.AddDynamic(Bind(Entry), &UTUSettingBinding::OnChoice);
		break;
	}
	case ESettingKind::Key:
	default:
	{
		// SHOWN, NOT YET REBINDABLE. A rebind control has to capture the next
		// keypress, detect conflicts and offer to swap — FInputMap already
		// reports conflicts and this row has nowhere to put that answer yet.
		// Displaying the binding is honest; a button that looks rebindable and
		// is not would be worse than a label.
		UTextBlock* Key = WidgetTree->ConstructWidget<UTextBlock>();
		Key->SetText(FText::FromString(Value));
		Key->SetColorAndOpacity(FSlateColor(FTUStyle::TextSecondary));
		Row->AddChildToHorizontalBox(Key);
		break;
	}
	}

	if (Entry.bNeedsRestart)
	{
		// A RESTART IS A PROMISE THE UI HAS TO KEEP. A control that appears to do
		// nothing is worse than one labelled as deferred.
		UTextBlock* Note = WidgetTree->ConstructWidget<UTextBlock>();
		Note->SetText(FText::FromString(TEXT("restart")));
		Note->SetColorAndOpacity(FSlateColor(FTUStyle::LampOccupied));
		Row->AddChildToHorizontalBox(Note);
	}

	if (UVerticalBoxSlot* S = RowBox->AddChildToVerticalBox(Row))
	{
		S->SetPadding(FMargin(0.f, 2.f));
	}
}

FString UTUSettingsWidget::ReadValue(const FSettingEntry& Entry) const
{
	if (Entry.Owner == ESettingOwner::Engine)
	{
		if (UGameUserSettings* G = UGameUserSettings::GetGameUserSettings())
		{
			if (Entry.Key == "video.windowMode") { return NameFromWindowMode(G->GetFullscreenMode()); }
			if (Entry.Key == "video.vsync")      { return G->IsVSyncEnabled() ? TEXT("true") : TEXT("false"); }
			if (Entry.Key == "video.frameCap")   { return FString::SanitizeFloat(G->GetFrameRateLimit()); }
			if (Entry.Key == "graphics.viewDistance") { return NameFromQuality(G->GetViewDistanceQuality()); }
			if (Entry.Key == "graphics.shadows")      { return NameFromQuality(G->GetShadowQuality()); }
			if (Entry.Key == "graphics.textures")     { return NameFromQuality(G->GetTextureQuality()); }
			if (Entry.Key == "graphics.effects")      { return NameFromQuality(G->GetVisualEffectQuality()); }
			if (Entry.Key == "graphics.antiAliasing") { return NameFromQuality(G->GetAntiAliasingQuality()); }
			if (Entry.Key == "graphics.postProcess")  { return NameFromQuality(G->GetPostProcessingQuality()); }
		}
	}
	// Ours and the input map both fall back to the declared default until those
	// stores are wired to the actor, which keeps every control showing something
	// real rather than blank.
	return UTF8_TO_TCHAR(Entry.Default.c_str());
}

void UTUSettingsWidget::WriteValue(const FSettingEntry& Entry, const FString& Value)
{
	if (Entry.Owner == ESettingOwner::Engine)
	{
		UGameUserSettings* G = UGameUserSettings::GetGameUserSettings();
		if (!G)
		{
			return;
		}
		if (Entry.Key == "video.windowMode") { G->SetFullscreenMode(WindowModeFromName(Value)); }
		else if (Entry.Key == "video.vsync")      { G->SetVSyncEnabled(Value == TEXT("true")); }
		else if (Entry.Key == "video.frameCap")   { G->SetFrameRateLimit(FCString::Atof(*Value)); }
		else if (Entry.Key == "graphics.viewDistance") { G->SetViewDistanceQuality(QualityFromName(Value)); }
		else if (Entry.Key == "graphics.shadows")      { G->SetShadowQuality(QualityFromName(Value)); }
		else if (Entry.Key == "graphics.textures")     { G->SetTextureQuality(QualityFromName(Value)); }
		else if (Entry.Key == "graphics.effects")      { G->SetVisualEffectQuality(QualityFromName(Value)); }
		else if (Entry.Key == "graphics.antiAliasing") { G->SetAntiAliasingQuality(QualityFromName(Value)); }
		else if (Entry.Key == "graphics.postProcess")  { G->SetPostProcessingQuality(QualityFromName(Value)); }

		// APPLIED AND SAVED IMMEDIATELY. A settings screen with an Apply button
		// is a screen where somebody changes four things, closes it, and loses
		// them — and UE persists these itself, so there is nothing for us to own.
		G->ApplySettings(false);
		G->SaveSettings();
		return;
	}

	UE_LOG(LogTUEvents, Log, TEXT("setting %s = %s"),
		UTF8_TO_TCHAR(Entry.Key.c_str()), *Value);
}

void UTUSettingsWidget::OnResetPage()
{
	// PER PAGE, and the button says so. A single "reset everything" is a control
	// somebody presses once, loses their bindings to, and never trusts again.
	for (const FSettingEntry& E : SettingsSchema())
	{
		if (E.Page == Current)
		{
			WriteValue(E, UTF8_TO_TCHAR(E.Default.c_str()));
		}
	}
	ShowPage(Current);
}
