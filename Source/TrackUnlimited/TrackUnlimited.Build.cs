using UnrealBuildTool;

public class TrackUnlimited : ModuleRules
{
	public TrackUnlimited(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		// The Phase 0 prototypes are engine-free header-only C++ and are the
		// canonical implementations, not a reference to copy from. Including
		// them directly keeps ONE source of truth: the standalone assert suites
		// under Prototypes/ test exactly the code that ships in the game.
		PublicIncludePaths.Add(System.IO.Path.Combine(ModuleDirectory, "..", "..", "Prototypes"));

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput",
			// Phase 4: the swept track mesh. A built-in engine plugin, enabled in
			// the .uproject — nothing is downloaded and removing both lines reverts it.
			"ProceduralMeshComponent",
			// Phase 5: the support placer asks the level's Landscape where the
			// ground is, rather than assuming a flat datum.
			"Landscape",
			// SetNearClipPlaneGlobals. UCameraComponent has no perspective near-plane
			// setter — the near plane is one global, and this is the engine's own
			// supported way to move it (r.SetNearClipPlane calls exactly this).
			"RenderCore",
			// Phase 3.5's UI. UMG is the authoring layer per UI_CONVENTIONS.md and
			// Slate is what it resolves to, so both are needed: the design language
			// lives in an FSlateStyleSet, which UMG widgets consume through
			// FButtonStyle/FTextBlockStyle exactly as Slate widgets do.
			"Slate", "SlateCore", "UMG" });

		// THE OS FILE DIALOG, WHICH IS EDITOR-ONLY AND SAYS SO HERE TOO.
		// IDesktopPlatform does not ship in a packaged game; the menu's open row
		// reveals the tracks folder there instead, and the browser list is the
		// primary path in both because it works everywhere.
		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.Add("DesktopPlatform");
		}
	}
}
