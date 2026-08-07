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
			"ProceduralMeshComponent" });
	}
}
