using UnrealBuildTool;

public class TrackUnlimitedTarget : TargetRules
{
	public TrackUnlimitedTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("TrackUnlimited");
	}
}
