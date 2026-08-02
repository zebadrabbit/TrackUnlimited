using UnrealBuildTool;

public class TrackUnlimitedEditorTarget : TargetRules
{
	public TrackUnlimitedEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("TrackUnlimited");
	}
}
