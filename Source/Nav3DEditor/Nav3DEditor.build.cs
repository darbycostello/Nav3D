using UnrealBuildTool;

public class Nav3DEditor : ModuleRules
{
	public Nav3DEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PublicDependencyModuleNames.AddRange(new[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"Nav3D",
			"InputCore"
		});

		PrivateDependencyModuleNames.AddRange(new[]
		{
			"Slate",
			"SlateCore",
			"PropertyEditor",
			"EditorStyle",
			"UnrealEd",
			"GraphEditor",
			"BlueprintGraph"
		});

		PrivateIncludePaths.AddRange(new[]
		{
			"Nav3DEditor/Private"
		});

		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	}
}