using UnrealBuildTool;

public class Nav3DEditor : ModuleRules
{
	public Nav3DEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		// Enable strict validation to catch issues like missing Category specifiers
		bTreatAsEngineModule = true; // Treat plugin as Engine module for stricter validation
		
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