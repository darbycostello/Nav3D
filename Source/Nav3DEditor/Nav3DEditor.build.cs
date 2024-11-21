using UnrealBuildTool;

public class Nav3DEditor : ModuleRules
{
	public Nav3DEditor(ReadOnlyTargetRules Target) : base(Target) {

	    PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine",  "Nav3D", "InputCore"});
	    PrivateDependencyModuleNames.AddRange(new string[] { "Slate", 
		    "SlateCore", "PropertyEditor", "EditorStyle", "UnrealEd", "GraphEditor", "BlueprintGraph","DetailCustomizations" });
	    PrivateIncludePaths.AddRange(new string[] { "Nav3DEditor/Private"} );
	    PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

    }
};