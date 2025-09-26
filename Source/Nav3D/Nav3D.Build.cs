// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class Nav3D : ModuleRules
{
	public Nav3D(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bUseUnity = true;


		PublicIncludePaths.AddRange(
			new string[]
			{
				ModuleDirectory + "/../ThirdParty"
			}
		);


		PrivateIncludePaths.AddRange(
			new[]
			{
				"Nav3D/Private"
				// ... add other private include paths required here ...
			}
		);


		PublicDependencyModuleNames.AddRange(
			new[]
			{
				"Core", "AIModule"
				// ... add other public dependencies that you statically link with here ...
			}
		);


		PrivateDependencyModuleNames.AddRange(
			new[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"RHI",
				"RenderCore",
				"DeveloperSettings",
				"GameplayTasks",
				"AIModule",
				"NavigationSystem",
				"Landscape",
				"InputCore",
			}
		);


		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
		);

		if (Target.bBuildEditor)
			PublicDependencyModuleNames.Add("UnrealEd");
	}
}