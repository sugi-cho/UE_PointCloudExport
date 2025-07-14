// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class PointCloudExport : ModuleRules
{
	public PointCloudExport(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
                PublicDependencyModuleNames.AddRange(new string[]
                {
                        "Core",
                        "CoreUObject",
                        "Engine",
                        "InputCore",
                        "EnhancedInput",
                        "LidarPointCloudRuntime"
                });

                PrivateDependencyModuleNames.AddRange(new string[] { });

                if (Target.bBuildEditor)
                {
                        PrivateDependencyModuleNames.AddRange(new string[] { "UnrealEd", "AssetRegistry" });
                }


	}
}
