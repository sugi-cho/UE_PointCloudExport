// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class PointCloudExport : ModuleRules
{
	public PointCloudExport(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput" });

                PrivateDependencyModuleNames.AddRange(new string[] { "LidarPointCloudRuntime" });

                if (Target.bBuildEditor)
                {
                        PrivateDependencyModuleNames.AddRange(new string[] { "UnrealEd", "AssetRegistry" });
                }


	}
}
