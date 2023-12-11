// Copyright Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;

public class Voxel : ModuleRules
{
	public Voxel(ReadOnlyTargetRules Target) : base(Target)
    {
        PublicIncludePaths.Add(ModuleDirectory);
        PrivateIncludePaths.Add(ModuleDirectory);
		
		PublicDependencyModuleNames.AddRange(
			new string[]
            {
                "Core",
                "Engine",
                "RealtimeMeshComponent",
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
			}
			);
	}
}
