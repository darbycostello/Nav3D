#pragma once

#include "CoreMinimal.h"
#include "Nav3DBoundsVolume.h"

class NAV3D_API FNav3DVolumeIDSystem
{
public:
	/** Find a Nav3DBoundsVolume by its GUID-based ID in the current world */
	static ANav3DBoundsVolume* FindVolumeByID(UWorld* World, uint16 VolumeID);

	/** Get all currently loaded volume IDs and their GUIDs for debugging */
	static TMap<uint16, FGuid> GetLoadedVolumeIDs(UWorld* World);

	/** Check for GUID hash collisions in currently loaded volumes */
	static bool ValidateNoCollisions(UWorld* World);
};


