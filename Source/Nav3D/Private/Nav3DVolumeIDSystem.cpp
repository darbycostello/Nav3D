#include "Nav3DVolumeIDSystem.h"
#include "Nav3D.h"
#include "Engine/World.h"
#include "EngineUtils.h"

ANav3DBoundsVolume* FNav3DVolumeIDSystem::FindVolumeByID(UWorld* World, const uint16 VolumeID)
{
	if (!World || VolumeID == UINT16_MAX)
	{
		return nullptr;
	}

	for (TActorIterator<ANav3DBoundsVolume> It(World); It; ++It)
	{
		ANav3DBoundsVolume* BoundsVolume = *It;
		if (BoundsVolume && IsValid(BoundsVolume))
		{
			if (BoundsVolume->GetVolumeID() == VolumeID)
			{
				return BoundsVolume;
			}
		}
	}

	return nullptr;
}

TMap<uint16, FGuid> FNav3DVolumeIDSystem::GetLoadedVolumeIDs(UWorld* World)
{
	TMap<uint16, FGuid> VolumeMap;

	if (!World)
	{
		return VolumeMap;
	}

	for (TActorIterator<ANav3DBoundsVolume> It(World); It; ++It)
	{
		ANav3DBoundsVolume* BoundsVolume = *It;
		if (BoundsVolume && IsValid(BoundsVolume))
		{
			uint16 VolumeID = BoundsVolume->GetVolumeID();
			if (VolumeID != UINT16_MAX)
			{
				VolumeMap.Add(VolumeID, BoundsVolume->VolumeGUID);
			}
		}
	}

	return VolumeMap;
}

bool FNav3DVolumeIDSystem::ValidateNoCollisions(UWorld* World)
{
	if (!World)
	{
		return true;
	}

	TMap<uint16, ANav3DBoundsVolume*> IDToVolumeMap;
	bool bHasCollisions = false;

	for (TActorIterator<ANav3DBoundsVolume> It(World); It; ++It)
	{
		ANav3DBoundsVolume* BoundsVolume = *It;
		if (BoundsVolume && IsValid(BoundsVolume))
		{
			uint16 VolumeID = BoundsVolume->GetVolumeID();
			if (VolumeID != UINT16_MAX)
			{
				if (ANav3DBoundsVolume** ExistingVolume = IDToVolumeMap.Find(VolumeID))
				{
					UE_LOG(LogNav3D, Error, TEXT("Volume ID collision detected! ID %d used by both '%s' (GUID: %s) and '%s' (GUID: %s)"),
					   VolumeID, 
					   *(*ExistingVolume)->GetName(), *(*ExistingVolume)->VolumeGUID.ToString(),
					   *BoundsVolume->GetName(), *BoundsVolume->VolumeGUID.ToString());
					bHasCollisions = true;
				}
				else
				{
					IDToVolumeMap.Add(VolumeID, BoundsVolume);
				}
			}
		}
	}

	return !bHasCollisions;
}


