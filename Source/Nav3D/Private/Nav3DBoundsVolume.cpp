#include "Nav3DBoundsVolume.h"
#include "Nav3D.h"
void ANav3DBoundsVolume::PostLoad()
{
	Super::PostLoad();
	EnsureValidGUID();
}

void ANav3DBoundsVolume::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	EnsureValidGUID();
}

void ANav3DBoundsVolume::EnsureValidGUID()
{
	if (!VolumeGUID.IsValid())
	{
		VolumeGUID = FGuid::NewGuid();
		UE_LOG(LogNav3D, Log, TEXT("Generated new GUID for Nav3DBoundsVolume '%s': %s"), 
		       *GetName(), *VolumeGUID.ToString());
	}
}

uint16 ANav3DBoundsVolume::GetVolumeID() const
{
	if (!VolumeGUID.IsValid())
	{
		UE_LOG(LogNav3D, Warning, TEXT("Nav3DBoundsVolume '%s' has invalid GUID"), *GetName());
		return UINT16_MAX;
	}

	// Hash the GUID to create a stable 16-bit ID
	const uint32 Hash = GetTypeHash(VolumeGUID);

	// Reduce to 16-bit while avoiding UINT16_MAX (reserved for invalid)
	uint16 VolumeID = static_cast<uint16>(Hash & 0xFFFE);
	if (VolumeID == UINT16_MAX)
	{
		VolumeID = 0;
	}

	return VolumeID;
}
