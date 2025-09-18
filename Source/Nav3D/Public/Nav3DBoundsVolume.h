#pragma once

#include <CoreMinimal.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include "Nav3DBoundsVolume.generated.h"

UCLASS(meta=(DisplayName="Nav3D Bounds Volume"))
class NAV3D_API ANav3DBoundsVolume final : public ANavMeshBoundsVolume
{
	GENERATED_BODY()

public:
	/** Persistent unique identifier for this volume (auto-generated if empty) */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Nav3D Volume", meta = (DisplayName = "Volume GUID"))
	FGuid VolumeGUID;

	/** Get a stable 16-bit volume ID derived from the GUID */
	uint16 GetVolumeID() const;

protected:
	virtual void PostLoad() override;
	virtual void OnConstruction(const FTransform& Transform) override;

private:
	/** Ensure the volume has a valid GUID */
	void EnsureValidGUID();
};