#pragma once

#include <CoreMinimal.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include "Nav3DBoundsVolume.generated.h"

UCLASS(meta=(DisplayName="Nav3D Bounds Volume"))
class NAV3D_API ANav3DBoundsVolume final : public ANavMeshBoundsVolume
{
	GENERATED_BODY()
};