#pragma once

#include "Nav3DVolumeNavigationData.h"
#include <AI/Navigation/NavigationDataChunk.h>
#include <CoreMinimal.h>
#include "Nav3DDataChunk.generated.h"

UCLASS()
class NAV3D_API UNav3DDataChunk final : public UNavigationDataChunk
{
	GENERATED_BODY()

public:
	virtual void Serialize(FArchive& Archive) override;
	void AddNavigationData(FNav3DVolumeNavigationData& NavData);
	void ReleaseNavigationData();

	TArray<FNav3DVolumeNavigationData> NavigationData;
};
