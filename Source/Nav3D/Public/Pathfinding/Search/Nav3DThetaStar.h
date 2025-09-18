#pragma once

#include "CoreMinimal.h"
#include "Pathfinding/Search/Nav3DAStar.h"

class NAV3D_API FNav3DThetaStar : public FNav3DAStar
{
public:
	FNav3DThetaStar();
	virtual ~FNav3DThetaStar() override;

	virtual ENavigationQueryResult::Type FindPath(
		FNav3DPath& OutPath,
		const FNav3DPathingRequest& Request,
		const FNav3DVolumeNavigationData* VolumeNavData) override;

protected:
	bool HasLineOfSight(const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const;
	void ProcessCurrentNodeWithLineOfSight(const FSearchNode& CurrentNode, int32& LineOfSightChecks);
	void ProcessNeighborWithLineOfSight(const FNav3DNodeAddress& NeighborAddress, const FSearchNode& CurrentNode, int32& LineOfSightChecks);

	FNav3DPathingRequest CurrentRequest;
	const ANav3DData* NavDataActor = nullptr;
	class UNav3DRaycaster* Raycaster = nullptr;
};


