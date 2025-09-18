#pragma once

#include "CoreMinimal.h"
#include "Nav3DVolumeNavigationData.h"
#include "Pathfinding/Core/Nav3DPath.h"
#include "Pathfinding/Core/Nav3DPathingTypes.h"
#include "Pathfinding/Search/Nav3DThetaStar.h"

class NAV3D_API FNav3DLazyThetaStar : public FNav3DThetaStar
{
public:
	FNav3DLazyThetaStar();
	virtual ~FNav3DLazyThetaStar() override;

	virtual ENavigationQueryResult::Type FindPath(
		FNav3DPath& OutPath,
		const FNav3DPathingRequest& Request,
		const FNav3DVolumeNavigationData* VolumeNavData) override;

protected:
	void ProcessCurrentNodeWithLazyLOS(FSearchNode& CurrentNode, int32& LineOfSightChecks);
	void UpdateVertexLazy(FSearchNode& CurrentNode);
	void ProcessCurrentNodeForNeighbors(const FSearchNode& CurrentNode);

	FNav3DPathingRequest CurrentRequest;
};


