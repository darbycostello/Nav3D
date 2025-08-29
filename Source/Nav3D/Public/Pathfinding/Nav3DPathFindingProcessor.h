#pragma once

#include "Nav3DPathFindingTypes.h"
#include "Nav3DVolumeNavigationData.h"
#include <GraphAStar.h>

class FNav3DPath;
class FNav3DPathStepper;
struct FNav3DPathFinderDebugData;

class NAV3D_API FNav3DPathFindingProcessor
{
public:
	explicit FNav3DPathFindingProcessor(const FNav3DPathStepper& Stepper);
	virtual ~FNav3DPathFindingProcessor() = default;

	// Process a single node during pathfinding
	virtual void ProcessNode(const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Node)
	{
	}

	// Process neighbor evaluation with parent context
	virtual void ProcessNeighborEvaluation(
		const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Parent,
		const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Neighbor,
		float Cost)
	{
	}

	// Process neighbor after selection
	virtual void ProcessNeighborSelection(
		const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Neighbor)
	{
	}

	// Process final path when search completes successfully
	virtual void ProcessFinalPath(
		const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses)
	{
	}

protected:
	const FNav3DPathStepper& Stepper;
};
