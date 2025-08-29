#pragma once
#include "Pathfinding/Nav3DPathFindingProcessor.h"

class NAV3D_API FNav3DPathDebug final : public FNav3DPathFindingProcessor
{
public:
	FNav3DPathDebug(FNav3DPathFinderDebugData& DebugData,
	                const FNav3DPathStepper& Stepper);

	virtual void ProcessNode(
		const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Node) override;

	virtual void ProcessNeighborEvaluation(
		const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Parent,
		const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Neighbor,
		float Cost) override;

	virtual void ProcessNeighborSelection(
		const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Neighbor) override;

	virtual void ProcessFinalPath(
		const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses) override;
	void UpdateProcessedNodeData(const FGraphAStarDefaultNode<FNav3DVolumeNavigationData>& Node) const;

private:
	void UpdateDebugIterationData() const;
	void UpdateCurrentBestPath(
		const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses,
		bool AddEndLocation) const;
	void UpdatePathMetrics() const;
	FNav3DPathFinderDebugData& DebugData;
};
