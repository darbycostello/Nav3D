#pragma once
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Stepper/Nav3DPathStepper.h"

class NAV3D_API FNav3DPathStepperAStar : public FNav3DPathStepper
{
public:
	explicit FNav3DPathStepperAStar(const FNav3DPathFindingParameters& Parameters);
	virtual bool FillNodeAddresses(TArray<FNav3DPathFinderNodeAddress>&) const override;

protected:
	virtual ENav3DPathStepperStatus Init(EGraphAStarResult& Result) override;
	virtual ENav3DPathStepperStatus ProcessSingleNode(EGraphAStarResult& Result) override;
	virtual ENav3DPathStepperStatus ProcessNeighbour(EGraphAStarResult& Result) override;
	virtual ENav3DPathStepperStatus Ended(EGraphAStarResult& Result) override;

	void FillNodeAddressNeighbours(const FNav3DNodeAddress& NodeAddress);
	float AdjustTotalCostWithNodeSizeCompensation(
		float TotalCost,
		FNav3DNodeAddress NeighbourNodeAddress) const;

	struct FNeighbourIndexIncrement
	{
		FNeighbourIndexIncrement(TArray<FNav3DNodeAddress>& Neighbours,
		                         int& NeighbourIndex,
		                         ENav3DPathFindingState& State);
		~FNeighbourIndexIncrement();

		TArray<FNav3DNodeAddress>& Neighbours;
		int& NeighbourIndex;
		ENav3DPathFindingState& State;
	};

	int32 ConsideredNodeIndex;
	int32 BestNodeIndex;
	float BestNodeCost;
	int NeighbourIndex;
	TArray<FNav3DNodeAddress> Neighbours;
};
