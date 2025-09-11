#pragma once
#include "Pathfinding/Nav3DPathFindingTypes.h"
#include "Pathfinding/Search/Nav3DAStar.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"

enum class NAV3D_API ENav3DPathStepperStatus : uint8
{
	MustContinue,
	IsStopped
};

class NAV3D_API FNav3DPathStepper
{
public:
	explicit FNav3DPathStepper(const FNav3DPathFindingParameters& Parameters);
	virtual ~FNav3DPathStepper() = default;

	ENav3DPathFindingState GetState() const;
	const FNav3DPathFindingParameters& GetParameters() const;
	void AddProcessor(const TSharedPtr<FNav3DPathFindingProcessor>& Processor);
	const FNav3DGraphAStar& GetGraph() const;

	ENav3DPathStepperStatus Step(EGraphAStarResult& Result);
	virtual bool FillNodeAddresses(TArray<FNav3DPathFinderNodeAddress>& NodeAddresses) const;

protected:
	virtual ENav3DPathStepperStatus Init(EGraphAStarResult& Result) = 0;
	virtual ENav3DPathStepperStatus ProcessSingleNode(EGraphAStarResult& Result) = 0;
	virtual ENav3DPathStepperStatus ProcessNeighbour(EGraphAStarResult& Result) = 0;
	virtual ENav3DPathStepperStatus Ended(EGraphAStarResult& Result) = 0;

	void SetState(ENav3DPathFindingState NewState);
	float GetHeuristicCost(const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const;
	float GetTraversalCost(const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const;

	FNav3DGraphAStar Graph;
	ENav3DPathFindingState State;
	FNav3DPathFindingParameters Parameters;
	TArray<TSharedPtr<FNav3DPathFindingProcessor>> Processors;
};

FORCEINLINE ENav3DPathFindingState FNav3DPathStepper::GetState() const
{
	return State;
}

FORCEINLINE const FNav3DPathFindingParameters& FNav3DPathStepper::GetParameters() const
{
	return Parameters;
}

FORCEINLINE const FNav3DGraphAStar& FNav3DPathStepper::GetGraph() const
{
	return Graph;
}
