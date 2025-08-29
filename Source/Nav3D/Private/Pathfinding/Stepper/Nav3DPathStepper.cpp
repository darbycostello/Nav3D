#include "Pathfinding/Stepper/Nav3DPathStepper.h"
#include "Pathfinding/Search/Nav3DPathHeuristicCalculator.h"
#include "Pathfinding/Search/Nav3DPathTraversalCostCalculator.h"

FNav3DPathStepper::FNav3DPathStepper(
	const FNav3DPathFindingParameters& Parameters)
	: Graph(Parameters.VolumeNavigationData),
	  State(ENav3DPathFindingState::Init), Parameters(Parameters)
{
}

void FNav3DPathStepper::AddProcessor(
	const TSharedPtr<FNav3DPathFindingProcessor>& Processor)
{
	Processors.Add(Processor);
}

ENav3DPathStepperStatus
FNav3DPathStepper::Step(EGraphAStarResult& Result)
{
	switch (State)
	{
	case ENav3DPathFindingState::Init:
		{
			return Init(Result);
		}
	case ENav3DPathFindingState::ProcessNode:
		{
			return ProcessSingleNode(Result);
		}
	case ENav3DPathFindingState::ProcessNeighbour:
		{
			return ProcessNeighbour(Result);
		}
	case ENav3DPathFindingState::Ended:
		{
			return Ended(Result);
		}
	default:
		{
			checkNoEntry();
			return ENav3DPathStepperStatus::IsStopped;
		}
	}
}

bool FNav3DPathStepper::FillNodeAddresses(
	TArray<FNav3DPathFinderNodeAddress>& NodeAddresses) const
{
	checkNoEntry();
	return false;
}

void FNav3DPathStepper::SetState(
	const ENav3DPathFindingState NewState)
{
	State = NewState;
}

float FNav3DPathStepper::GetHeuristicCost(
	const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const
{
	return Parameters.HeuristicCalculator->GetHeuristicCost(
			Parameters.VolumeNavigationData, From, To) *
		Parameters.NavigationQueryFilter.GetHeuristicScale();
}

float FNav3DPathStepper::GetTraversalCost(
	const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const
{
	return Parameters.CostCalculator->GetTraversalCost(
		Parameters.VolumeNavigationData, From, To);
}
