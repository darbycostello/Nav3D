#include "Pathfinding/Search/Nav3DAStar.h"
#include "Nav3D.h"
#include "Nav3DVolumeNavigationData.h"
#include "Nav3DUtils.h"
#include "Pathfinding/Search/Nav3DPathHeuristicCalculator.h"
#include "Pathfinding/Search/Nav3DPathTraversalCostCalculator.h"

FNav3DAStar::FNav3DAStar()
{
}

FNav3DAStar::~FNav3DAStar()
{
}

ENavigationQueryResult::Type FNav3DAStar::FindPath(
	FNav3DPath& OutPath,
	const FNav3DPathingRequest& Request,
	const FNav3DVolumeNavigationData* VolumeNavData)
{
	LogPathfindingStart(Request, TEXT("A*"));
	
	if (!VolumeNavData)
	{
		UE_LOG(LogNav3D, Warning, TEXT("A*: No volume navigation data provided"));
		return ENavigationQueryResult::Error;
	}

	// Initialize search state
	InitializeSearch(Request, VolumeNavData);
	
	// Calculate minimum layer index based on agent radius
	const float AgentRadius = Request.AgentProperties.AgentRadius;
	const LayerIndex MinLayerIndex = VolumeNavData->GetMinLayerIndexForAgentSize(AgentRadius);

	
	
    // Resolve endpoints to nearest navigable nodes using volume data
    if (!VolumeNavData->GetNodeAddressFromPosition(StartAddress, Request.StartLocation, MinLayerIndex))
    {
        UE_LOG(LogNav3D, Warning, TEXT("A*: Could not resolve start location %s to a navigable node"), *Request.StartLocation.ToString());
        return ENavigationQueryResult::Error;
    }
    if (!VolumeNavData->GetNodeAddressFromPosition(GoalAddress, Request.EndLocation, MinLayerIndex))
    {
        UE_LOG(LogNav3D, Warning, TEXT("A*: Could not resolve goal location %s to a navigable node"), *Request.EndLocation.ToString());
        return ENavigationQueryResult::Error;
    }
    const FVector ProjectedStartLocation = VolumeNavData->GetNodePositionFromAddress(StartAddress, false);
    const FVector ProjectedGoalLocation = VolumeNavData->GetNodePositionFromAddress(GoalAddress, false);

	UE_LOG(LogTemp, Warning, TEXT("Start Node Address: %s"), *StartAddress.ToString());
	UE_LOG(LogTemp, Warning, TEXT("Goal Node Address: %s"), *GoalAddress.ToString());
	UE_LOG(LogTemp, Warning, TEXT("Projected Start Location: %s"), *ProjectedStartLocation.ToString());
	UE_LOG(LogTemp, Warning, TEXT("Projected Goal Location: %s"), *ProjectedGoalLocation.ToString());

	if (StartAddress == GoalAddress)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("A*: Start and goal project to same node - creating direct path"));
		OutPath.ResetForRepath();
		TArray<FNavPathPoint>& Points = OutPath.GetPathPoints();
		Points.Add(FNavPathPoint(Request.StartLocation));
		Points.Add(FNavPathPoint(ProjectedStartLocation));
		if (!ProjectedStartLocation.Equals(ProjectedGoalLocation, 1.0f))
		{
			Points.Add(FNavPathPoint(ProjectedGoalLocation));
		}
		Points.Add(FNavPathPoint(Request.EndLocation));
		OutPath.MarkReady();
		return ENavigationQueryResult::Success;
	}

	// Initialize start node
	FSearchNode& StartNode = AllNodes.Add(StartAddress);
	StartNode.Address = StartAddress;
	StartNode.GScore = 0.0f;
	StartNode.FScore = CalculateHeuristic(StartAddress, GoalAddress);
	StartNode.bInOpenSet = true;
	OpenSet.Add(StartAddress);

	int32 Iteration = 0;
	constexpr int32 MaxIterations = 10000; // Safety limit

	// Main A* loop
	while (OpenSet.Num() > 0 && Iteration < MaxIterations)
	{
		Iteration++;

		// Find node with lowest F score
		int32 BestIndex = 0;
		float BestFScore = AllNodes[OpenSet[0]].FScore;
		for (int32 i = 1; i < OpenSet.Num(); ++i)
		{
			const float FScore = AllNodes[OpenSet[i]].FScore;
			if (FScore < BestFScore)
			{
				BestFScore = FScore;
				BestIndex = i;
			}
		}

		// Get current node and remove from open set
		FNav3DNodeAddress CurrentAddress = OpenSet[BestIndex];
		OpenSet.RemoveAtSwap(BestIndex);
		
		FSearchNode& CurrentNode = AllNodes[CurrentAddress];
		CurrentNode.bInOpenSet = false;
		CurrentNode.bInClosedSet = true;

		// Log progress periodically
		if (LogVerbosity >= ENav3DPathingLogVerbosity::Detailed && (Iteration % 100 == 0))
		{
			LogSearchProgress(Iteration, OpenSet.Num(), BestFScore);
		}

		// Check if we reached the goal
		if (CurrentAddress == GoalAddress)
		{
			const ENavigationQueryResult::Type Result = ReconstructPath(OutPath, CurrentNode);
			LogPathfindingResult(Result, OutPath.GetPathPoints().Num(), TEXT("A*"));
			return Result;
		}

		// Process neighbors
		ProcessCurrentNode(CurrentNode);
	}

	if (Iteration >= MaxIterations)
	{
		UE_LOG(LogNav3D, Warning, TEXT("A*: Reached maximum iteration limit (%d)"), MaxIterations);
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("A*: No path found - open set exhausted"));
	}

	return ENavigationQueryResult::Fail;
}

void FNav3DAStar::InitializeSearch(const FNav3DPathingRequest& Request, const FNav3DVolumeNavigationData* VolumeNavData)
{
	AllNodes.Empty();
	OpenSet.Empty();
	VolumeData = VolumeNavData;
	LogVerbosity = Request.LogVerbosity;
	CurrentRequest = Request;
}

void FNav3DAStar::ProcessCurrentNode(const FSearchNode& CurrentNode)
{
	// Get neighbors for current node using the actual API
	TArray<FNav3DNodeAddress> Neighbors;
	VolumeData->GetNodeNeighbours(Neighbors, CurrentNode.Address);

	for (const FNav3DNodeAddress& NeighborAddress : Neighbors)
	{
		ProcessNeighbor(NeighborAddress, CurrentNode);
	}
}

void FNav3DAStar::ProcessNeighbor(const FNav3DNodeAddress& NeighborAddress, const FSearchNode& CurrentNode)
{
	// Skip if neighbor is in closed set
	if (const FSearchNode* ExistingNeighbor = AllNodes.Find(NeighborAddress))
	{
		if (ExistingNeighbor->bInClosedSet)
			return;
	}

	// Calculate tentative G score using cost calculator
	const float TentativeGScore = CurrentNode.GScore + CalculateDistance(CurrentNode.Address, NeighborAddress);

	// Get or create neighbor node
	FSearchNode& NeighborNode = AllNodes.FindOrAdd(NeighborAddress);
	if (NeighborNode.Address.IsValid() == false)
	{
		// New node
		NeighborNode.Address = NeighborAddress;
		NeighborNode.GScore = TNumericLimits<float>::Max();
	}

    // Check if this path to neighbor is better (avoid self-parent cycles)
    if (TentativeGScore < NeighborNode.GScore && NeighborAddress != CurrentNode.Address)
	{
		NeighborNode.Parent = CurrentNode.Address;
		NeighborNode.GScore = TentativeGScore;
		const float HeuristicCost = CalculateHeuristic(NeighborAddress, GoalAddress);
		const float TotalCost = TentativeGScore + HeuristicCost;
		NeighborNode.FScore = AdjustTotalCostWithNodeSizeCompensation(TotalCost, NeighborAddress);

		// Add to open set if not already there
		if (!NeighborNode.bInOpenSet)
		{
			NeighborNode.bInOpenSet = true;
			OpenSet.Add(NeighborAddress);
		}
	}
}

ENavigationQueryResult::Type FNav3DAStar::ReconstructPath(FNav3DPath& OutPath, const FSearchNode& GoalNode)
{
	TArray<FNav3DNodeAddress> PathAddresses;
    
	// Trace back from goal to start
	FNav3DNodeAddress Current = GoalNode.Address;
	int32 ChainIndex = 0;
	while (Current.IsValid() && PathAddresses.Num() < 1000)
	{
		PathAddresses.Add(Current);
	
		if (const FSearchNode* Node = AllNodes.Find(Current))
		{
			Current = Node->Parent;
			if (Current == StartAddress)
			{
				PathAddresses.Add(StartAddress);
				break;
			}
		}
		else
		{
			UE_LOG(LogNav3D, Error, TEXT("Chain[%d]: Node not found in AllNodes! Breaking chain."), ChainIndex);
			break;
		}
		ChainIndex++;
	}
    

	// Reverse to get start-to-goal order
	Algo::Reverse(PathAddresses);

	// Include original endpoints
	TArray<FNavPathPoint>& PathPoints = OutPath.GetPathPoints();
	TArray<float>& PathCosts = OutPath.GetPathPointCosts();
	PathPoints.Empty();
	PathCosts.Empty();

	PathPoints.Add(FNavPathPoint(CurrentRequest.StartLocation));
	PathCosts.Add(0.0f);

	// Safety check: ensure we have at least one path address
	if (PathAddresses.Num() == 0)
	{
		UE_LOG(LogNav3D, Error, TEXT("ReconstructPath: No path addresses found, returning empty path"));
		return ENavigationQueryResult::Type::Invalid;
	}

    const FVector ProjectedStartPos = VolumeData->GetNodePositionFromAddress(PathAddresses[0], true);
	if (!CurrentRequest.StartLocation.Equals(ProjectedStartPos, 1.0f))
	{
		PathPoints.Add(FNavPathPoint(ProjectedStartPos));
		PathCosts.Add(FVector::Dist(CurrentRequest.StartLocation, ProjectedStartPos));
	}

	for (int32 i = 1; i < PathAddresses.Num(); ++i)
	{
        const FVector WorldPos = VolumeData->GetNodePositionFromAddress(PathAddresses[i], true);
		PathPoints.Add(FNavPathPoint(WorldPos));
		const float Cost = FVector::Dist(WorldPos, PathPoints[PathPoints.Num() - 2].Location);
		PathCosts.Add(Cost);
	}

    const FVector ProjectedGoalPos = VolumeData->GetNodePositionFromAddress(PathAddresses.Last(), true);
	if (!CurrentRequest.EndLocation.Equals(ProjectedGoalPos, 1.0f))
	{
		PathPoints.Add(FNavPathPoint(ProjectedGoalPos));
		PathCosts.Add(FVector::Dist(PathPoints.Last().Location, ProjectedGoalPos));
	}

	if (!CurrentRequest.EndLocation.Equals(PathPoints.Last().Location, 1.0f))
	{
		PathPoints.Add(FNavPathPoint(CurrentRequest.EndLocation));
		PathCosts.Add(FVector::Dist(PathPoints[PathPoints.Num() - 2].Location, CurrentRequest.EndLocation));
	}

	OutPath.MarkReady();
	return ENavigationQueryResult::Success;
}

float FNav3DAStar::CalculateHeuristic(const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const
{
	if (CurrentRequest.HeuristicCalculator)
	{
		const float HeuristicCost = CurrentRequest.HeuristicCalculator->GetHeuristicCost(*VolumeData, From, To);
		return HeuristicCost * CurrentRequest.HeuristicScale;
	}
	const FVector FromPos = VolumeData->GetNodePositionFromAddress(From, false);
	const FVector ToPos = VolumeData->GetNodePositionFromAddress(To, false);
	return FVector::Dist(FromPos, ToPos) * CurrentRequest.HeuristicScale;
}

float FNav3DAStar::CalculateDistance(const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const
{
	if (CurrentRequest.CostCalculator)
	{
		return CurrentRequest.CostCalculator->GetTraversalCost(*VolumeData, From, To);
	}
	const FVector FromPos = VolumeData->GetNodePositionFromAddress(From, false);
	const FVector ToPos = VolumeData->GetNodePositionFromAddress(To, false);
	return FVector::Dist(FromPos, ToPos);
}

void FNav3DAStar::LogSearchProgress(int32 Iteration, int32 OpenSetSize, float BestFScore)
{
	UE_LOG(LogNav3D, Verbose, TEXT("A*[%d]: OpenSet=%d, BestF=%.2f"), 
		   Iteration, OpenSetSize, BestFScore);
}

float FNav3DAStar::AdjustTotalCostWithNodeSizeCompensation(float TotalCost, const FNav3DNodeAddress& NodeAddress) const
{
	if (!CurrentRequest.bUseNodeSizeCompensation)
	{
		return TotalCost;
	}

	const LayerIndex Layer = NodeAddress.LayerIndex;
	const FNav3DData& NavData = VolumeData->GetData();
	
	// Bounds check to prevent array index out of bounds crash
	if (Layer < 0 || Layer >= NavData.GetLayerCount())
	{
		UE_LOG(LogNav3D, Warning, TEXT("AdjustTotalCostWithNodeSizeCompensation: Invalid layer index %d (valid range: 0-%d), falling back to base layer"), 
			   Layer, NavData.GetLayerCount() - 1);
		return TotalCost; // No compensation for invalid layers
	}
	
	const float NodeSize = NavData.GetLayer(Layer).GetNodeSize();
	const float BaseSize = NavData.GetLayer(0).GetNodeSize();
	const float LayerMultiplier = NodeSize / FMath::Max(BaseSize, KINDA_SMALL_NUMBER);
	const float CompensationFactor = 1.0f / FMath::Max(1.0f, LayerMultiplier);
	return TotalCost * CompensationFactor;
}


