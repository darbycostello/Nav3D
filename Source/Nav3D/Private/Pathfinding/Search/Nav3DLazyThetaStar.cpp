#include "Pathfinding/Search/Nav3DLazyThetaStar.h"
#include "Nav3D.h"
#include "Nav3DUtils.h"

FNav3DLazyThetaStar::FNav3DLazyThetaStar()
{
}

FNav3DLazyThetaStar::~FNav3DLazyThetaStar()
{
}

ENavigationQueryResult::Type FNav3DLazyThetaStar::FindPath(
	FNav3DPath& OutPath,
	const FNav3DPathingRequest& Request,
	const FNav3DVolumeNavigationData* VolumeNavData)
{
	LogPathfindingStart(Request, TEXT("Lazy Theta*"));
	
	if (!VolumeNavData)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Lazy Theta*: No volume navigation data provided"));
		return ENavigationQueryResult::Error;
	}

	// Store current request for use in HasLineOfSight
	CurrentRequest = Request;
	NavDataActor = Request.NavData;

	// Initialize search state
	InitializeSearch(Request, VolumeNavData);
	
	// Calculate minimum layer index based on agent radius
	const float AgentRadius = Request.AgentProperties.AgentRadius;
	const LayerIndex MinLayerIndex = VolumeNavData->GetMinLayerIndexForAgentSize(AgentRadius);
	
    // Resolve endpoints to nearest navigable nodes using volume data
    if (!VolumeNavData->GetNodeAddressFromPosition(StartAddress, Request.StartLocation, MinLayerIndex))
    {
        UE_LOG(LogNav3D, Warning, TEXT("Lazy Theta*: Could not resolve start location %s to a navigable node"), *Request.StartLocation.ToString());
        return ENavigationQueryResult::Error;
    }
    if (!VolumeNavData->GetNodeAddressFromPosition(GoalAddress, Request.EndLocation, MinLayerIndex))
    {
        UE_LOG(LogNav3D, Warning, TEXT("Lazy Theta*: Could not resolve goal location %s to a navigable node"), *Request.EndLocation.ToString());
        return ENavigationQueryResult::Error;
    }
	
	// Initialize start node
	FSearchNode& StartNode = AllNodes.Add(StartAddress);
	StartNode.Address = StartAddress;
	StartNode.GScore = 0.0f;
	StartNode.FScore = CalculateHeuristic(StartAddress, GoalAddress);
	StartNode.bInOpenSet = true;
	OpenSet.Add(StartAddress);

	int32 Iteration = 0;
	int32 LineOfSightChecks = 0;
	constexpr int32 MaxIterations = 10000;

	// Main Lazy Theta* loop
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

		// Lazy Theta*: Check line of sight when node is expanded, not when added
		ProcessCurrentNodeWithLazyLOS(CurrentNode, LineOfSightChecks);

		// Log progress periodically
		if (LogVerbosity >= ENav3DPathingLogVerbosity::Detailed && (Iteration % 100 == 0))
		{
			UE_LOG(LogNav3D, Verbose, TEXT("Lazy Theta*[%d]: OpenSet=%d, BestF=%.2f, LOSChecks=%d"), 
				   Iteration, OpenSet.Num(), BestFScore, LineOfSightChecks);
		}

		// Check if we reached the goal
		if (CurrentAddress == GoalAddress)
		{
			// CRITICAL FIX: Check line of sight from start to goal before accepting path
			// This prevents jumping directly through occlusions
			LineOfSightChecks++;
			if (!HasLineOfSight(StartAddress, GoalAddress))
			{
				// No direct line of sight - continue searching for a proper path
				UE_LOG(LogNav3D, Verbose, TEXT("Lazy Theta*: No direct line of sight from start to goal, continuing search"));
				ProcessCurrentNodeForNeighbors(CurrentNode);
				continue;
			}
			
			const ENavigationQueryResult::Type Result = ReconstructPath(OutPath, CurrentNode);
			LogPathfindingResult(Result, OutPath.GetPathPoints().Num(), TEXT("Lazy Theta*"));
			UE_LOG(LogNav3D, Log, TEXT("Lazy Theta*: Completed with %d line-of-sight checks"), LineOfSightChecks);
			return Result;
		}

		// Process neighbors (without immediate LOS checks)
		ProcessCurrentNodeForNeighbors(CurrentNode);
	}

	if (Iteration >= MaxIterations)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Lazy Theta*: Reached maximum iteration limit (%d)"), MaxIterations);
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("Lazy Theta*: No path found - open set exhausted"));
	}

	return ENavigationQueryResult::Fail;
}

void FNav3DLazyThetaStar::ProcessCurrentNodeWithLazyLOS(FSearchNode& CurrentNode, int32& LineOfSightChecks)
{
	// Lazy evaluation: check line of sight from parent's parent when expanding current node
	if (CurrentNode.Parent.IsValid())
	{
		if (const FSearchNode* ParentNode = AllNodes.Find(CurrentNode.Parent))
		{
			if (ParentNode->Parent.IsValid())
			{
				LineOfSightChecks++;
				
				if (!HasLineOfSight(ParentNode->Parent, CurrentNode.Address))
				{
					// No line of sight - update current node to use best neighbor of parent as parent
					UpdateVertexLazy(CurrentNode);
				}
			}
		}
	}
}

void FNav3DLazyThetaStar::UpdateVertexLazy(FSearchNode& CurrentNode)
{
    // Find the best parent for current node from its parent's neighbors
    if (!CurrentNode.Parent.IsValid())
        return;

    const FSearchNode* ParentNode = AllNodes.Find(CurrentNode.Parent);
    if (!ParentNode)
        return;

    // Get parent's neighbors using actual API
    TArray<FNav3DNodeAddress> ParentNeighbors;
    VolumeData->GetNodeNeighbours(ParentNeighbors, ParentNode->Address);

    float BestGScore = TNumericLimits<float>::Max();
    FNav3DNodeAddress BestParent;

    // Check each neighbor of parent as potential parent for current
    for (const FNav3DNodeAddress& NeighborAddress : ParentNeighbors)
    {
        // Avoid selecting current node as its own parent (would create a cycle)
        if (NeighborAddress == CurrentNode.Address)
        {
            continue;
        }
        if (const FSearchNode* NeighborNode = AllNodes.Find(NeighborAddress))
        {
            if (NeighborNode->bInClosedSet)
            {
                const float TentativeGScore = NeighborNode->GScore + CalculateDistance(NeighborAddress, CurrentNode.Address);
                if (TentativeGScore < BestGScore)
                {
                    BestGScore = TentativeGScore;
                    BestParent = NeighborAddress;
                }
            }
        }
    }

    // CRITICAL FIX: If no valid parent found, fall back to using the parent's parent
    // or in worst case, the parent itself (which should be valid)
    if (!BestParent.IsValid() || BestParent == CurrentNode.Address)
    {
        if (ParentNode->Parent.IsValid())
        {
            BestParent = ParentNode->Parent;
            BestGScore = ParentNode->GScore + CalculateDistance(ParentNode->Address, CurrentNode.Address);
        }
        else
        {
            // Fallback to parent - this should prevent invalid chains
            BestParent = CurrentNode.Parent;
            BestGScore = ParentNode->GScore + CalculateDistance(CurrentNode.Parent, CurrentNode.Address);
        }
    }

    // Update current node with best parent found
    CurrentNode.Parent = BestParent;
    CurrentNode.GScore = BestGScore;
    const float HeuristicCost = CalculateHeuristic(CurrentNode.Address, GoalAddress);
    const float TotalCost = BestGScore + HeuristicCost;
    CurrentNode.FScore = AdjustTotalCostWithNodeSizeCompensation(TotalCost, CurrentNode.Address);
}

void FNav3DLazyThetaStar::ProcessCurrentNodeForNeighbors(const FSearchNode& CurrentNode)
{
	// Standard neighbor processing without immediate line of sight checks
	TArray<FNav3DNodeAddress> Neighbors;
	VolumeData->GetNodeNeighbours(Neighbors, CurrentNode.Address);

	for (const FNav3DNodeAddress& NeighborAddress : Neighbors)
	{
		// Skip if neighbor is in closed set
		if (const FSearchNode* ExistingNeighbor = AllNodes.Find(NeighborAddress))
		{
			if (ExistingNeighbor->bInClosedSet)
				continue;
		}

		// Calculate tentative G score (assume line of sight for now)
		float TentativeGScore;
		FNav3DNodeAddress TentativeParent;

		// FIXED: Use current node as parent, not current node's parent
		// This ensures proper exploration and prevents direct jumps through occlusions
		TentativeGScore = CurrentNode.GScore + CalculateDistance(CurrentNode.Address, NeighborAddress);
		TentativeParent = CurrentNode.Address;

		// Get or create neighbor node
		FSearchNode& NeighborNode = AllNodes.FindOrAdd(NeighborAddress);
		if (NeighborNode.Address.IsValid() == false)
		{
			// New node
			NeighborNode.Address = NeighborAddress;
			NeighborNode.GScore = TNumericLimits<float>::Max();
		}

		// Check if this path to neighbor is better
        if (TentativeGScore < NeighborNode.GScore)
		{
			NeighborNode.Parent = TentativeParent;
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
}
