#include "Pathfinding/Search/Nav3DThetaStar.h"
#include "Nav3DData.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Nav3D.h"

FNav3DThetaStar::FNav3DThetaStar()
{
}

FNav3DThetaStar::~FNav3DThetaStar()
{
}

ENavigationQueryResult::Type FNav3DThetaStar::FindPath(
	FNav3DPath& OutPath,
	const FNav3DPathingRequest& Request,
	const FNav3DVolumeNavigationData* VolumeNavData)
{
	LogPathfindingStart(Request, TEXT("Theta*"));
	
	if (!VolumeNavData)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Theta*: No volume navigation data provided"));
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
        UE_LOG(LogNav3D, Warning, TEXT("Theta*: Could not resolve start location %s to a navigable node"), *Request.StartLocation.ToString());
        return ENavigationQueryResult::Error;
    }
    if (!VolumeNavData->GetNodeAddressFromPosition(GoalAddress, Request.EndLocation, MinLayerIndex))
    {
        UE_LOG(LogNav3D, Warning, TEXT("Theta*: Could not resolve goal location %s to a navigable node"), *Request.EndLocation.ToString());
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

	// Main Theta* loop
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
			UE_LOG(LogNav3D, Verbose, TEXT("Theta*[%d]: OpenSet=%d, BestF=%.2f, LOSChecks=%d"), 
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
				UE_LOG(LogNav3D, Verbose, TEXT("Theta*: No direct line of sight from start to goal, continuing search"));
				ProcessCurrentNodeWithLineOfSight(CurrentNode, LineOfSightChecks);
				continue;
			}
			
			const ENavigationQueryResult::Type Result = ReconstructPath(OutPath, CurrentNode);
			LogPathfindingResult(Result, OutPath.GetPathPoints().Num(), TEXT("Theta*"));
			UE_LOG(LogNav3D, Log, TEXT("Theta*: Completed with %d line-of-sight checks"), LineOfSightChecks);
			return Result;
		}

		// Process neighbors with line of sight optimization
		ProcessCurrentNodeWithLineOfSight(CurrentNode, LineOfSightChecks);
	}

	if (Iteration >= MaxIterations)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Theta*: Reached maximum iteration limit (%d)"), MaxIterations);
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("Theta*: No path found - open set exhausted"));
	}

	return ENavigationQueryResult::Fail;
}

void FNav3DThetaStar::ProcessCurrentNodeWithLineOfSight(const FSearchNode& CurrentNode, int32& LineOfSightChecks)
{
    // Get neighbors for current node using actual API
    TArray<FNav3DNodeAddress> Neighbors;
    VolumeData->GetNodeNeighbours(Neighbors, CurrentNode.Address);

	for (const FNav3DNodeAddress& NeighborAddress : Neighbors)
	{
        // Skip self-loops which lead to duplicated points
        if (NeighborAddress == CurrentNode.Address)
        {
            continue;
        }
        ProcessNeighborWithLineOfSight(NeighborAddress, CurrentNode, LineOfSightChecks);
	}
}

void FNav3DThetaStar::ProcessNeighborWithLineOfSight(const FNav3DNodeAddress& NeighborAddress, const FSearchNode& CurrentNode, int32& LineOfSightChecks)
{
	// Skip if neighbor is in closed set
	if (const FSearchNode* ExistingNeighbor = AllNodes.Find(NeighborAddress))
	{
		if (ExistingNeighbor->bInClosedSet)
			return;
	}

	// Get or create neighbor node
	FSearchNode& NeighborNode = AllNodes.FindOrAdd(NeighborAddress);
	if (NeighborNode.Address.IsValid() == false)
	{
		// New node
		NeighborNode.Address = NeighborAddress;
		NeighborNode.GScore = TNumericLimits<float>::Max();
	}

    // Theta* optimization: check line of sight from current's parent to neighbor
    const FNav3DNodeAddress ParentAddress = CurrentNode.Parent;
	float TentativeGScore;
	FNav3DNodeAddress TentativeParent;

	if (ParentAddress.IsValid())
	{
		LineOfSightChecks++;
		
		if (HasLineOfSight(ParentAddress, NeighborAddress))
		{
			// Direct path from parent to neighbor
			if (const FSearchNode* ParentNode = AllNodes.Find(ParentAddress))
			{
                TentativeGScore = ParentNode->GScore + CalculateDistance(ParentAddress, NeighborAddress);
				TentativeParent = ParentAddress;
			}
			else
			{
				// Fallback to current node
				TentativeGScore = CurrentNode.GScore + CalculateDistance(CurrentNode.Address, NeighborAddress);
				TentativeParent = CurrentNode.Address;
			}
		}
		else
		{
			// No line of sight, use current node as parent
			TentativeGScore = CurrentNode.GScore + CalculateDistance(CurrentNode.Address, NeighborAddress);
			TentativeParent = CurrentNode.Address;
		}
	}
	else
	{
		// Current is start node
		TentativeGScore = CurrentNode.GScore + CalculateDistance(CurrentNode.Address, NeighborAddress);
		TentativeParent = CurrentNode.Address;
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

bool FNav3DThetaStar::HasLineOfSight(const FNav3DNodeAddress& From, const FNav3DNodeAddress& To) const
{
	if (!VolumeData)
		return false;

    const UNav3DRaycaster* RaycasterToUse = Raycaster;
	if (RaycasterToUse == nullptr)
	{
		RaycasterToUse = NewObject<UNav3DRaycaster>();
	}

	// Get adjusted positions (handles start/end locations vs node positions)
	auto GetAdjustedPosition = [this](const FNav3DNodeAddress& Address) -> FVector
	{
		if (Address == StartAddress)
		{
			return CurrentRequest.StartLocation;
		}
		if (Address == GoalAddress)
		{
			return CurrentRequest.EndLocation;
		}
		return VolumeData->GetNodePositionFromAddress(Address, true);
	};

	const FVector FromPosition = GetAdjustedPosition(From);
	const FVector ToPosition = GetAdjustedPosition(To);

	// Check if either endpoint is navigable using stored occlusion data
	bool bFromNavigable = false;
	bool bToNavigable = false;
	
	// Check From node navigability
	if (From.LayerIndex == 0)
	{
		const auto& LeafNodes = VolumeData->GetData().GetLeafNodes();
		if (LeafNodes.GetLeafNodes().IsValidIndex(From.NodeIndex))
		{
			const auto& LeafNode = LeafNodes.GetLeafNode(From.NodeIndex);
			bFromNavigable = !LeafNode.IsSubNodeOccluded(From.SubNodeIndex);
		}
	}
	else
	{
		const auto& Node = VolumeData->GetNodeFromAddress(From);
		bFromNavigable = !Node.HasChildren();
	}
	
	// Check To node navigability
	if (To.LayerIndex == 0)
	{
		const auto& LeafNodes = VolumeData->GetData().GetLeafNodes();
		if (LeafNodes.GetLeafNodes().IsValidIndex(To.NodeIndex))
		{
			const auto& LeafNode = LeafNodes.GetLeafNode(To.NodeIndex);
			bToNavigable = !LeafNode.IsSubNodeOccluded(To.SubNodeIndex);
		}
	}
	else
	{
		const auto& Node = VolumeData->GetNodeFromAddress(To);
		bToNavigable = !Node.HasChildren();
	}
	
	if (!bFromNavigable || !bToNavigable)
	{
		return false;
	}

	// Use raycaster to check for occlusion along the path
	FNav3DRaycastHit Hit;
	const bool bHit = RaycasterToUse->Trace(*VolumeData, FromPosition, ToPosition, Hit);

	// If we hit something, there's no line of sight
	return !bHit;
}
