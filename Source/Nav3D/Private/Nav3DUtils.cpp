#include "Nav3DUtils.h"

#include "Nav3D.h"
#include "GameFramework/NavMovementComponent.h"
#include "ThirdParty/libmorton/morton.h"
#include "Nav3DDataChunk.h"
#include "Nav3DDataChunkActor.h"
#include "Pathfinding/Nav3DCrossVolumeGraph.h"
#include "NavigationSystem.h"
#include "Nav3DData.h"

MortonCode FNav3DUtils::GetMortonCodeFromVector(const FVector& Vector)
{
	return morton3D_64_encode(Vector.X, Vector.Y, Vector.Z);
}

MortonCode FNav3DUtils::GetMortonCodeFromVector(const FIntVector& Vector)
{
	return morton3D_64_encode(Vector.X, Vector.Y, Vector.Z);
}

FVector FNav3DUtils::GetVectorFromMortonCode(const MortonCode MortonCode)
{
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);

	return FVector(X, Y, Z);
}

MortonCode FNav3DUtils::GetParentMortonCode(const MortonCode ChildMortonCode)
{
	return ChildMortonCode >> 3;
}

MortonCode FNav3DUtils::GetFirstChildMortonCode(const MortonCode ParentMortonCode)
{
	return ParentMortonCode << 3;
}

FVector FNav3DUtils::GetSubNodeOffset(const SubNodeIndex SubIdx, const float NodeExtent)
{
	// Convert morton index to 3D coordinates
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(SubIdx, X, Y, Z);

	const float SubNodeSize = NodeExtent * 0.5f;
	return FVector(
		(X * SubNodeSize) - NodeExtent,
		(Y * SubNodeSize) - NodeExtent,
		(Z * SubNodeSize) - NodeExtent
	);
}

ENavigationQueryResult::Type
FNav3DUtils::GraphAStarResultToNavigationTypeResult(
	const EGraphAStarResult Result)
{
	constexpr ENavigationQueryResult::Type ResultConversionTable[] = {
		ENavigationQueryResult::Fail, ENavigationQueryResult::Success,
		ENavigationQueryResult::Fail, ENavigationQueryResult::Fail
	};

	return ResultConversionTable[static_cast<int>(Result)];
}

bool FNav3DUtils::RayBoxIntersection(const FBox& Box, const FVector& RayOrigin, const FVector& RayDir,
                                     const float RayLength, float& OutTMin, float& OutTMax)
{
	// Calculate inverse ray direction for efficient tests
	const FVector InvDir(
		FMath::IsNearlyZero(RayDir.X) ? BIG_NUMBER : 1.0f / RayDir.X,
		FMath::IsNearlyZero(RayDir.Y) ? BIG_NUMBER : 1.0f / RayDir.Y,
		FMath::IsNearlyZero(RayDir.Z) ? BIG_NUMBER : 1.0f / RayDir.Z
	);

	// Calculate intersections with axis-aligned planes
	float TMin = -BIG_NUMBER;
	float TMax = BIG_NUMBER;

	for (int32 i = 0; i < 3; i++)
	{
		const float RayOrig = i == 0 ? RayOrigin.X : (i == 1 ? RayOrigin.Y : RayOrigin.Z);
		const float InvRayDir = i == 0 ? InvDir.X : (i == 1 ? InvDir.Y : InvDir.Z);
		const float BoxMin = i == 0 ? Box.Min.X : (i == 1 ? Box.Min.Y : Box.Min.Z);
		const float BoxMax = i == 0 ? Box.Max.X : (i == 1 ? Box.Max.Y : Box.Max.Z);

		if (FMath::Abs(InvRayDir) < SMALL_NUMBER)
		{
			// Ray parallel to axis - check if ray origin is within box planes
			if (RayOrig < BoxMin || RayOrig > BoxMax)
			{
				return false;
			}
		}
		else
		{
			float T1 = (BoxMin - RayOrig) * InvRayDir;
			float T2 = (BoxMax - RayOrig) * InvRayDir;

			if (T1 > T2)
			{
				const float Temp = T1;
				T1 = T2;
				T2 = Temp;
			}

			TMin = FMath::Max(T1, TMin);
			TMax = FMath::Min(T2, TMax);

			if (TMin > TMax || TMax < 0.0f)
			{
				return false;
			}
		}
	}

	// Check if intersection is within ray length
	if (TMin > RayLength)
	{
		return false;
	}

	TMax = FMath::Min(TMax, RayLength);

	OutTMin = TMin;
	OutTMax = TMax;
	return true;
}

FNavAgentProperties FNav3DUtils::GetNavAgentPropsFromQuerier(const UObject* Querier)
{
	if (const AActor* Actor = Cast<AActor>(Querier))
	{
		if (const UNavMovementComponent* MoveComp = Actor->FindComponentByClass<UNavMovementComponent>())
		{
			return MoveComp->GetNavAgentPropertiesRef();
		}
	}
	return FNavAgentProperties::DefaultProperties;
}

void FNav3DUtils::IdentifyBoundaryVoxels(UNav3DDataChunk* Chunk)
{
	if (!Chunk)
	{
		return;
	}

	Chunk->BoundaryVoxels.Empty();
	Chunk->MortonToBoundaryIndex.Empty();

	for (int32 VolIdx = 0; VolIdx < Chunk->NavigationData.Num(); ++VolIdx)
	{
		const FNav3DVolumeNavigationData& Volume = Chunk->NavigationData[VolIdx];
		const FNav3DData& Data = Volume.GetData();
		if (!Data.IsValid() || Data.GetLayerCount() == 0)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("IdentifyBoundaryVoxels: Volume %d invalid or empty (IsValid=%s, Layers=%d)"),
			       VolIdx, Data.IsValid() ? TEXT("true") : TEXT("false"), Data.GetLayerCount());
			continue;
		}

		const FBox& Bounds = Data.GetNavigationBounds();
		const float AdjacencyClearance = Volume.GetSettings().GenerationSettings.AdjacencyClearance;
		int32 TotalAddedForVolume = 0;

		UE_LOG(LogNav3D, VeryVerbose, TEXT("IdentifyBoundaryVoxels: Vol=%d Bounds=%s AdjacencyClearance=%.3f"),
		       VolIdx, *Bounds.ToString(), AdjacencyClearance);

		// Check all layers, not just layer 0
		const int32 LayerCount = Data.GetLayerCount();
		for (LayerIndex LayerIdx = 0; LayerIdx < LayerCount; ++LayerIdx)
		{
			const FNav3DLayer& Layer = Data.GetLayer(LayerIdx);
			const TArray<FNav3DNode>& Nodes = Layer.GetNodes();
			
			// Get layer-specific voxel extent
			float VoxelExtent;
			if (LayerIdx == 0)
			{
				VoxelExtent = Data.GetLeafNodes().GetLeafNodeExtent();
			}
			else
			{
				VoxelExtent = Layer.GetNodeExtent();
			}
			
			const float Epsilon = FMath::Max(1.0f, VoxelExtent * 0.1f);
			int32 LayerAdded = 0;

			UE_LOG(LogNav3D, VeryVerbose, TEXT("IdentifyBoundaryVoxels: Vol=%d Layer=%d Nodes=%d VoxelExtent=%.3f"),
			       VolIdx, LayerIdx, Nodes.Num(), VoxelExtent);

			for (int32 NodeIdx = 0; NodeIdx < Nodes.Num(); ++NodeIdx)
			{
				const FNav3DNode& Node = Nodes[NodeIdx];
				
				// Check if node is navigable
				bool bNavigable = false;
				if (LayerIdx == 0)
				{
					// For leaf nodes, check if they have children and are not completely occluded
					if (Node.FirstChild.IsValid())
					{
						const FNav3DLeafNode& Leaf = Data.GetLeafNodes().GetLeafNode(Node.FirstChild.NodeIndex);
						bNavigable = !Leaf.IsCompletelyOccluded();
					}
				}
				else
				{
					// For non-leaf nodes, check if they don't have children (meaning they're free)
					bNavigable = !Node.HasChildren();
				}

				if (!bNavigable)
				{
					continue;
				}

				// Get world position for this layer
				FVector WorldPos;
				if (LayerIdx == 0)
				{
					WorldPos = Volume.GetLeafNodePositionFromMortonCode(Node.MortonCode);
				}
				else
				{
					WorldPos = Volume.GetNodePositionFromLayerAndMortonCode(LayerIdx, Node.MortonCode);
				}

				// Check which boundary faces this voxel is on
				const bool bOnMinXFace = (WorldPos.X - VoxelExtent) <= (Bounds.Min.X + Epsilon);
				const bool bOnMaxXFace = (WorldPos.X + VoxelExtent) >= (Bounds.Max.X - Epsilon);
				const bool bOnMinYFace = (WorldPos.Y - VoxelExtent) <= (Bounds.Min.Y + Epsilon);
				const bool bOnMaxYFace = (WorldPos.Y + VoxelExtent) >= (Bounds.Max.Y - Epsilon);
				const bool bOnMinZFace = (WorldPos.Z - VoxelExtent) <= (Bounds.Min.Z + Epsilon);
				const bool bOnMaxZFace = (WorldPos.Z + VoxelExtent) >= (Bounds.Max.Z - Epsilon);
				
				const bool bOnBoundary = bOnMinXFace || bOnMaxXFace || bOnMinYFace || bOnMaxYFace || bOnMinZFace || bOnMaxZFace;

				if (bOnBoundary)
				{
					FNav3DEdgeVoxel Edge;
					Edge.Morton = Node.MortonCode;
					Edge.bIsNavigable = 1;
					Edge.VolumeIndex = VolIdx;
					Edge.LayerIndex = LayerIdx;  // Store the layer index
					
					// Record which boundary faces this voxel is on
					Edge.bOnMinXFace = bOnMinXFace;
					Edge.bOnMaxXFace = bOnMaxXFace;
					Edge.bOnMinYFace = bOnMinYFace;
					Edge.bOnMaxYFace = bOnMaxYFace;
					Edge.bOnMinZFace = bOnMinZFace;
					Edge.bOnMaxZFace = bOnMaxZFace;
					
					const int32 Index = Chunk->BoundaryVoxels.Add(Edge);
					Chunk->MortonToBoundaryIndex.Add(Edge.Morton, Index);
					++LayerAdded;
				}
			}

			TotalAddedForVolume += LayerAdded;
			UE_LOG(LogNav3D, VeryVerbose, TEXT("IdentifyBoundaryVoxels: Vol=%d Layer=%d Added=%d"),
			       VolIdx, LayerIdx, LayerAdded);
		}

		UE_LOG(LogNav3D, Verbose, TEXT("IdentifyBoundaryVoxels: Vol=%d TotalAdded=%d"),
		       VolIdx, TotalAddedForVolume);
	}

	if (Chunk->BoundaryVoxels.Num() == 0)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("IdentifyBoundaryVoxels: No boundary voxels found for chunk (Volumes=%d)"), Chunk->NavigationData.Num());
	}
}

void FNav3DUtils::BuildAdjacencyBetweenChunks(UNav3DDataChunk* ChunkA, UNav3DDataChunk* ChunkB, const float VoxelSize, const float ConnectionThresholdMultiplier)
{
	if (!ChunkA || !ChunkB)
	{
		return;
	}

	// Get volume data for proper world position conversion
	const FNav3DVolumeNavigationData* VolumeA = nullptr;
	const FNav3DVolumeNavigationData* VolumeB = nullptr;
	
	if (ChunkA->NavigationData.Num() > 0)
	{
		VolumeA = &ChunkA->NavigationData[0];
	}
	if (ChunkB->NavigationData.Num() > 0)
	{
		VolumeB = &ChunkB->NavigationData[0];
	}
	
	if (!VolumeA || !VolumeB)
	{
		UE_LOG(LogNav3D, Warning, TEXT("BuildAdjacencyBetweenChunks: Missing volume data for chunks"));
		return;
	}

	const float AdjacencyClearance = VolumeA->GetSettings().GenerationSettings.AdjacencyClearance;

	for (FNav3DEdgeVoxel& VoxelA : ChunkA->BoundaryVoxels)
	{
		// Get world position for voxel A
		FVector PosA;
		if (VoxelA.LayerIndex == 0)
		{
			PosA = VolumeA->GetLeafNodePositionFromMortonCode(VoxelA.Morton);
		}
		else
		{
			PosA = VolumeA->GetNodePositionFromLayerAndMortonCode(VoxelA.LayerIndex, VoxelA.Morton);
		}

		struct FNeighbor { MortonCode Code; float Dist; };
		FNeighbor Best[3] = { {0, FLT_MAX}, {0, FLT_MAX}, {0, FLT_MAX} };

		for (const FNav3DEdgeVoxel& VoxelB : ChunkB->BoundaryVoxels)
		{
			// Get world position for voxel B
			FVector PosB;
			if (VoxelB.LayerIndex == 0)
			{
				PosB = VolumeB->GetLeafNodePositionFromMortonCode(VoxelB.Morton);
			}
			else
			{
				PosB = VolumeB->GetNodePositionFromLayerAndMortonCode(VoxelB.LayerIndex, VoxelB.Morton);
			}

			// Calculate distance between voxel centers
			const float CenterToCenterDist = FVector::Dist(PosA, PosB);
			
			// For adjacent volumes, use a simpler edge-to-edge distance calculation
			// This is more appropriate for truly adjacent volumes than boundary projection
			const FBox& BoundsA = VolumeA->GetNavigationBounds();
			const FBox& BoundsB = VolumeB->GetNavigationBounds();
			
			// Calculate edge-to-edge distance
			// For each volume, the distance from center to edge is half the extent size
			const float EdgeToEdgeDist = CenterToCenterDist - (BoundsA.GetExtent().Size() * 0.5f + BoundsB.GetExtent().Size() * 0.5f);
			
			// Voxels are adjacent if their edge-to-edge distance is within clearance
			const float Threshold = AdjacencyClearance;
			
			if (EdgeToEdgeDist >= 0.0f && EdgeToEdgeDist <= Threshold)
			{
				if (EdgeToEdgeDist < Best[0].Dist) { Best[2] = Best[1]; Best[1] = Best[0]; Best[0] = { VoxelB.Morton, EdgeToEdgeDist }; }
				else if (EdgeToEdgeDist < Best[1].Dist) { Best[2] = Best[1]; Best[1] = { VoxelB.Morton, EdgeToEdgeDist }; }
				else if (EdgeToEdgeDist < Best[2].Dist) { Best[2] = { VoxelB.Morton, EdgeToEdgeDist }; }
			}
		}

		for (const FNeighbor& N : Best)
		{
			if (N.Code != 0 && N.Dist < FLT_MAX)
			{
				VoxelA.AdjacentChunkVoxels.Add(N.Code);
			}
		}
	}
}

FBox FNav3DUtils::ComputeChunkBounds(const UNav3DDataChunk* Chunk)
{
	FBox Bounds(ForceInit);
	if (!Chunk)
	{
		return Bounds;
	}
	for (const FNav3DVolumeNavigationData& Volume : Chunk->NavigationData)
	{
		Bounds += Volume.GetData().GetNavigationBounds();
	}
	return Bounds;
}

bool FNav3DUtils::AreChunksAdjacent(const UNav3DDataChunk* ChunkA, const UNav3DDataChunk* ChunkB, const float Threshold)
{
	if (!ChunkA || !ChunkB)
	{
		return false;
	}
	const FBox A = ComputeChunkBounds(ChunkA).ExpandBy(Threshold);
	const FBox B = ComputeChunkBounds(ChunkB);
	return A.Intersect(B);
}

float FNav3DUtils::GetChunkLeafNodeSize(const UNav3DDataChunk* Chunk)
{
	if (!Chunk || Chunk->NavigationData.Num() == 0)
	{
		return 0.0f;
	}
	return Chunk->NavigationData[0].GetData().GetLeafNodes().GetLeafNodeSize();
}

void FNav3DUtils::BuildAdjacencyForChunk(UNav3DDataChunk* Chunk, const TArray<UNav3DDataChunk*>& OtherChunks, const float VoxelSize, const float ConnectionThresholdMultiplier)
{
	if (!Chunk)
	{
		return;
	}

	for (UNav3DDataChunk* Other : OtherChunks)
	{
		if (Other == nullptr || Other == Chunk)
		{
			continue;
		}
		if (AreChunksAdjacent(Chunk, Other, VoxelSize))
		{
			BuildAdjacencyBetweenChunks(Chunk, Other, VoxelSize, ConnectionThresholdMultiplier);
		}
	}
}

ANav3DData* FNav3DUtils::GetNav3DData(const UWorld* World)
{
	if (!World) return nullptr;
	if (const UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
	{
		for (ANavigationData* NavData : NavSys->NavDataSet)
		{
			if (ANav3DData* Nav3D = Cast<ANav3DData>(NavData))
			{
				return Nav3D;
			}
		}
	}
	return nullptr;
}

FLinearColor FNav3DUtils::GetChunkColorByIndex(int32 ChunkIndex)
{
	static const TArray<FLinearColor> Palette = {
		FLinearColor(0.0f, 1.0f, 1.0f),
		FLinearColor::Green,
		FLinearColor::Blue,
		FLinearColor::Yellow,
		FLinearColor(1.0f, 0.5f, 0.0f),
		FLinearColor(0.5f, 0.0f, 1.0f),
		FLinearColor(0.0f, 1.0f, 0.5f),
		FLinearColor(1.0f, 0.0f, 0.5f),
		FLinearColor(0.5f, 1.0f, 0.0f),
		FLinearColor(0.0f, 0.5f, 1.0f),
	};
	if (Palette.Num() == 0)
	{
		return FLinearColor::White;
	}
	return Palette[FMath::Abs(ChunkIndex) % Palette.Num()];
}

// Endpoint projection utilities for cross-volume pathfinding

FNav3DUtils::FEndpointProjectionResult FNav3DUtils::ProjectPointToFreeVoxel(
	const FNav3DVolumeNavigationData& VolumeData,
	const FVector& InputPosition,
	const FNavAgentProperties& AgentProperties,
	LayerIndex MinLayerIndex,
	float MaxSearchRadius,
	int32 MaxSearchIterations)
{
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPointToFreeVoxel: Projecting point %s with agent radius %.2f"), 
	       *InputPosition.ToString(), AgentProperties.AgentRadius);

	// First, clamp the input position to be within navigation bounds
	const FBox& NavBounds = VolumeData.GetNavigationBounds();
	FVector ClampedPosition = InputPosition;
	
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPointToFreeVoxel: Input position %s, NavBounds %s"), 
		*InputPosition.ToString(), *NavBounds.ToString());
	
	if (!NavBounds.IsInside(ClampedPosition))
	{
		ClampedPosition = NavBounds.GetClosestPointTo(ClampedPosition);
		UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPointToFreeVoxel: Clamped position to %s"), *ClampedPosition.ToString());
	}

	// Try direct resolution first
	FNav3DNodeAddress NodeAddress;
	if (VolumeData.GetNodeAddressFromPosition(NodeAddress, ClampedPosition, MinLayerIndex))
	{
		// Verify the node is actually navigable for this agent
		const FNav3DNode& Node = VolumeData.GetNodeFromAddress(NodeAddress);
		
		// Check if node is navigable - nodes without children are free
		bool bIsNavigable = false;
		if (NodeAddress.LayerIndex == 0)
		{
			// For leaf nodes, check if the specific subnode is free
			const auto& LeafNodes = VolumeData.GetData().GetLeafNodes();
			if (LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
			{
				const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
				bIsNavigable = !LeafNode.IsSubNodeOccluded(NodeAddress.SubNodeIndex);
			}
		}
		else
		{
			// For non-leaf nodes, check if they don't have children (meaning they're free)
			bIsNavigable = !Node.HasChildren();
		}
		
		if (bIsNavigable)
		{
			FVector NodePosition = VolumeData.GetNodePositionFromAddress(NodeAddress, false);
			UE_LOG(LogNav3D, Verbose, TEXT("ProjectPointToFreeVoxel: Direct resolution successful at layer %d, node %d, subnode %d, position %s"), 
				NodeAddress.LayerIndex, NodeAddress.NodeIndex, NodeAddress.SubNodeIndex, *NodePosition.ToString());
			return FEndpointProjectionResult(true, NodePosition, NodeAddress, NodeAddress.LayerIndex);
		}
		else
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPointToFreeVoxel: Direct resolution found node but it's not navigable at layer %d, node %d, subnode %d"), 
				NodeAddress.LayerIndex, NodeAddress.NodeIndex, NodeAddress.SubNodeIndex);
		}
	}

	// If direct resolution failed, try layer fallback
	const LayerIndex LayerCount = VolumeData.GetLayerCount();
	for (LayerIndex TestLayer = MinLayerIndex + 1; TestLayer < LayerCount; ++TestLayer)
	{
		UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPointToFreeVoxel: Trying layer fallback at layer %d"), TestLayer);
		if (VolumeData.GetNodeAddressFromPosition(NodeAddress, ClampedPosition, TestLayer))
		{
			const FNav3DNode& Node = VolumeData.GetNodeFromAddress(NodeAddress);
			
			// Check if node is navigable - nodes without children are free
			bool bIsNavigable = false;
			if (NodeAddress.LayerIndex == 0)
			{
				// For leaf nodes, check if the specific subnode is free
				const auto& LeafNodes = VolumeData.GetData().GetLeafNodes();
				if (LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
				{
					const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
					bIsNavigable = !LeafNode.IsSubNodeOccluded(NodeAddress.SubNodeIndex);
				}
			}
			else
			{
				// For non-leaf nodes, check if they don't have children (meaning they're free)
				bIsNavigable = !Node.HasChildren();
			}
			
			if (bIsNavigable)
			{
				FVector NodePosition = VolumeData.GetNodePositionFromAddress(NodeAddress, false);
				UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPointToFreeVoxel: Layer fallback successful at layer %d"), TestLayer);
				return FEndpointProjectionResult(true, NodePosition, NodeAddress, TestLayer);
			}
		}
	}

	// If layer fallback failed, try spatial search
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPointToFreeVoxel: Attempting spatial search within radius %.2f"), MaxSearchRadius);
	return FindNearestFreeVoxel(VolumeData, ClampedPosition, MaxSearchRadius, AgentProperties, MinLayerIndex, MaxSearchIterations);
}

// ============================================================================
// Cross-volume adjacency utilities
// ============================================================================

TArray<FNav3DEdgeVoxel> FNav3DUtils::ExtractCrossVolumeBoundaryVoxels(UNav3DDataChunk* Chunk, int32 /*ChunkIndex*/)
{
	TArray<FNav3DEdgeVoxel> Result;
	if (!Chunk)
	{
		return Result;
	}
	if (Chunk->BoundaryVoxels.Num() == 0)
	{
		IdentifyBoundaryVoxels(Chunk);
	}
	// Only include navigable (free) boundary voxels
	Result.Reserve(Chunk->BoundaryVoxels.Num());
	for (const FNav3DEdgeVoxel& V : Chunk->BoundaryVoxels)
	{
		if (V.bIsNavigable)
		{
			Result.Add(V);
		}
	}
	return Result;
}

static const FNav3DVolumeNavigationData* ResolveVolumeDataForVoxelID(const FNav3DVoxelID& VoxelID, const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
	if (!ChunkActors.IsValidIndex(VoxelID.ChunkIndex)) { return nullptr; }
	const ANav3DDataChunkActor* Actor = ChunkActors[VoxelID.ChunkIndex];
	if (!Actor) { return nullptr; }
	if (Actor->Nav3DChunks.Num() == 0) { return nullptr; }
	const UNav3DDataChunk* Chunk = Actor->Nav3DChunks[0];
	if (!Chunk) { return nullptr; }
	if (!Chunk->NavigationData.IsValidIndex(VoxelID.VolumeIndex)) { return nullptr; }
	return &Chunk->NavigationData[VoxelID.VolumeIndex];
}

FVector FNav3DUtils::GetVoxelWorldPosition(const FNav3DVoxelID& VoxelID, const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
	if (const FNav3DVolumeNavigationData* Vol = ResolveVolumeDataForVoxelID(VoxelID, ChunkActors))
	{
		if (VoxelID.Layer == 0)
		{
			return Vol->GetLeafNodePositionFromMortonCode(VoxelID.Morton);
		}
		return Vol->GetNodePositionFromLayerAndMortonCode(VoxelID.Layer, VoxelID.Morton);
	}
	return FVector::ZeroVector;
}

float FNav3DUtils::GetVoxelExtent(const FNav3DVoxelID& VoxelID, const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
	if (const FNav3DVolumeNavigationData* Vol = ResolveVolumeDataForVoxelID(VoxelID, ChunkActors))
	{
		if (VoxelID.Layer == 0)
		{
			return Vol->GetData().GetLeafNodes().GetLeafNodeExtent();
		}
		return Vol->GetData().GetLayer(VoxelID.Layer).GetNodeExtent();
	}
	return 0.0f;
}

FBox FNav3DUtils::GetVoxelWorldBounds(const FNav3DVoxelID& VoxelID, const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
	const FVector Center = GetVoxelWorldPosition(VoxelID, ChunkActors);
	const float Ext = GetVoxelExtent(VoxelID, ChunkActors);
	return FBox(Center - FVector(Ext), Center + FVector(Ext));
}

bool FNav3DUtils::AreFaceAdjacent(const FNav3DVoxelID& A, const FNav3DVoxelID& B, const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
	const FBox BoxA = GetVoxelWorldBounds(A, ChunkActors);
	const FBox BoxB = GetVoxelWorldBounds(B, ChunkActors);
	// Face adjacency: boxes touch on one axis with overlap on the other two
	auto OverlapOn = [](float Amin, float Amax, float Bmin, float Bmax) { return !(Amax < Bmin || Bmax < Amin); };

	const bool TouchX = FMath::IsNearlyEqual(BoxA.Max.X, BoxB.Min.X) || FMath::IsNearlyEqual(BoxB.Max.X, BoxA.Min.X);
	const bool OverlapY = OverlapOn(BoxA.Min.Y, BoxA.Max.Y, BoxB.Min.Y, BoxB.Max.Y);
	const bool OverlapZ = OverlapOn(BoxA.Min.Z, BoxA.Max.Z, BoxB.Min.Z, BoxB.Max.Z);
	if (TouchX && OverlapY && OverlapZ) return true;

	const bool TouchY = FMath::IsNearlyEqual(BoxA.Max.Y, BoxB.Min.Y) || FMath::IsNearlyEqual(BoxB.Max.Y, BoxA.Min.Y);
	const bool OverlapX = OverlapOn(BoxA.Min.X, BoxA.Max.X, BoxB.Min.X, BoxB.Max.X);
	if (TouchY && OverlapX && OverlapZ) return true;

	const bool TouchZ = FMath::IsNearlyEqual(BoxA.Max.Z, BoxB.Min.Z) || FMath::IsNearlyEqual(BoxB.Max.Z, BoxA.Min.Z);
	if (TouchZ && OverlapX && OverlapY) return true;

	return false;
}

FVector FNav3DUtils::CalculateSharedFacePortal(const FNav3DVoxelID& A, const FNav3DVoxelID& B, const TArray<ANav3DDataChunkActor*>& ChunkActors)
{
	const FVector PosA = GetVoxelWorldPosition(A, ChunkActors);
	const FVector PosB = GetVoxelWorldPosition(B, ChunkActors);
	const float ExtA = GetVoxelExtent(A, ChunkActors);
	const float ExtB = GetVoxelExtent(B, ChunkActors);

	const bool bAIsSmaller = ExtA < ExtB;
	const FNav3DVoxelID& Smaller = bAIsSmaller ? A : B;
	const FVector SmallerPos = bAIsSmaller ? PosA : PosB;
	const FVector LargerPos = bAIsSmaller ? PosB : PosA;
	const FBox SmallerBox = GetVoxelWorldBounds(Smaller, ChunkActors);

	const FVector Dir = (LargerPos - SmallerPos).GetSafeNormal();
	FVector Portal = SmallerPos;
	const float ax = FMath::Abs(Dir.X), ay = FMath::Abs(Dir.Y), az = FMath::Abs(Dir.Z);
	if (ax >= ay && ax >= az)
	{
		Portal.X = Dir.X > 0 ? SmallerBox.Max.X : SmallerBox.Min.X;
	}
	else if (ay >= az)
	{
		Portal.Y = Dir.Y > 0 ? SmallerBox.Max.Y : SmallerBox.Min.Y;
	}
	else
	{
		Portal.Z = Dir.Z > 0 ? SmallerBox.Max.Z : SmallerBox.Min.Z;
	}
	return Portal;
}

bool FNav3DUtils::FindCrossActorBoundaryPortal(const ANav3DDataChunkActor* FromActor,
	const ANav3DDataChunkActor* ToActor,
	FVector& OutLocalPortal,
	FVector& OutRemotePortal,
	FVector& OutPortalLocation)
{
	if (!FromActor || !ToActor) { return false; }
	if (FromActor->Nav3DChunks.Num() == 0 || ToActor->Nav3DChunks.Num() == 0) { return false; }
	const UNav3DDataChunk* FromChunk = FromActor->Nav3DChunks[0];
	const UNav3DDataChunk* ToChunk = ToActor->Nav3DChunks[0];
	if (!FromChunk || !ToChunk) { return false; }

	// Extract free boundary voxels
	TArray<FNav3DEdgeVoxel> FromEdges = FromChunk->BoundaryVoxels;
	TArray<FNav3DEdgeVoxel> ToEdges = ToChunk->BoundaryVoxels;
	FromEdges.RemoveAllSwap([](const FNav3DEdgeVoxel& E){ return !E.bIsNavigable; });
	ToEdges.RemoveAllSwap([](const FNav3DEdgeVoxel& E){ return !E.bIsNavigable; });

	// Prepare actors array for utility calls
	TArray<ANav3DDataChunkActor*> Actors;
	Actors.Add(const_cast<ANav3DDataChunkActor*>(FromActor));
	Actors.Add(const_cast<ANav3DDataChunkActor*>(ToActor));

	float BestDist2 = TNumericLimits<float>::Max();
	FNav3DVoxelID BestA, BestB;
	bool bFound = false;

	for (const FNav3DEdgeVoxel& A : FromEdges)
	{
		for (const FNav3DEdgeVoxel& B : ToEdges)
		{
			FNav3DVoxelID VA{ A.VolumeIndex, 0, A.LayerIndex, A.Morton };
			FNav3DVoxelID VB{ B.VolumeIndex, 1, B.LayerIndex, B.Morton };
			if (!AreFaceAdjacent(VA, VB, Actors)) { continue; }
			const FVector PA = GetVoxelWorldPosition(VA, Actors);
			const FVector PB = GetVoxelWorldPosition(VB, Actors);
			const float D2 = FVector::DistSquared(PA, PB);
			if (D2 < BestDist2)
			{
				BestDist2 = D2;
				BestA = VA; BestB = VB; bFound = true;
			}
		}
	}

	if (!bFound) { return false; }

	OutLocalPortal = GetVoxelWorldPosition(BestA, Actors);
	OutRemotePortal = GetVoxelWorldPosition(BestB, Actors);
	OutPortalLocation = CalculateSharedFacePortal(BestA, BestB, Actors);
	return true;
}

FNav3DUtils::FEndpointProjectionResult FNav3DUtils::ProjectPortalToFreeVoxel(
	const FNav3DVolumeNavigationData& VolumeData,
	const FNav3DVoxelConnection& Connection,
	bool bUseLocal,
	const FNavAgentProperties& AgentProperties,
	LayerIndex MinLayerIndex)
{
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPortalToFreeVoxel: Projecting portal connection (Local=%s, VolIdx=%d)"), 
	       bUseLocal ? TEXT("true") : TEXT("false"), bUseLocal ? Connection.LocalVolumeIndex : Connection.RemoteVolumeIndex);

	const uint64 MortonCode = bUseLocal ? Connection.Local : Connection.Remote;
	
	// First try to get the position from the morton code
	FVector PortalPosition = VolumeData.GetLeafNodePositionFromMortonCode(MortonCode);
	
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPortalToFreeVoxel: Portal position from morton code %llu: %s"), 
	       MortonCode, *PortalPosition.ToString());
	
	// Check if this position can be resolved back to a node address
	FNav3DNodeAddress TestAddress;
	if (VolumeData.GetNodeAddressFromPosition(TestAddress, PortalPosition, 0))
	{
		UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPortalToFreeVoxel: Portal position can be resolved to layer=%d, node=%d, subnode=%d"), 
		       TestAddress.LayerIndex, TestAddress.NodeIndex, TestAddress.SubNodeIndex);
	}
	else
	{
		UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPortalToFreeVoxel: Portal position CANNOT be resolved to a node address!"));
	}
	
	// Try to resolve this position to a node address
	FNav3DNodeAddress NodeAddress;
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPortalToFreeVoxel: Attempting to resolve portal position %s at layer %d"), 
	       *PortalPosition.ToString(), MinLayerIndex);
	
	if (VolumeData.GetNodeAddressFromPosition(NodeAddress, PortalPosition, MinLayerIndex))
	{
		const FNav3DNode& Node = VolumeData.GetNodeFromAddress(NodeAddress);
		
		// Check if node is navigable - nodes without children are free
		bool bIsNavigable = false;
		if (NodeAddress.LayerIndex == 0)
		{
			// For leaf nodes, check if the specific subnode is free
			const auto& LeafNodes = VolumeData.GetData().GetLeafNodes();
			if (LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
			{
				const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
				bIsNavigable = !LeafNode.IsSubNodeOccluded(NodeAddress.SubNodeIndex);
			}
		}
		else
		{
			// For non-leaf nodes, check if they don't have children (meaning they're free)
			bIsNavigable = !Node.HasChildren();
		}
		
		if (bIsNavigable)
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPortalToFreeVoxel: Direct portal resolution successful"));
			return FEndpointProjectionResult(true, PortalPosition, NodeAddress, NodeAddress.LayerIndex);
		}
	}

	// If direct resolution failed, try layer fallback
	const LayerIndex LayerCount = VolumeData.GetLayerCount();
	for (LayerIndex TestLayer = MinLayerIndex + 1; TestLayer < LayerCount; ++TestLayer)
	{
		if (VolumeData.GetNodeAddressFromPosition(NodeAddress, PortalPosition, TestLayer))
		{
			const FNav3DNode& Node = VolumeData.GetNodeFromAddress(NodeAddress);
			
			// Check if node is navigable - nodes without children are free
			bool bIsNavigable = false;
			if (NodeAddress.LayerIndex == 0)
			{
				// For leaf nodes, check if the specific subnode is free
				const auto& LeafNodes = VolumeData.GetData().GetLeafNodes();
				if (LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
				{
					const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
					bIsNavigable = !LeafNode.IsSubNodeOccluded(NodeAddress.SubNodeIndex);
				}
			}
			else
			{
				// For non-leaf nodes, check if they don't have children (meaning they're free)
				bIsNavigable = !Node.HasChildren();
			}
			
			if (bIsNavigable)
			{
				FVector NodePosition = VolumeData.GetNodePositionFromAddress(NodeAddress, false);
				UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPortalToFreeVoxel: Layer fallback successful at layer %d"), TestLayer);
				return FEndpointProjectionResult(true, NodePosition, NodeAddress, TestLayer);
			}
		}
	}

	// If all else fails, try spatial search around the portal position
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectPortalToFreeVoxel: Attempting spatial search around portal position"));
	return FindNearestFreeVoxel(VolumeData, PortalPosition, 200.0f, AgentProperties, MinLayerIndex, 15);
}

FNav3DUtils::FEndpointProjectionResult FNav3DUtils::ProjectBoundaryToNavigable(
	const FNav3DVolumeNavigationData& VolumeData,
	const FVector& BoundaryPosition,
	const FNavAgentProperties& AgentProperties,
	LayerIndex MinLayerIndex)
{
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectBoundaryToNavigable: Projecting boundary position %s"), *BoundaryPosition.ToString());

	// First, ensure the boundary position is within navigation bounds
	const FBox& NavBounds = VolumeData.GetNavigationBounds();
	FVector ClampedPosition = BoundaryPosition;
	
	if (!NavBounds.IsInside(ClampedPosition))
	{
		ClampedPosition = NavBounds.GetClosestPointTo(ClampedPosition);
		UE_LOG(LogNav3D, VeryVerbose, TEXT("ProjectBoundaryToNavigable: Clamped boundary position to %s"), *ClampedPosition.ToString());
	}

	// Try to project this to a free voxel
	return ProjectPointToFreeVoxel(VolumeData, ClampedPosition, AgentProperties, MinLayerIndex, 500.0f, 15);
}

bool FNav3DUtils::ValidatePortalConnection(
	const FNav3DVoxelConnection& Connection,
	const FNav3DVolumeNavigationData& LocalVolume,
	const FNav3DVolumeNavigationData& RemoteVolume,
	FString& OutValidationError)
{
	UE_LOG(LogNav3D, VeryVerbose, TEXT("ValidatePortalConnection: Validating connection between volumes"));

	// Check if the connection has valid morton codes
	if (Connection.Local == 0 && Connection.Remote == 0)
	{
		OutValidationError = TEXT("Portal connection has invalid morton codes (both zero)");
		return false;
	}

	// Check if volume indices are valid
	if (Connection.LocalVolumeIndex < 0 || Connection.RemoteVolumeIndex < 0)
	{
		OutValidationError = FString::Printf(TEXT("Portal connection has invalid volume indices (Local=%d, Remote=%d)"), 
			Connection.LocalVolumeIndex, Connection.RemoteVolumeIndex);
		return false;
	}

	// Try to resolve both portal positions
	FVector LocalPos = LocalVolume.GetLeafNodePositionFromMortonCode(Connection.Local);
	FVector RemotePos = RemoteVolume.GetLeafNodePositionFromMortonCode(Connection.Remote);

	// Check if positions are within their respective volume bounds
	if (!LocalVolume.GetNavigationBounds().IsInside(LocalPos))
	{
		OutValidationError = FString::Printf(TEXT("Local portal position %s is outside volume bounds"), *LocalPos.ToString());
		return false;
	}

	if (!RemoteVolume.GetNavigationBounds().IsInside(RemotePos))
	{
		OutValidationError = FString::Printf(TEXT("Remote portal position %s is outside volume bounds"), *RemotePos.ToString());
		return false;
	}

	// Check if the distance is reasonable - use dynamic threshold based on voxel size
	float Distance = FVector::Dist(LocalPos, RemotePos);
	float VoxelSize = LocalVolume.GetData().GetLeafNodes().GetLeafNodeSize();
	float MaxReasonableDistance = VoxelSize * 2.0f; // Allow up to 2 voxel lengths for adjacent volumes
	
	if (Distance > MaxReasonableDistance)
	{
		OutValidationError = FString::Printf(TEXT("Portal distance %.2f exceeds reasonable threshold (%.2f, voxel size %.2f)"), 
			Distance, MaxReasonableDistance, VoxelSize);
		return false;
	}

	UE_LOG(LogNav3D, VeryVerbose, TEXT("ValidatePortalConnection: Portal validation successful (distance=%.2f, voxel size=%.2f, threshold=%.2f)"), 
		Distance, VoxelSize, MaxReasonableDistance);
	return true;
}

FNav3DUtils::FEndpointProjectionResult FNav3DUtils::FindNearestFreeVoxel(
	const FNav3DVolumeNavigationData& VolumeData,
	const FVector& SearchCenter,
	float SearchRadius,
	const FNavAgentProperties& AgentProperties,
	LayerIndex MinLayerIndex,
	int32 MaxSearchIterations)
{
	UE_LOG(LogNav3D, VeryVerbose, TEXT("FindNearestFreeVoxel: Searching for free voxel around %s within radius %.2f"), 
	       *SearchCenter.ToString(), SearchRadius);

	const FBox& NavBounds = VolumeData.GetNavigationBounds();
	
	// Create a search pattern: start with small radius and expand
	float CurrentRadius = 50.0f; // Start with a small radius

	for (int32 Iteration = 0; Iteration < MaxSearchIterations && CurrentRadius <= SearchRadius; ++Iteration)
	{
		constexpr float RadiusIncrement = 50.0f;
		UE_LOG(LogNav3D, VeryVerbose, TEXT("FindNearestFreeVoxel: Iteration %d, searching radius %.2f"), Iteration, CurrentRadius);
		
		// Create a bounding box for this search iteration
		FBox SearchBounds(SearchCenter - FVector(CurrentRadius), SearchCenter + FVector(CurrentRadius));
		SearchBounds = SearchBounds.Overlap(NavBounds); // Clamp to navigation bounds
		
		// Sample points in a grid pattern within the search bounds
		constexpr int32 GridSize = 5; // 5x5x5 grid
		const FVector GridSpacing = SearchBounds.GetSize() / (GridSize - 1);
		
		for (int32 X = 0; X < GridSize; ++X)
		{
			for (int32 Y = 0; Y < GridSize; ++Y)
			{
				for (int32 Z = 0; Z < GridSize; ++Z)
				{
					FVector TestPosition = SearchBounds.Min + FVector(X * GridSpacing.X, Y * GridSpacing.Y, Z * GridSpacing.Z);
					
					// Try to resolve this position
					FNav3DNodeAddress NodeAddress;
					if (VolumeData.GetNodeAddressFromPosition(NodeAddress, TestPosition, MinLayerIndex))
					{
						const FNav3DNode& Node = VolumeData.GetNodeFromAddress(NodeAddress);
						
						// Check if node is navigable - nodes without children are free
						bool bIsNavigable = false;
						if (NodeAddress.LayerIndex == 0)
						{
							// For leaf nodes, check if the specific subnode is free
							const auto& LeafNodes = VolumeData.GetData().GetLeafNodes();
							if (LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
							{
								const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
								bIsNavigable = !LeafNode.IsSubNodeOccluded(NodeAddress.SubNodeIndex);
							}
						}
						else
						{
							// For non-leaf nodes, check if they don't have children (meaning they're free)
							bIsNavigable = !Node.HasChildren();
						}
						
						if (bIsNavigable)
						{
							FVector NodePosition = VolumeData.GetNodePositionFromAddress(NodeAddress, false);
							UE_LOG(LogNav3D, VeryVerbose, TEXT("FindNearestFreeVoxel: Found free voxel at %s (layer %d)"), 
							       *NodePosition.ToString(), NodeAddress.LayerIndex);
							return FEndpointProjectionResult(true, NodePosition, NodeAddress, NodeAddress.LayerIndex);
						}
					}
					
					// Try layer fallback for this position
					const LayerIndex LayerCount = VolumeData.GetLayerCount();
					for (LayerIndex TestLayer = MinLayerIndex + 1; TestLayer < LayerCount; ++TestLayer)
					{
						if (VolumeData.GetNodeAddressFromPosition(NodeAddress, TestPosition, TestLayer))
						{
							const FNav3DNode& Node = VolumeData.GetNodeFromAddress(NodeAddress);
							
							// Check if node is navigable - nodes without children are free
							bool bIsNavigable = false;
							if (NodeAddress.LayerIndex == 0)
							{
								// For leaf nodes, check if the specific subnode is free
								const auto& LeafNodes = VolumeData.GetData().GetLeafNodes();
								if (LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
								{
									const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
									bIsNavigable = !LeafNode.IsSubNodeOccluded(NodeAddress.SubNodeIndex);
								}
							}
							else
							{
								// For non-leaf nodes, check if they don't have children (meaning they're free)
								bIsNavigable = !Node.HasChildren();
							}
							
							if (bIsNavigable)
							{
								FVector NodePosition = VolumeData.GetNodePositionFromAddress(NodeAddress, false);
								UE_LOG(LogNav3D, VeryVerbose, TEXT("FindNearestFreeVoxel: Found free voxel at %s (layer %d, fallback)"), 
								       *NodePosition.ToString(), TestLayer);
								return FEndpointProjectionResult(true, NodePosition, NodeAddress, TestLayer);
							}
						}
					}
				}
			}
		}
		
		CurrentRadius += RadiusIncrement;
	}

	FString FailureReason = FString::Printf(TEXT("No free voxel found within radius %.2f after %d iterations"), SearchRadius, MaxSearchIterations);
	UE_LOG(LogNav3D, Warning, TEXT("FindNearestFreeVoxel: %s"), *FailureReason);
	return FEndpointProjectionResult(false, SearchCenter, FNav3DNodeAddress(), MinLayerIndex, FailureReason);
}