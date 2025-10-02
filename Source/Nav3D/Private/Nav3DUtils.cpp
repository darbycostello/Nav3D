#include "Nav3DUtils.h"
#include "Nav3D.h"
#include "GameFramework/NavMovementComponent.h"
#include "ThirdParty/libmorton/morton.h"
#include "Nav3DDataChunk.h"
#include "Nav3DDataChunkActor.h"
#include "NavigationSystem.h"
#include "Nav3DData.h"
#include "Nav3DSettings.h"

MortonCode FNav3DUtils::GetMortonCodeFromVector(const FVector& Vector)
{
	return morton3D_64_encode(Vector.X, Vector.Y, Vector.Z);
}

MortonCode FNav3DUtils::GetMortonCodeFromIntVector(const FIntVector& IntVector)
{
	// Ensure coordinates are non-negative and within valid range for Morton encoding
	const uint32 X = FMath::Clamp(IntVector.X, 0, 1023); // 10 bits max per coordinate
	const uint32 Y = FMath::Clamp(IntVector.Y, 0, 1023);
	const uint32 Z = FMath::Clamp(IntVector.Z, 0, 1023);
    
	MortonCode Result = 0;
    
	// Interleave bits: Z|Y|X for each bit position
	for (int32 i = 0; i < 10; ++i) // 10 bits per coordinate = 30 bits total
	{
		const uint32 BitPos = i * 3;
		Result |= ((X >> i) & 1) << (BitPos + 0);  // X bits at positions 0, 3, 6, 9...
		Result |= ((Y >> i) & 1) << (BitPos + 1);  // Y bits at positions 1, 4, 7, 10...
		Result |= ((Z >> i) & 1) << (BitPos + 2);  // Z bits at positions 2, 5, 8, 11...
	}
    
	return Result;
}

FVector FNav3DUtils::GetVectorFromMortonCode(const MortonCode MortonCode)
{
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);

	return FVector(X, Y, Z);
}

FIntVector FNav3DUtils::GetIntVectorFromMortonCode(const MortonCode MortonCode)
{
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);
	return FIntVector(X, Y, Z);  // Direct integer conversion
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
	float TMin = -BIG_NUMBER;
	float TMax = BIG_NUMBER;

	for (int32 i = 0; i < 3; i++)
	{
		const float RayOrig = i == 0 ? RayOrigin.X : (i == 1 ? RayOrigin.Y : RayOrigin.Z);
		const float RayDirComponent = i == 0 ? RayDir.X : (i == 1 ? RayDir.Y : RayDir.Z);
		const float BoxMin = i == 0 ? Box.Min.X : (i == 1 ? Box.Min.Y : Box.Min.Z);
		const float BoxMax = i == 0 ? Box.Max.X : (i == 1 ? Box.Max.Y : Box.Max.Z);

		if (FMath::Abs(RayDirComponent) < SMALL_NUMBER)
		{
			// Ray parallel to this axis
			if (RayOrig < BoxMin || RayOrig > BoxMax)
			{
				return false;
			}
		}
		else
		{
			const float InvRayDir = 1.0f / RayDirComponent;
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

			if (TMin > TMax)
			{
				return false;
			}
		}
	}

	// Check if intersection is within ray segment
	if (TMax < 0.0f || TMin > RayLength)
	{
		return false;
	}

	OutTMin = FMath::Max(TMin, 0.0f);
	OutTMax = FMath::Min(TMax, RayLength);
	
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

float FNav3DUtils::GetMaxSearchDistance()
{
	if (const UNav3DSettings* Nav3DSettings = GetDefault<UNav3DSettings>())
	{
		return Nav3DSettings->MaxVolumePartitionSize; 
	}
	return 10000.0f;
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

				if (bOnMinXFace || bOnMaxXFace || bOnMinYFace || bOnMaxYFace || bOnMinZFace || bOnMaxZFace)
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

FLinearColor FNav3DUtils::GetChunkColorByIndex(const int32 ChunkIndex)
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

float FNav3DUtils::GetDefaultVoxelSize(const ANav3DData* NavData)
{
	if (NavData)
	{
		const float VoxelExtent = NavData->GetVoxelExtent();
		if (VoxelExtent > 0.0f)
		{
			return VoxelExtent;
		}
	}
    
	// Fallback to default if no NavData or invalid extent
	static constexpr float DefaultVoxelSize = 100.0f; // cm
	return DefaultVoxelSize;
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
	const FVector LocalPos = LocalVolume.GetLeafNodePositionFromMortonCode(Connection.Local);
	const FVector RemotePos = RemoteVolume.GetLeafNodePositionFromMortonCode(Connection.Remote);

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
	const float Distance = FVector::Dist(LocalPos, RemotePos);
	const float VoxelSize = LocalVolume.GetData().GetLeafNodes().GetLeafNodeSize();
	const float MaxReasonableDistance = VoxelSize * 2.0f; // Allow up to 2 voxel lengths for adjacent volumes
	
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

bool FNav3DUtils::CheckVoxelFaceAdjacency(
    const FNav3DEdgeVoxel& VoxelA, 
    const FNav3DEdgeVoxel& VoxelB,
    const FNav3DVolumeNavigationData* VolumeA,
    const FNav3DVolumeNavigationData* VolumeB,
    uint8 FaceA, 
    uint8 FaceB,
    float AdjacencyClearance)
{
    // Get world positions
    FVector PosA, PosB;
    if (VoxelA.LayerIndex == 0)
    {
        PosA = VolumeA->GetLeafNodePositionFromMortonCode(VoxelA.Morton);
    }
    else
    {
        PosA = VolumeA->GetNodePositionFromLayerAndMortonCode(VoxelA.LayerIndex, VoxelA.Morton);
    }
    
    if (VoxelB.LayerIndex == 0)
    {
        PosB = VolumeB->GetLeafNodePositionFromMortonCode(VoxelB.Morton);
    }
    else
    {
        PosB = VolumeB->GetNodePositionFromLayerAndMortonCode(VoxelB.LayerIndex, VoxelB.Morton);
    }

    // Get voxel extents for each layer
    float VoxelExtentA, VoxelExtentB;
    if (VoxelA.LayerIndex == 0)
    {
        VoxelExtentA = VolumeA->GetData().GetLeafNodes().GetLeafNodeExtent();
    }
    else
    {
        VoxelExtentA = VolumeA->GetData().GetLayer(VoxelA.LayerIndex).GetNodeExtent();
    }
    
    if (VoxelB.LayerIndex == 0)
    {
        VoxelExtentB = VolumeB->GetData().GetLeafNodes().GetLeafNodeExtent();
    }
    else
    {
        VoxelExtentB = VolumeB->GetData().GetLayer(VoxelB.LayerIndex).GetNodeExtent();
    }

    // Project voxel centers to their respective face boundaries
    FVector FacePointA = PosA;
    FVector FacePointB = PosB;
    
    // Project VoxelA to its face boundary
    if (FaceA == 1)      FacePointA.X += VoxelExtentA;  // MaxX face
    else if (FaceA == 2) FacePointA.X -= VoxelExtentA;  // MinX face
    else if (FaceA == 4) FacePointA.Y += VoxelExtentA;  // MaxY face
    else if (FaceA == 8) FacePointA.Y -= VoxelExtentA;  // MinY face
    else if (FaceA == 16) FacePointA.Z += VoxelExtentA; // MaxZ face
    else if (FaceA == 32) FacePointA.Z -= VoxelExtentA; // MinZ face
    
    // Project VoxelB to its face boundary
    if (FaceB == 1)      FacePointB.X += VoxelExtentB;  // MaxX face
    else if (FaceB == 2) FacePointB.X -= VoxelExtentB;  // MinX face
    else if (FaceB == 4) FacePointB.Y += VoxelExtentB;  // MaxY face
    else if (FaceB == 8) FacePointB.Y -= VoxelExtentB;  // MinY face
    else if (FaceB == 16) FacePointB.Z += VoxelExtentB; // MaxZ face
    else if (FaceB == 32) FacePointB.Z -= VoxelExtentB; // MinZ face

    // Calculate face-to-face distance along the shared axis
    float FaceDistance = 0.0f;

    // Determine shared axis and calculate overlap
    if ((FaceA == 1 && FaceB == 2) || (FaceA == 2 && FaceB == 1)) // X-axis shared
    {
        FaceDistance = FMath::Abs(FacePointA.X - FacePointB.X);
        
        // Check Y-Z plane overlap
        float AMinY = PosA.Y - VoxelExtentA, AMaxY = PosA.Y + VoxelExtentA;
        float AMinZ = PosA.Z - VoxelExtentA, AMaxZ = PosA.Z + VoxelExtentA;
        float BMinY = PosB.Y - VoxelExtentB, BMaxY = PosB.Y + VoxelExtentB;
        float BMinZ = PosB.Z - VoxelExtentB, BMaxZ = PosB.Z + VoxelExtentB;
        
        // Check if there's overlap in both Y and Z dimensions
        bool YOverlap = (AMaxY >= BMinY) && (AMinY <= BMaxY);
        bool ZOverlap = (AMaxZ >= BMinZ) && (AMinZ <= BMaxZ);
        
        if (!YOverlap || !ZOverlap)
        {
            return false; // No face overlap
        }
    }
    else if ((FaceA == 4 && FaceB == 8) || (FaceA == 8 && FaceB == 4)) // Y-axis shared
    {
        FaceDistance = FMath::Abs(FacePointA.Y - FacePointB.Y);
        
        // Check X-Z plane overlap
        float AMinX = PosA.X - VoxelExtentA, AMaxX = PosA.X + VoxelExtentA;
        float AMinZ = PosA.Z - VoxelExtentA, AMaxZ = PosA.Z + VoxelExtentA;
        float BMinX = PosB.X - VoxelExtentB, BMaxX = PosB.X + VoxelExtentB;
        float BMinZ = PosB.Z - VoxelExtentB, BMaxZ = PosB.Z + VoxelExtentB;
        
        bool XOverlap = (AMaxX >= BMinX) && (AMinX <= BMaxX);
        bool ZOverlap = (AMaxZ >= BMinZ) && (AMinZ <= BMaxZ);
        
        if (!XOverlap || !ZOverlap)
        {
            return false;
        }
    }
    else if ((FaceA == 16 && FaceB == 32) || (FaceA == 32 && FaceB == 16)) // Z-axis shared
    {
        FaceDistance = FMath::Abs(FacePointA.Z - FacePointB.Z);
        
        // Check X-Y plane overlap
        float AMinX = PosA.X - VoxelExtentA, AMaxX = PosA.X + VoxelExtentA;
        float AMinY = PosA.Y - VoxelExtentA, AMaxY = PosA.Y + VoxelExtentA;
        float BMinX = PosB.X - VoxelExtentB, BMaxX = PosB.X + VoxelExtentB;
        float BMinY = PosB.Y - VoxelExtentB, BMaxY = PosB.Y + VoxelExtentB;
        
        bool XOverlap = (AMaxX >= BMinX) && (AMinX <= BMaxX);
        bool YOverlap = (AMaxY >= BMinY) && (AMinY <= BMaxY);
        
        if (!XOverlap || !YOverlap)
        {
            return false;
        }
    }
    else
    {
        // Non-adjacent faces, shouldn't happen for properly shared faces
        return false;
    }

    // Check if face distance is within adjacency clearance
    bool bAdjacent = FaceDistance <= AdjacencyClearance;
    
    UE_LOG(LogNav3D, VeryVerbose, TEXT("Face adjacency check: VoxelA(Extent=%.1f) <-> VoxelB(Extent=%.1f), FaceDistance=%.1f, Clearance=%.1f, Adjacent=%s"),
           VoxelExtentA, VoxelExtentB, FaceDistance, AdjacencyClearance, bAdjacent ? TEXT("YES") : TEXT("NO"));
    
    return bAdjacent;
}

FSharedConstNavQueryFilter FNav3DUtils::GetNav3DQueryFilter(
	const ANav3DData* Nav3DData,
	const TSubclassOf<UNavigationQueryFilter>& NavigationQueryFilter,
	const UObject* Querier)
{
	FSharedConstNavQueryFilter QueryFilter;
    
	// In editor mode, use default
	if (GIsEditor && !GIsPlayInEditorWorld)
	{
		QueryFilter = Nav3DData->GetDefaultQueryFilter();
	}
	else
	{
		if (NavigationQueryFilter)
		{
			QueryFilter = UNavigationQueryFilter::GetQueryFilter(*Nav3DData, Querier, NavigationQueryFilter);
		}
		else
		{
			QueryFilter = Nav3DData->GetDefaultQueryFilter();
		}
	}
    
	return QueryFilter;
}


bool FNav3DUtils::IsNodeFreeSpace(
	const FNav3DVolumeNavigationData& VolumeData,
	const FNav3DNodeAddress& NodeAddress)
{
	const FNav3DNode& Node = VolumeData.GetNodeFromAddress(NodeAddress);
	
	if (NodeAddress.LayerIndex == 0)
	{
		// For leaf nodes, check if the specific subnode is free
		const auto& LeafNodes = VolumeData.GetData().GetLeafNodes();
		if (LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
		{
			const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
			return !LeafNode.IsSubNodeOccluded(NodeAddress.SubNodeIndex);
		}
		return false;
	}
	else
	{
		// For non-leaf nodes, free space means no children (not subdivided due to obstacles)
		return !Node.HasChildren();
	}
}
