#include "Raycasting/Nav3DMultiChunkConeCaster.h"
#include "Nav3D.h"
#include "Nav3DData.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DVolumeNavigationData.h"
#include "Nav3DDataChunk.h"

int32 UNav3DMultiChunkConeCaster::FindOccludedVoxelsInCone(
	const ANav3DData* Nav3DData,
	const FNav3DConeCastParams& Params,
	TArray<FNav3DOccludedVoxel>& OutOccludedVoxels)
{
	if (!Nav3DData)
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindOccludedVoxelsInCone: No Nav3DData provided"));
		return 0;
	}

	OutOccludedVoxels.Reset();

	// Build chunk segments that intersect the cone
	TArray<FChunkConeSegment> Segments;
	if (!BuildChunkSegments(Nav3DData, Params, Segments))
	{
		return 0;
	}

	UE_LOG(LogNav3D, VeryVerbose, TEXT("FindOccludedVoxelsInCone: %d chunks, Target=Layer%d"), Segments.Num(), Params.TargetLayer);

	// Traverse octree in each chunk segment
	for (const FChunkConeSegment& Segment : Segments)
	{
		if (OutOccludedVoxels.Num() >= Params.MaxResults)
		{
			break;
		}

		TraverseConeInChunk(Segment, Params, OutOccludedVoxels);
	}

	UE_LOG(LogNav3D, VeryVerbose, TEXT("FindOccludedVoxelsInCone: %d occluded voxels found"), OutOccludedVoxels.Num());
	return OutOccludedVoxels.Num();
}

bool UNav3DMultiChunkConeCaster::BuildChunkSegments(
	const ANav3DData* Nav3DData,
	const FNav3DConeCastParams& Params,
	TArray<FChunkConeSegment>& OutSegments)
{
	OutSegments.Reset();

	TArray<ANav3DDataChunkActor*> AllChunks = Nav3DData->GetAllChunkActors();
	const FVector ConeEnd = Params.Origin + Params.Direction * Params.MaxDistance;

	for (ANav3DDataChunkActor* ChunkActor : AllChunks)
	{
		if (!ChunkActor) continue;

		// Check if chunk AABB intersects the cone
		if (AABBIntersectsCone(
			ChunkActor->DataChunkActorBounds,
			Params.Origin,
			Params.Direction,
			Params.ConeAngleDeg,
			Params.MaxDistance))
		{
			FChunkConeSegment Segment;
			Segment.ChunkActor = ChunkActor;
			Segment.SegmentStart = Params.Origin;
			Segment.SegmentEnd = ConeEnd;
			OutSegments.Add(Segment);
		}
	}

	// Sort by distance from origin (process closer chunks first)
	OutSegments.Sort([&Params](const FChunkConeSegment& A, const FChunkConeSegment& B)
	{
		const float DistA = FVector::DistSquared(Params.Origin, A.ChunkActor->DataChunkActorBounds.GetCenter());
		const float DistB = FVector::DistSquared(Params.Origin, B.ChunkActor->DataChunkActorBounds.GetCenter());
		return DistA < DistB;
	});

	return OutSegments.Num() > 0;
}

void UNav3DMultiChunkConeCaster::TraverseConeInChunk(
	const FChunkConeSegment& Segment,
	const FNav3DConeCastParams& Params,
	TArray<FNav3DOccludedVoxel>& OutOccludedVoxels)
{
	if (!Segment.ChunkActor || Segment.ChunkActor->Nav3DChunks.Num() == 0)
	{
		return;
	}

	const FNav3DVolumeNavigationData* VolumeData = Segment.ChunkActor->Nav3DChunks[0]->GetVolumeNavigationData();
	if (!VolumeData)
	{
		return;
	}

	// Start traversal from highest layer (coarsest) and recurse down to target layer
	const FNav3DData& Nav3DData = VolumeData->GetData();
	const LayerIndex TopLayer = Nav3DData.GetLayerCount() - 1;
	
	if (Params.TargetLayer > TopLayer)
	{
		UE_LOG(LogNav3D, Warning, TEXT("TraverseConeInChunk: Invalid target layer %d (max: %d)"), Params.TargetLayer, TopLayer);
		return;
	}

	// Get root layer (highest layer index = coarsest voxels)
	const FNav3DLayer& RootLayer = Nav3DData.GetLayer(TopLayer);
	const TArray<FNav3DNode>& RootNodes = RootLayer.GetNodes();


	// Traverse from root nodes, will recurse down to target layer
	for (int32 i = 0; i < RootNodes.Num(); ++i)
	{
		if (OutOccludedVoxels.Num() >= Params.MaxResults)
		{
			break;
		}

		FNav3DNodeAddress NodeAddress(TopLayer, i, 0);
		TraverseNodeRecursive(VolumeData, NodeAddress, Params, OutOccludedVoxels);
	}
}

void UNav3DMultiChunkConeCaster::TraverseNodeRecursive(
	const FNav3DVolumeNavigationData* VolumeData,
	const FNav3DNodeAddress& NodeAddress,
	const FNav3DConeCastParams& Params,
	TArray<FNav3DOccludedVoxel>& OutOccludedVoxels)
{
	if (OutOccludedVoxels.Num() >= Params.MaxResults)
	{
		return;
	}

	const FNav3DNode& Node = VolumeData->GetNodeFromAddress(NodeAddress);
	const FVector NodePosition = VolumeData->GetNodePositionFromAddress(NodeAddress, false);
	const float NodeExtent = VolumeData->GetNodeExtentFromNodeAddress(NodeAddress);

	// Build AABB for this node
	const FBox NodeBounds(NodePosition - FVector(NodeExtent), NodePosition + FVector(NodeExtent));

	// Early rejection: check if node AABB intersects cone
	if (!AABBIntersectsCone(NodeBounds, Params.Origin, Params.Direction, Params.ConeAngleDeg, Params.MaxDistance))
	{
		return;
	}

	// If we've reached our target layer, this node is occluded (SVO only stores occluded voxels)
	if (NodeAddress.LayerIndex == Params.TargetLayer)
	{
		OutOccludedVoxels.Emplace(NodePosition, NodeExtent * 2.0f, NodeAddress);
		return;
	}

	// If we haven't reached target layer yet but node has no children, use this coarser voxel
	if (!Node.HasChildren())
	{
		OutOccludedVoxels.Emplace(NodePosition, NodeExtent * 2.0f, NodeAddress);
		return;
	}

	// Node has children - recurse to children in layer below (lower index)
	
	for (int32 ChildOffset = 0; ChildOffset < 8; ++ChildOffset)
	{
		if (OutOccludedVoxels.Num() >= Params.MaxResults)
		{
			break;
		}

		FNav3DNodeAddress ChildAddress = Node.FirstChild;
		ChildAddress.NodeIndex += ChildOffset;

		TraverseNodeRecursive(VolumeData, ChildAddress, Params, OutOccludedVoxels);
	}
}

bool UNav3DMultiChunkConeCaster::AABBIntersectsCone(
	const FBox& Box,
	const FVector& ConeOrigin,
	const FVector& ConeDirection,
	float ConeAngleDeg,
	float MaxDistance)
{
	// Quick rejection test: check if box is behind cone origin
	const FVector BoxCenter = Box.GetCenter();
	const FVector ToBox = BoxCenter - ConeOrigin;
	
	if (FVector::DotProduct(ToBox, ConeDirection) < 0.0f)
	{
		// Box is behind cone - but check if cone origin is inside box
		if (Box.IsInside(ConeOrigin))
		{
			return true;
		}
		return false;
	}

	// Distance test: check if box is beyond max distance
	const FVector ClosestPoint = Box.GetClosestPointTo(ConeOrigin);
	const float DistSq = FVector::DistSquared(ConeOrigin, ClosestPoint);
	
	if (DistSq > MaxDistance * MaxDistance)
	{
		return false;
	}

	// Use sphere test on box for cone intersection
	// Use box diagonal as conservative sphere radius
	const FVector BoxExtent = Box.GetExtent();
	const float SphereRadius = BoxExtent.Size();
	const float CosHalfAngle = FMath::Cos(FMath::DegreesToRadians(ConeAngleDeg * 0.5f));

	return SphereIntersectsCone(BoxCenter, SphereRadius, ConeOrigin, ConeDirection, CosHalfAngle, MaxDistance);
}

bool UNav3DMultiChunkConeCaster::SphereIntersectsCone(
	const FVector& SphereCenter,
	float SphereRadius,
	const FVector& ConeOrigin,
	const FVector& ConeDirection,
	float CosHalfAngle,
	float MaxDistance)
{
	const FVector ToSphere = SphereCenter - ConeOrigin;
	const float DistAlongAxis = FVector::DotProduct(ToSphere, ConeDirection);

	// Check if sphere is behind cone origin (accounting for radius)
	if (DistAlongAxis < -SphereRadius)
	{
		return false;
	}

	// Check if sphere is beyond max distance (accounting for radius)
	if (DistAlongAxis > MaxDistance + SphereRadius)
	{
		return false;
	}

	// Project sphere center onto cone axis
	const FVector ClosestOnAxis = ConeOrigin + ConeDirection * DistAlongAxis;
	const float PerpendicularDist = FVector::Distance(SphereCenter, ClosestOnAxis);

	// At this distance along the axis, what's the cone radius?
	const float ConeRadiusAtDist = FMath::Max(0.0f, DistAlongAxis) * FMath::Tan(FMath::Acos(CosHalfAngle));

	// Check if sphere intersects cone surface (accounting for sphere radius)
	return PerpendicularDist <= ConeRadiusAtDist + SphereRadius;
}

