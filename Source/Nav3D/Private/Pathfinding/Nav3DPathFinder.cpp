#include "Pathfinding/Nav3DPathFinder.h"
#include "Nav3D.h"
#include "Nav3DData.h"
#include "Nav3DSettings.h"
#include "Nav3DUtils.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Nav3DQueryFilter.h"
#include "Raycasting/Nav3DRaycaster.h"

UNav3DPathFindingSearch* FNav3DPathFinder::GetPathFindingSearch(
	const FSharedConstNavQueryFilter& NavQueryFilter)
{
	// Get filter implementation
	const FNav3DQueryFilter* QueryFilterImplementation = nullptr;
    
	if (NavQueryFilter.IsValid())
	{
		QueryFilterImplementation = static_cast<const FNav3DQueryFilter*>(
			NavQueryFilter->GetImplementation());
	}
    
	// If we don't have a valid filter implementation, use defaults from settings
	if (QueryFilterImplementation == nullptr)
	{
		const UNav3DSettings* Settings = UNav3DSettings::Get();
		return Settings->DefaultPathFinder->GetDefaultObject<UNav3DPathFindingSearch>();
	}
    
	const auto& QueryFilterSettings = QueryFilterImplementation->QueryFilterSettings;
    
	if (!ensureAlwaysMsgf(QueryFilterSettings.PathFinder != nullptr,
					  TEXT("The PathFinder is not valid")))
	{
		return nullptr;
	}
    
	return QueryFilterSettings.PathFinder;
}

ENavigationQueryResult::Type FNav3DPathFinder::GetPathInternal(
	FNav3DPath& NavigationPath,
	const ANav3DData& NavData,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const FNavAgentProperties& NavAgentProperties,
	const FSharedConstNavQueryFilter& NavQueryFilter)
{
	if (const auto* VolumeNavigationData = NavData.
		GetVolumeNavigationDataContainingPoints({StartLocation, EndLocation}))
	{
		if (const auto* Raycaster = NewObject<UNav3DRaycaster>())
		{
			if (Raycaster->Trace(*VolumeNavigationData, StartLocation, EndLocation))
			{
				auto& PathPoints = NavigationPath.GetPathPoints();
				PathPoints.Emplace(StartLocation);
				PathPoints.Emplace(EndLocation);
				NavigationPath.MarkReady();

				return ENavigationQueryResult::Success;
			}
		}

		// Use the query filter from default settings if none provided here.
		FSharedConstNavQueryFilter QueryFilterCopy = NavQueryFilter;
		if (!QueryFilterCopy.IsValid())
		{
			// Create a filter with global settings
			UNav3DQueryFilter* TempFilter = NewObject<UNav3DQueryFilter>();
			TempFilter->SetQueryFilterSettings(UNav3DSettings::Get()->GetDefaultQueryFilterSettings());
			QueryFilterCopy = TempFilter->GetQueryFilter(NavData, nullptr);
		}
		
		if (const auto* PathFinder = GetPathFindingSearch(QueryFilterCopy))
		{
			const LayerIndex MinLayerIndex =
				VolumeNavigationData->GetMinLayerIndexForAgentSize(NavAgentProperties.AgentRadius);
			UE_LOG(LogNav3D, Verbose, TEXT("Minimum layer index for nav agent radius %f: %d"),
			       NavAgentProperties.AgentRadius, MinLayerIndex);

			if (const auto Params = FNav3DPathFindingParameters::Initialize(
					*VolumeNavigationData,
					StartLocation,
					EndLocation,
					*QueryFilterCopy,
					MinLayerIndex);
				Params.IsSet())
			{
				return PathFinder->GetPath(NavigationPath, Params.GetValue());
			}
		}
	}

	return ENavigationQueryResult::Fail;
}

ENavigationQueryResult::Type FNav3DPathFinder::GetPath(
	FNav3DPath& NavigationPath,
	const ANav3DData& NavData,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const FNavAgentProperties& NavAgentProperties,
	const FSharedConstNavQueryFilter& NavQueryFilter)
{
	// First sanitize the path into segments
	FSanitizedPath SanitizedPath = SanitizePath(NavData, StartLocation, EndLocation);

	if (SanitizedPath.Segments.Num() == 0)
	{
		return ENavigationQueryResult::Fail;
	}

	// For each segment, find path
	for (const auto& Segment : SanitizedPath.Segments)
	{
		// Handle direct movement segments (outside nav volumes)
		if (!Segment.NavVolume)
		{
			auto& PathPoints = NavigationPath.GetPathPoints();
			auto& PathCosts = NavigationPath.GetPathPointCosts();

			// Add direct line segment
			if (PathPoints.Num() == 0 || !PathPoints.Last().Location.Equals(Segment.StartPoint))
			{
				PathPoints.Add(FNavPathPoint(Segment.StartPoint));
				PathCosts.Add(0.0f);
			}
			PathPoints.Add(FNavPathPoint(Segment.EndPoint));
			PathCosts.Add(FVector::Dist(Segment.StartPoint, Segment.EndPoint));
			continue;
		}

		FNav3DPath SegmentPath;
		const auto Result = GetPathInternal(
			SegmentPath,
			NavData,
			Segment.StartPoint,
			Segment.EndPoint,
			NavAgentProperties,
			NavQueryFilter);

		if (Result != ENavigationQueryResult::Success)
		{
			return Result; // Return any error or fail result immediately
		}

		// Append points from this segment (skipping duplicate points)
		const auto& SegmentPoints = SegmentPath.GetPathPoints();
		if (SegmentPoints.Num() > 0)
		{
			auto& PathPoints = NavigationPath.GetPathPoints();
			auto& PathCosts = NavigationPath.GetPathPointCosts();

			// Skip first point if not first segment (to avoid duplicates)
			const int32 StartIdx = (PathPoints.Num() > 0) ? 1 : 0;

			for (int32 i = StartIdx; i < SegmentPoints.Num(); i++)
			{
				PathPoints.Add(SegmentPoints[i]);
				PathCosts.Add(SegmentPath.GetPathPointCosts()[i]);
			}
		}
	}

	// Mark path ready
	NavigationPath.MarkReady();

	return ENavigationQueryResult::Success;
}

TSharedPtr<FNav3DPathStepper> FNav3DPathFinder::GetDebugPathStepper(
	FNav3DPathFinderDebugData& DebugData,
	const ANav3DData& NavigationData,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const FNavAgentProperties& NavAgentProperties,
	const FSharedConstNavQueryFilter& NavQueryFilter)
{
	// Use the query filter from default settings if none provided here.
	FSharedConstNavQueryFilter QueryFilterCopy = NavQueryFilter;
	if (!QueryFilterCopy.IsValid())
	{
		UNav3DQueryFilter* TempFilter = NewObject<UNav3DQueryFilter>();
		TempFilter->SetQueryFilterSettings(UNav3DSettings::Get()->GetDefaultQueryFilterSettings());
		QueryFilterCopy = TempFilter->GetQueryFilter(NavigationData, nullptr);
	}
	
	if (const auto* PathFinder = GetPathFindingSearch(QueryFilterCopy))
	{
		if (const auto* VolumeNavigationData = NavigationData.GetVolumeNavigationDataContainingPoints(
			{StartLocation, EndLocation}))
		{
			const LayerIndex MinLayerIndex =
				VolumeNavigationData->GetMinLayerIndexForAgentSize(NavAgentProperties.AgentRadius);
			UE_LOG(LogNav3D, Verbose, TEXT("Minimum layer index for nav agent radius %f: %d"),
			       NavAgentProperties.AgentRadius, MinLayerIndex);

			if (const auto Params = FNav3DPathFindingParameters::Initialize(
					*VolumeNavigationData,
					StartLocation,
					EndLocation,
					*NavQueryFilter,
					MinLayerIndex);
				Params.IsSet())
			{
				return PathFinder->GetDebugPathStepper(DebugData, Params.GetValue());
			}
		}
	}

	return nullptr;
}

FSanitizedPath FNav3DPathFinder::SanitizePath(
	const ANav3DData& NavData,
	const FVector& OriginalStart,
	const FVector& OriginalEnd)
{
	FSanitizedPath Result;
	const auto& AllVolumes = NavData.GetVolumeNavigationData();

	// Find volumes containing start and end
	const FNav3DVolumeNavigationData* StartVolume = nullptr;
	const FNav3DVolumeNavigationData* EndVolume = nullptr;

	// First find if any volume contains both points
	for (const auto& Volume : AllVolumes)
	{
		const FBox& Bounds = Volume.GetVolumeBounds();
		if (Bounds.IsInside(OriginalStart) && Bounds.IsInside(OriginalEnd))
		{
			// Both points in same volume - simplest case
			Result.Segments.Add(FPathSegment(OriginalStart, OriginalEnd, &Volume));
			return Result;
		}

		if (Bounds.IsInside(OriginalStart))
		{
			StartVolume = &Volume;
		}
		if (Bounds.IsInside(OriginalEnd))
		{
			EndVolume = &Volume;
		}
	}

	FVector Start = OriginalStart;
	FVector End = OriginalEnd;

	// If start point isn't in any volume, try to find intersection with bounds
	if (!StartVolume)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Start point %s not in any nav volume, finding intersection point"),
		       *Start.ToString());

		// Get direction vector from start to end
		const FVector Dir = (End - Start).GetSafeNormal();
		float BestEnterTime = FLT_MAX;

		// Find first intersection with any volume along the path to target
		for (const auto& Volume : AllVolumes)
		{
			const FBox& Bounds = Volume.GetVolumeBounds();
			float EnterTime, ExitTime;

			if (FNav3DUtils::RayBoxIntersection(Bounds, Start, Dir, FVector::Dist(Start, End), EnterTime, ExitTime))
			{
				if (EnterTime >= 0 && EnterTime < BestEnterTime)
				{
					BestEnterTime = EnterTime;
					StartVolume = &Volume;
				}
			}
		}

		if (!StartVolume)
		{
			UE_LOG(LogNav3D, Warning, TEXT("Could not find valid nav volume intersection from start point"));
			return Result;
		}

		// Add segment from original start to intersection point
		const FVector IntersectionPoint = Start + Dir * BestEnterTime;
		Result.Segments.Add(FPathSegment(Start, IntersectionPoint, nullptr)); // null volume for non-pathfinding segment
		Start = IntersectionPoint; // Update start for remaining path segments
	}

	// If end point isn't in any volume, find closest point in bounds
	if (!EndVolume)
	{
		UE_LOG(LogNav3D, Warning, TEXT("End point %s not in any nav volume, finding closest valid point"),
		       *End.ToString());

		float ClosestDistSq = FLT_MAX;
		FVector ClosestPoint = End;

		for (const auto& Volume : AllVolumes)
		{
			const FBox& Bounds = Volume.GetVolumeBounds();
			const FVector ClosestInVolume = Bounds.GetClosestPointTo(End);
			const float DistSq = FVector::DistSquared(End, ClosestInVolume);

			if (DistSq < ClosestDistSq)
			{
				ClosestDistSq = DistSq;
				ClosestPoint = ClosestInVolume;
				EndVolume = &Volume;
			}
		}

		End = ClosestPoint;
		Result.bAdjustedEndPoint = true;
	}

	// Now handle case where start and end are in different volumes
	if (StartVolume != EndVolume)
	{
		// Get direction vector from start to end
		const FVector Dir = (End - Start).GetSafeNormal();

		// Find intersections with volume bounds along this line
		TArray<TPair<float, const FNav3DVolumeNavigationData*>> Intersections;

		for (const auto& Volume : AllVolumes)
		{
			const FBox& Bounds = Volume.GetVolumeBounds();
			float EnterTime, ExitTime;

			if (FNav3DUtils::RayBoxIntersection(Bounds, Start, Dir, FVector::Dist(Start, End), EnterTime, ExitTime))
			{
				if (EnterTime >= 0)
				{
					Intersections.Add(TPair<float, const FNav3DVolumeNavigationData*>(EnterTime, &Volume));
				}
				if (ExitTime >= 0)
				{
					Intersections.Add(TPair<float, const FNav3DVolumeNavigationData*>(ExitTime, &Volume));
				}
			}
		}

		// Sort intersections by distance
		Intersections.Sort([](const auto& A, const auto& B) { return A.Key < B.Key; });

		// Create path segments
		FVector CurrentPoint = Start;
		const FNav3DVolumeNavigationData* CurrentVolume = StartVolume;

		for (const auto& Intersection : Intersections)
		{
			const FVector IntersectionPoint = Start + Dir * Intersection.Key;

			// Add segment if it's meaningful (not too short)
			if (!CurrentPoint.Equals(IntersectionPoint, 1.0f))
			{
				Result.Segments.Add(FPathSegment(CurrentPoint, IntersectionPoint, CurrentVolume));
			}

			CurrentPoint = IntersectionPoint;
			CurrentVolume = Intersection.Value;
		}

		// Add final segment to end point
		if (!CurrentPoint.Equals(End, 1.0f))
		{
			Result.Segments.Add(FPathSegment(CurrentPoint, End, EndVolume));
		}
	}
	else
	{
		// Start and end in same volume
		Result.Segments.Add(FPathSegment(Start, End, StartVolume));
	}

	return Result;
}

void FNav3DPathFinder::BuildPath(FNav3DPath& Path, const FNav3DPathFindingParameters& Params,
                                 const TArray<FNav3DPathFinderNodeAddress>& NodeAddresses,
                                 const bool AddEndLocation)
{
	auto& PathPoints = Path.GetPathPoints();
	auto& PathPointCosts = Path.GetPathPointCosts();

	const auto PathPointsSize = NodeAddresses.Num() + 1;

	ensureAlways(NodeAddresses[0].NodeAddress == Params.StartNodeAddress);

	const auto& BoundsData = Params.VolumeNavigationData;

	PathPoints.Reset(PathPointsSize);
	PathPointCosts.Reset(PathPointsSize);

	PathPoints.Emplace(Params.StartLocation);
	PathPointCosts.Add(0.0f);

	for (auto Index = 1; Index < NodeAddresses.Num() - 1; Index++)
	{
		const auto AddressWithCost = NodeAddresses[Index];
		PathPoints.Emplace(BoundsData.GetNodePositionFromAddress(
			AddressWithCost.NodeAddress, true));
		PathPointCosts.Add(AddressWithCost.Cost);
	}

	if (AddEndLocation)
	{
		PathPoints.Emplace(Params.EndLocation);
		PathPointCosts.Add(NodeAddresses.Last().Cost);
	}
}
