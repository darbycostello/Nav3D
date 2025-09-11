#include "Pathfinding/Nav3DPathFinder.h"
#include "Nav3D.h"
#include "Nav3DData.h"
#include "Nav3DSettings.h"
#include "Nav3DUtils.h"
#include "Nav3DWorldSubsystem.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Nav3DQueryFilter.h"
#include "Pathfinding/Nav3DCrossVolumePathfinder.h"
#include "Pathfinding/Nav3DCrossVolumeDebugStepper.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Nav3DDataChunkActor.h"
#include "EngineUtils.h"
#include "Pathfinding/Nav3DPathSmoothing.h"

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
	const FNav3DVolumeNavigationData& VolumeNavData,
	const ANav3DData& NavData,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const FNavAgentProperties& NavAgentProperties,
	const FSharedConstNavQueryFilter& NavQueryFilter)
{
	UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Starting single-volume pathfinding from %s to %s"), 
	       *StartLocation.ToString(), *EndLocation.ToString());

	// Check if both points are within the volume bounds
	// For cross-volume pathfinding, we only need the start point to be in this volume
	if (!VolumeNavData.GetVolumeBounds().IsInside(StartLocation))
	{
		UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Start point is outside volume bounds"));
		return ENavigationQueryResult::Fail;
	}

	// For cross-volume pathfinding, the end point might be in a different volume
	if (!VolumeNavData.GetVolumeBounds().IsInside(EndLocation))
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: End point is outside volume bounds - this is cross-volume pathfinding"));
		// Don't fail here - this is expected for cross-volume pathfinding
	}
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Both points are within volume bounds"));
	}

	// Remove direct LOS short-circuit for intra-volume segments to ensure voxel-valid paths
	// if (const auto* Raycaster = NewObject<UNav3DRaycaster>())
	// {
	// 	UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Attempting direct raycast trace"));
	// 	if (!Raycaster->Trace(*VolumeNavigationData, StartLocation, EndLocation))
	// 	{
	// 		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Direct raycast successful - using straight line path"));
	// 		auto& PathPoints = NavigationPath.GetPathPoints();
	// 		PathPoints.Emplace(StartLocation);
	// 		PathPoints.Emplace(EndLocation);
	// 		NavigationPath.MarkReady();
	//
	// 		return ENavigationQueryResult::Success;
	// 	}
	// 	else
	// 	{
	// 		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Direct raycast failed - proceeding with pathfinding"));
	// 	}
	// }
	// else
	// {
	// 	UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Failed to create raycaster"));
	// }

	// Use the query filter from default settings if none provided here.
	FSharedConstNavQueryFilter QueryFilterCopy = NavQueryFilter;
	if (!QueryFilterCopy.IsValid())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: No query filter provided, creating default filter"));
		// Create a filter with global settings
		UNav3DQueryFilter* TempFilter = NewObject<UNav3DQueryFilter>();
		TempFilter->SetQueryFilterSettings(UNav3DSettings::Get()->GetDefaultQueryFilterSettings());
		// Use the instance method to get the query filter
		QueryFilterCopy = TempFilter->GetQueryFilter(NavData, nullptr);
	}

	// Calculate minimum layer index based on agent radius
	const float AgentRadius = NavAgentProperties.AgentRadius;
	const int32 MinLayerIndex = FMath::Max(0, FMath::FloorToInt(FMath::Log2(AgentRadius / VolumeNavData.GetSettings().VoxelExtent)));
	
	UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Minimum layer index for nav agent radius %f: %d"), 
	       AgentRadius, MinLayerIndex);

	// Initialize pathfinding parameters
	if (const auto Params = FNav3DPathFindingParameters::Initialize(
			VolumeNavData,
			StartLocation,
			EndLocation,
			*QueryFilterCopy,
			MinLayerIndex);
		Params.IsSet())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Pathfinding parameters initialized successfully"));
		
		// Get the pathfinder from the query filter
		UNav3DPathFindingSearch* PathFinder = GetPathFindingSearch(QueryFilterCopy);
		if (!PathFinder)
		{
			UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Failed to get pathfinder"));
			return ENavigationQueryResult::Fail;
		}
		
		// Perform pathfinding
		const ENavigationQueryResult::Type Result = PathFinder->GetPath(NavigationPath, Params.GetValue());
		if (Result != ENavigationQueryResult::Success)
		{
			UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Pathfinding failed with result: %d"), (int32)Result);
			return Result;
		}
		
		NavigationPath.MarkReady();
		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Pathfinding successful, generated %d path points"), 
		       NavigationPath.GetPathPoints().Num());
		return ENavigationQueryResult::Success;
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Failed to initialize pathfinding parameters"));
		return ENavigationQueryResult::Fail;
	}
}

ENavigationQueryResult::Type FNav3DPathFinder::GetPathInternal(
	FNav3DPath& NavigationPath,
	const ANav3DData& NavData,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const FNavAgentProperties& NavAgentProperties,
	const FSharedConstNavQueryFilter& NavQueryFilter)
{
	UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Starting single-volume pathfinding from %s to %s"), 
	       *StartLocation.ToString(), *EndLocation.ToString());

	if (const auto* VolumeNavigationData = NavData.
		GetVolumeNavigationDataContainingPoints({StartLocation, EndLocation}))
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Found single volume containing both points"));

		// Remove direct LOS short-circuit for intra-volume segments to ensure voxel-valid paths
		// if (const auto* Raycaster = NewObject<UNav3DRaycaster>())
		// {
		// 	UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Attempting direct raycast trace"));
		// 	if (!Raycaster->Trace(*VolumeNavigationData, StartLocation, EndLocation))
		// 	{
		// 		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Direct raycast successful - using straight line path"));
		// 		auto& PathPoints = NavigationPath.GetPathPoints();
		// 		PathPoints.Emplace(StartLocation);
		// 		PathPoints.Emplace(EndLocation);
		// 		NavigationPath.MarkReady();
		//
		// 		return ENavigationQueryResult::Success;
		// 	}
		// 	else
		// 	{
		// 		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Direct raycast failed - proceeding with pathfinding"));
		// 	}
		// }
		// else
		// {
		// 	UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Failed to create raycaster"));
		// }

		// Use the query filter from default settings if none provided here.
		FSharedConstNavQueryFilter QueryFilterCopy = NavQueryFilter;
		if (!QueryFilterCopy.IsValid())
		{
			UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: No valid query filter provided, creating default filter"));
			// Create a filter with global settings
			UNav3DQueryFilter* TempFilter = NewObject<UNav3DQueryFilter>();
			TempFilter->SetQueryFilterSettings(UNav3DSettings::Get()->GetDefaultQueryFilterSettings());
			QueryFilterCopy = TempFilter->GetQueryFilter(NavData, nullptr);
		}
		
		if (const auto* PathFinder = GetPathFindingSearch(QueryFilterCopy))
		{
			const LayerIndex MinLayerIndex =
				VolumeNavigationData->GetMinLayerIndexForAgentSize(NavAgentProperties.AgentRadius);
			UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Minimum layer index for nav agent radius %f: %d"),
			       NavAgentProperties.AgentRadius, MinLayerIndex);

			FNav3DNodeAddress TmpA, TmpB;
			const bool AOk = VolumeNavigationData->GetNodeAddressFromPosition(TmpA, StartLocation, MinLayerIndex);
			const bool BOk = VolumeNavigationData->GetNodeAddressFromPosition(TmpB, EndLocation, MinLayerIndex);
			UE_LOG(LogNav3D, Verbose, TEXT("InitProbe: AOk=%d, BOk=%d, MinLayer=%d, A=%s, B=%s"), 
				   AOk, BOk, MinLayerIndex, *StartLocation.ToString(), *EndLocation.ToString());
			
			if (AOk)
			{
				UE_LOG(LogNav3D, VeryVerbose, TEXT("InitProbe: Start resolved to layer %d, node %d, subnode %d"), 
					TmpA.LayerIndex, TmpA.NodeIndex, TmpA.SubNodeIndex);
			}
			if (BOk)
			{
				UE_LOG(LogNav3D, VeryVerbose, TEXT("InitProbe: End resolved to layer %d, node %d, subnode %d"), 
					TmpB.LayerIndex, TmpB.NodeIndex, TmpB.SubNodeIndex);
			}

			if (const auto Params = FNav3DPathFindingParameters::Initialize(
					*VolumeNavigationData,
					StartLocation,
					EndLocation,
					*QueryFilterCopy,
					MinLayerIndex);
				Params.IsSet())
			{
				UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Pathfinding parameters initialized successfully, calling PathFinder->GetPath"));
				const ENavigationQueryResult::Type Result = PathFinder->GetPath(NavigationPath, Params.GetValue());
				UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: PathFinder->GetPath returned: %d"), (int32)Result);
				return Result;
			}
			else
			{
				UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Failed to initialize pathfinding parameters"));

				// Fallback 1: Retry at higher layers to avoid missing/occluded leaf subnodes
				const int32 LayerCount = VolumeNavigationData->GetLayerCount();
				const LayerIndex MaxTryLayer = FMath::Min<LayerIndex>(LayerCount > 0 ? LayerCount - 1 : 0, MinLayerIndex + 2);
				for (LayerIndex TestLayer = MinLayerIndex + 1; TestLayer <= MaxTryLayer; ++TestLayer)
				{
					UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Retrying Initialize at higher layer %d"), TestLayer);
					if (const auto ParamsHi = FNav3DPathFindingParameters::Initialize(
							*VolumeNavigationData,
							StartLocation,
							EndLocation,
							*QueryFilterCopy,
							TestLayer);
						ParamsHi.IsSet())
					{
						UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Higher-layer parameters initialized successfully at layer %d"), TestLayer);
						const ENavigationQueryResult::Type Result = PathFinder->GetPath(NavigationPath, ParamsHi.GetValue());
						UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: PathFinder->GetPath (higher layer %d) returned: %d"), TestLayer, (int32)Result);
						return Result;
					}
				}

				// Fallback 2: Nudge endpoints inward toward volume center and retry
				const FVector VolCenter = VolumeNavigationData->GetNavigationBounds().GetCenter();
				const float VoxelSize = VolumeNavigationData->GetData().GetLayer(MinLayerIndex).GetNodeSize();
				const FVector NudgeDirA = (VolCenter - StartLocation).GetSafeNormal();
				const FVector NudgeDirB = (VolCenter - EndLocation).GetSafeNormal();
				const float NudgeDist = FMath::Max(50.0f, VoxelSize * 0.25f);
				const FVector NudgedA = StartLocation + NudgeDirA * NudgeDist;
				const FVector NudgedB = EndLocation + NudgeDirB * NudgeDist;

				UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Retrying Initialize with nudged points A=%s, B=%s (nudge=%.1f)"), *NudgedA.ToString(), *NudgedB.ToString(), NudgeDist);

				if (const auto ParamsNudge = FNav3DPathFindingParameters::Initialize(
						*VolumeNavigationData,
						NudgedA,
						NudgedB,
						*QueryFilterCopy,
						MinLayerIndex);
					ParamsNudge.IsSet())
				{
					const ENavigationQueryResult::Type Result = PathFinder->GetPath(NavigationPath, ParamsNudge.GetValue());
					UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: PathFinder->GetPath (nudged) returned: %d"), (int32)Result);
					return Result;
				}

				// Fallback 3: Combine nudging with higher layer
				for (LayerIndex TestLayer = MinLayerIndex + 1; TestLayer <= MaxTryLayer; ++TestLayer)
				{
					UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: Retrying Initialize (nudged) at higher layer %d"), TestLayer);
					if (const auto ParamsNudgeHi = FNav3DPathFindingParameters::Initialize(
							*VolumeNavigationData,
							NudgedA,
							NudgedB,
							*QueryFilterCopy,
							TestLayer);
						ParamsNudgeHi.IsSet())
					{
						const ENavigationQueryResult::Type Result = PathFinder->GetPath(NavigationPath, ParamsNudgeHi.GetValue());
						UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: PathFinder->GetPath (nudged, layer %d) returned: %d"), TestLayer, (int32)Result);
						return Result;
					}
				}
			}
		}
		else
		{
			UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Failed to get pathfinding search"));
		}
	}
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPathInternal: No single volume found containing both points - this should not happen in GetPathInternal"));
	}

	UE_LOG(LogNav3D, Warning, TEXT("GetPathInternal: Single-volume pathfinding failed"));
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
	UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Starting pathfinding from %s to %s"), 
	       *StartLocation.ToString(), *EndLocation.ToString());

	// Early line-of-sight check before any volume lookups
	if (const auto* VolumeNavigationData = NavData.GetVolumeNavigationDataContainingPoints({StartLocation, EndLocation}))
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Both points in same volume - attempting early line-of-sight check"));
        
		if (const auto* Raycaster = NewObject<UNav3DRaycaster>())
		{
			FNav3DRaycastHit Hit;
			const bool bDirectPathBlocked = Raycaster->Trace(*VolumeNavigationData, StartLocation, EndLocation, Hit);
            
			if (!bDirectPathBlocked)
			{
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Early line-of-sight check passed - returning direct path"));
                
				// Create simple two-point path
				auto& PathPoints = NavigationPath.GetPathPoints();
				PathPoints.Emplace(StartLocation);
				PathPoints.Emplace(EndLocation);
				NavigationPath.MarkReady();
                
				return ENavigationQueryResult::Success;
			}
			UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Early line-of-sight check FAILED at %s - proceeding with full pathfinding"),
			       *Hit.ImpactPoint.ToString());
		}
		else
		{
			UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to create raycaster for early line-of-sight check"));
		}
	}
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Points in different volumes - skipping early line-of-sight check"));
	}

	// Attempt world-partition hierarchical path first
	if (const UWorld* World = NavData.GetWorld())
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Attempting cross-volume pathfinding via FNav3DCrossVolumePathfinder"));
		
		// Start debug session for cross-volume pathfinding
		FString DebugSessionId = FNav3DCrossVolumeDebugStepper::GenerateSessionId();
		FNav3DCrossVolumeDebugStepper::FDebugSession* DebugSession = 
			FNav3DCrossVolumeDebugStepper::StartDebugSession(DebugSessionId, StartLocation, EndLocation);
		
		TArray<ANav3DDataChunkActor*> ActorPath;
		TArray<FNav3DActorPortal> Portals;
		if (FNav3DCrossVolumePathfinder::FindActorPath(World, StartLocation, EndLocation, ActorPath, Portals) && ActorPath.Num() > 0)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Cross-volume pathfinder found %d actors and %d portals"), 
			       ActorPath.Num(), Portals.Num());
			
			// Add debug step for actor path finding
			FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("ActorPathFound"), 
				FString::Printf(TEXT("Found %d actors and %d portals"), ActorPath.Num(), Portals.Num()),
				StartLocation, true);
			
			if (ActorPath.Num() == 1)
			{
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Single actor found - delegating to GetPathInternal for single-volume pathfinding"));
				FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("SingleActorDelegation"), 
					TEXT("Single actor found, delegating to single-volume pathfinding"), StartLocation, true);
				FNav3DCrossVolumeDebugStepper::CompleteDebugSession(DebugSession, true, TEXT("Single actor pathfinding"));
				return GetPathInternal(NavigationPath, NavData, StartLocation, EndLocation, NavAgentProperties, NavQueryFilter);
			}

			UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Multi-actor cross-volume pathfinding - building path segments"));

			auto AppendIntraVolume = [&](const FVector& A, const FVector& B, ANav3DDataChunkActor* VolumeActor = nullptr) -> bool
			{
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Building intra-volume segment from %s to %s"), 
				       *A.ToString(), *B.ToString());
				
				// If we have a specific volume actor, use it for pathfinding
				if (VolumeActor && VolumeActor->Nav3DChunks.Num() > 0)
				{
					UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Using specific volume actor %s for intra-volume pathfinding"), 
					       *VolumeActor->GetName());
					
					// Get the volume navigation data directly from the chunk
					const FNav3DVolumeNavigationData* VolumeNavData = VolumeActor->Nav3DChunks[0]->GetVolumeNavigationData();
					if (!VolumeNavData)
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: No volume navigation data found in chunk actor %s"), 
						       *VolumeActor->GetName());
						return false;
					}
					
					// Get the ANav3DData from the world
					const ANav3DData* NavDataActor = nullptr;
					if (const UWorld* VolumeActorWorld = VolumeActor->GetWorld())
					{
						for (const TActorIterator<ANav3DData> ActorItr(VolumeActorWorld); ActorItr;)
						{
							NavDataActor = *ActorItr;
							break; // Take the first one
						}
					}
					
					if (!NavDataActor)
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: No Nav3DData found in world"));
						return false;
					}
					
					FNav3DPath SegmentPath;
					const ENavigationQueryResult::Type R = GetPathInternal(SegmentPath, *VolumeNavData, *NavDataActor, A, B, NavAgentProperties, NavQueryFilter);
					if (R != ENavigationQueryResult::Success) 
					{ 
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Intra-volume segment failed with result: %d"), (int32)R);
						return false; 
					}
					UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Intra-volume segment successful, appending %d points"), 
					       SegmentPath.GetPathPoints().Num());
					const auto& SegmentPoints = SegmentPath.GetPathPoints();
					auto& OutPts = NavigationPath.GetPathPoints();
					auto& OutCosts = NavigationPath.GetPathPointCosts();
					
					if (SegmentPoints.Num() == 0)
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Segment path has 0 points, skipping"));
						return false;
					}
					
					if (OutPts.Num() == 0)
					{
						OutPts.Add(SegmentPoints[0]);
						OutCosts.Add(0.0f);
					}
					
					// Only add additional points if there are more than 1
					const auto& SegmentCosts = SegmentPath.GetPathPointCosts();
					for (int32 i = 1; i < SegmentPoints.Num(); ++i)
					{
						OutPts.Add(SegmentPoints[i]);
						// Safety check: ensure costs array has enough elements
						if (i < SegmentCosts.Num())
						{
							OutCosts.Add(SegmentCosts[i]);
						}
						else
						{
							UE_LOG(LogNav3D, Warning, TEXT("GetPath: Costs array size mismatch - points: %d, costs: %d, using default cost"), 
							       SegmentPoints.Num(), SegmentCosts.Num());
							OutCosts.Add(0.0f);
						}
					}
					return true;
				}
				else
				{
					// Fallback to original behavior
					FNav3DPath SegmentPath;
					const ENavigationQueryResult::Type R = GetPathInternal(SegmentPath, NavData, A, B, NavAgentProperties, NavQueryFilter);
					if (R != ENavigationQueryResult::Success) 
					{ 
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Intra-volume segment failed with result: %d"), (int32)R);
						return false; 
					}
					UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Intra-volume segment successful, appending %d points"), 
					       SegmentPath.GetPathPoints().Num());
					const auto& SegmentPoints = SegmentPath.GetPathPoints();
					auto& OutPts = NavigationPath.GetPathPoints();
					auto& OutCosts = NavigationPath.GetPathPointCosts();
					
					if (SegmentPoints.Num() == 0)
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Segment path has 0 points, skipping"));
						return false;
					}
					
					if (OutPts.Num() == 0)
					{
						OutPts.Add(SegmentPoints[0]);
						OutCosts.Add(0.0f);
					}
					
					// Only add additional points if there are more than 1
					const auto& SegmentCosts = SegmentPath.GetPathPointCosts();
					for (int32 i = 1; i < SegmentPoints.Num(); ++i)
					{
						OutPts.Add(SegmentPoints[i]);
						// Safety check: ensure costs array has enough elements
						if (i < SegmentCosts.Num())
						{
							OutCosts.Add(SegmentCosts[i]);
						}
						else
						{
							UE_LOG(LogNav3D, Warning, TEXT("GetPath: Costs array size mismatch - points: %d, costs: %d, using default cost"), 
							       SegmentPoints.Num(), SegmentCosts.Num());
							OutCosts.Add(0.0f);
						}
					}
					return true;
				}
			};

			auto ResolvePortalPos = [&](ANav3DDataChunkActor* Actor, const FNav3DVoxelConnection& Conn, bool bLocal) -> TOptional<FVector>
			{
				if (!Actor) 
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: ResolvePortalPos failed - null actor"));
					return TOptional<FVector>();
				}
				
				const int32 VolIdx = bLocal ? Conn.LocalVolumeIndex : Conn.RemoteVolumeIndex;
				const uint64 Morton = bLocal ? Conn.Local : Conn.Remote;
				
				UE_LOG(LogNav3D, VeryVerbose, TEXT("GetPath: Resolving portal position (Local=%s, VolIdx=%d, Morton=%llu)"), 
				       bLocal ? TEXT("true") : TEXT("false"), VolIdx, Morton);
				
				for (UNav3DDataChunk* C : Actor->Nav3DChunks)
				{
					if (!C) continue;
					if (C->NavigationData.IsValidIndex(VolIdx))
					{
						const FNav3DVolumeNavigationData& Vol = C->NavigationData[VolIdx];
						
						// Use the new endpoint projection utility to ensure we get a free voxel
						FNav3DUtils::FEndpointProjectionResult ProjectionResult = 
							FNav3DUtils::ProjectPortalToFreeVoxel(Vol, Conn, bLocal, NavAgentProperties, 0);
						
						if (ProjectionResult.bSuccess)
						{
							UE_LOG(LogNav3D, VeryVerbose, TEXT("GetPath: Portal projection successful at layer %d"), ProjectionResult.ResolvedLayer);
							return ProjectionResult.ProjectedPosition;
						}
						else
						{
							UE_LOG(LogNav3D, Warning, TEXT("GetPath: Portal projection failed: %s"), *ProjectionResult.FailureReason);
							// Fallback to original method for backward compatibility
							return Vol.GetLeafNodePositionFromMortonCode(Morton);
						}
					}
				}
				
				UE_LOG(LogNav3D, Warning, TEXT("GetPath: ResolvePortalPos failed - no valid volume found for index %d"), VolIdx);
				return TOptional<FVector>();
			};

			// Project start and end locations to free voxels in their respective volumes
			FVector ProjectedStartLocation = StartLocation;
			FVector ProjectedEndLocation = EndLocation;
			
			// Project start location to free voxel in first actor's volume
			if (ActorPath.Num() > 0 && ActorPath[0] && ActorPath[0]->Nav3DChunks.Num() > 0)
			{
				if (const FNav3DVolumeNavigationData* StartVolumeData = ActorPath[0]->Nav3DChunks[0]->GetVolumeNavigationData())
				{
					FNav3DUtils::FEndpointProjectionResult StartProjection = 
						FNav3DUtils::ProjectPointToFreeVoxel(*StartVolumeData, StartLocation, NavAgentProperties, 0);
					
					if (StartProjection.bSuccess)
					{
						ProjectedStartLocation = StartProjection.ProjectedPosition;
						UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Projected start location to free voxel: %s"), 
						       *ProjectedStartLocation.ToString());
						FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("StartLocationProjection"), 
							TEXT("Successfully projected start location to free voxel"), ProjectedStartLocation, 
							StartProjection.NodeAddress, true);
					}
					else
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to project start location: %s"), *StartProjection.FailureReason);
						FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("StartLocationProjection"), 
							TEXT("Failed to project start location to free voxel"), StartLocation, false, StartProjection.FailureReason);
					}
				}
			}
			
			// Project end location to free voxel in last actor's volume
			if (ActorPath.Num() > 0 && ActorPath.Last() && ActorPath.Last()->Nav3DChunks.Num() > 0)
			{
				if (const FNav3DVolumeNavigationData* EndVolumeData = ActorPath.Last()->Nav3DChunks[0]->GetVolumeNavigationData())
				{
					FNav3DUtils::FEndpointProjectionResult EndProjection = 
						FNav3DUtils::ProjectPointToFreeVoxel(*EndVolumeData, EndLocation, NavAgentProperties, 0);
					
					if (EndProjection.bSuccess)
					{
						ProjectedEndLocation = EndProjection.ProjectedPosition;
						UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Projected end location to free voxel: %s"), 
						       *ProjectedEndLocation.ToString());
						FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("EndLocationProjection"), 
							TEXT("Successfully projected end location to free voxel"), ProjectedEndLocation, 
							EndProjection.NodeAddress, true);
					}
					else
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to project end location: %s"), *EndProjection.FailureReason);
						FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("EndLocationProjection"), 
							TEXT("Failed to project end location to free voxel"), EndLocation, false, EndProjection.FailureReason);
					}
				}
			}

			// Start to first portal
			if (Portals.Num() > 0)
			{
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Building start-to-first-portal segment"));
				const FNav3DActorPortal& First = Portals[0];
				const TOptional<FVector> FirstLocal = ResolvePortalPos(First.From.Get(), First.Connection, true);
				if (!FirstLocal.IsSet())
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to resolve first portal local position"));
					FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("FirstPortalResolution"), 
						TEXT("Failed to resolve first portal local position"), ProjectedStartLocation, false, 
						TEXT("Portal position resolution failed"));
					FNav3DCrossVolumeDebugStepper::CompleteDebugSession(DebugSession, false, TEXT("First portal resolution failed"));
					return ENavigationQueryResult::Fail;
				}
				
				FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("FirstPortalResolution"), 
					TEXT("Successfully resolved first portal local position"), FirstLocal.GetValue(), true);
				
				if (!AppendIntraVolume(ProjectedStartLocation, FirstLocal.GetValue(), First.From.Get()))
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: Start-to-first-portal segment failed"));
					FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("StartToFirstPortalSegment"), 
						TEXT("Failed to build start-to-first-portal path segment"), FirstLocal.GetValue(), false, 
						TEXT("Intra-volume pathfinding failed"));
					FNav3DCrossVolumeDebugStepper::CompleteDebugSession(DebugSession, false, TEXT("Start-to-first-portal segment failed"));
					return ENavigationQueryResult::Fail;
				}
				
				FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("StartToFirstPortalSegment"), 
					TEXT("Successfully built start-to-first-portal path segment"), FirstLocal.GetValue(), true);
			}

			// Middle actor hops
			for (int32 i = 0; i < Portals.Num() - 1; ++i)
			{
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Building middle segment %d/%d"), i+1, Portals.Num()-1);
				const FNav3DActorPortal& Cur = Portals[i];
				const FNav3DActorPortal& Next = Portals[i+1];
				const TOptional<FVector> CurRemote = ResolvePortalPos(Cur.To.Get(), Cur.Connection, false);
				const TOptional<FVector> NextLocal = ResolvePortalPos(Next.From.Get(), Next.Connection, true);
				if (!CurRemote.IsSet())
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to resolve current portal remote position"));
					return ENavigationQueryResult::Fail;
				}
				if (!NextLocal.IsSet())
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to resolve next portal local position"));
					return ENavigationQueryResult::Fail;
				}
				
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Middle segment %d - CurRemote: %s, NextLocal: %s"), 
				       i+1, *CurRemote.GetValue().ToString(), *NextLocal.GetValue().ToString());
				
				// Check if the positions are identical (which would cause 0-point path)
				if (FVector::Dist(CurRemote.GetValue(), NextLocal.GetValue()) < 1.0f)
				{
					UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Middle segment %d has identical start/end positions, skipping"), i+1);
					continue; // Skip this segment instead of failing
				}
				
				if (!AppendIntraVolume(CurRemote.GetValue(), NextLocal.GetValue(), Next.From.Get()))
				{
					UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Middle segment %d failed"), i+1);
					return ENavigationQueryResult::Fail;
				}
			}

			// Last portal to end
			if (Portals.Num() > 0)
			{
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Building last-portal-to-end segment"));
				const FNav3DActorPortal& Last = Portals.Last();
				const TOptional<FVector> LastRemote = ResolvePortalPos(Last.To.Get(), Last.Connection, false);
				if (!LastRemote.IsSet())
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to resolve last portal remote position"));
					return ENavigationQueryResult::Fail;
				}
				if (!AppendIntraVolume(LastRemote.GetValue(), ProjectedEndLocation, Last.To.Get()))
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: Last-portal-to-end segment failed"));
					return ENavigationQueryResult::Fail;
				}
			}
			else if (ActorPath.Num() > 1)
			{
				// No portals but multiple actors - build path through volume boundaries
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: No portals but %d actors - building path through volume boundaries"), ActorPath.Num());
				
				// For cross-volume without portals, we need to find boundary points between volumes
				// and do pathfinding within each volume
				
				ANav3DDataChunkActor* StartActor = ActorPath[0];
				ANav3DDataChunkActor* EndActor = ActorPath.Last();
				
				// Find closest points on the bounding boxes
				FVector StartBoundary = StartActor->DataChunkActorBounds.GetClosestPointTo(
					EndActor->DataChunkActorBounds.GetCenter());
				FVector EndBoundary = EndActor->DataChunkActorBounds.
				                                GetClosestPointTo(StartActor->DataChunkActorBounds.GetCenter());
				
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Start boundary: %s, End boundary: %s"), 
				       *StartBoundary.ToString(), *EndBoundary.ToString());
				
				// Project boundary points to navigable positions within their respective volumes
				FVector ProjectedStartBoundary = StartBoundary;
				FVector ProjectedEndBoundary = EndBoundary;
				
				if (StartActor->Nav3DChunks.Num() > 0 && StartActor->Nav3DChunks[0]->GetVolumeNavigationData())
				{
					const FNav3DVolumeNavigationData* StartVolumeData = StartActor->Nav3DChunks[0]->GetVolumeNavigationData();
					FNav3DUtils::FEndpointProjectionResult StartProjection = 
						FNav3DUtils::ProjectBoundaryToNavigable(*StartVolumeData, StartBoundary, NavAgentProperties, 0);
					
					if (StartProjection.bSuccess)
					{
						ProjectedStartBoundary = StartProjection.ProjectedPosition;
						UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Projected start boundary to navigable position: %s"), 
						       *ProjectedStartBoundary.ToString());
					}
					else
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to project start boundary: %s"), *StartProjection.FailureReason);
					}
				}
				
				if (EndActor->Nav3DChunks.Num() > 0 && EndActor->Nav3DChunks[0]->GetVolumeNavigationData())
				{
					const FNav3DVolumeNavigationData* EndVolumeData = EndActor->Nav3DChunks[0]->GetVolumeNavigationData();
					FNav3DUtils::FEndpointProjectionResult EndProjection = 
						FNav3DUtils::ProjectBoundaryToNavigable(*EndVolumeData, EndBoundary, NavAgentProperties, 0);
					
					if (EndProjection.bSuccess)
					{
						ProjectedEndBoundary = EndProjection.ProjectedPosition;
						UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Projected end boundary to navigable position: %s"), 
						       *ProjectedEndBoundary.ToString());
					}
					else
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Failed to project end boundary: %s"), *EndProjection.FailureReason);
					}
				}
				
				// Try to do pathfinding within the start volume from start location to projected boundary
				if (StartActor->Nav3DChunks[0]->GetVolumeNavigationData())
				{
					UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Attempting pathfinding in start volume from %s to %s"), 
					       *ProjectedStartLocation.ToString(), *ProjectedStartBoundary.ToString());

					if (FNav3DPath StartSegment;
						GetPathInternal(StartSegment, NavData, ProjectedStartLocation, ProjectedStartBoundary, NavAgentProperties, NavQueryFilter) == ENavigationQueryResult::Success)
					{
						UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Start volume pathfinding successful, %d points"), StartSegment.GetPathPoints().Num());
						
						// Append start segment
						const auto& StartPoints = StartSegment.GetPathPoints();
						const auto& StartCosts = StartSegment.GetPathPointCosts();
						auto& PathPoints = NavigationPath.GetPathPoints();
						auto& PathCosts = NavigationPath.GetPathPointCosts();
						
						for (int32 i = 0; i < StartPoints.Num(); ++i)
						{
							PathPoints.Add(StartPoints[i]);
							PathCosts.Add(StartCosts[i]);
						}
					}
					else
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: Start volume pathfinding failed, using direct path"));
						// Fallback to direct path
						auto& PathPoints = NavigationPath.GetPathPoints();
						auto& PathCosts = NavigationPath.GetPathPointCosts();
						
						if (PathPoints.Num() == 0)
						{
							PathPoints.Add(FNavPathPoint(StartLocation));
							PathCosts.Add(0.0f);
						}
						PathPoints.Add(FNavPathPoint(EndLocation));
						PathCosts.Add(FVector::Dist(StartLocation, EndLocation));
					}
				}
				else
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: No start volume data, using direct path"));
					// Fallback to direct path
					auto& PathPoints = NavigationPath.GetPathPoints();
					auto& PathCosts = NavigationPath.GetPathPointCosts();
					
					if (PathPoints.Num() == 0)
					{
						PathPoints.Add(FNavPathPoint(StartLocation));
						PathCosts.Add(0.0f);
					}
					PathPoints.Add(FNavPathPoint(EndLocation));
					PathCosts.Add(FVector::Dist(StartLocation, EndLocation));
				}
				
				// Add direct path segment from projected end boundary to projected end location
				if (EndActor->Nav3DChunks.Num() > 0 && EndActor->Nav3DChunks[0]->GetVolumeNavigationData())
				{
					UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Attempting pathfinding in end volume from %s to %s"), 
					       *ProjectedEndBoundary.ToString(), *ProjectedEndLocation.ToString());

					if (FNav3DPath EndSegment;
						GetPathInternal(EndSegment, NavData, ProjectedEndBoundary, ProjectedEndLocation, NavAgentProperties, NavQueryFilter) == ENavigationQueryResult::Success)
					{
						UE_LOG(LogNav3D, Verbose, TEXT("GetPath: End volume pathfinding successful, %d points"), EndSegment.GetPathPoints().Num());
						
						// Append end segment (skip first point to avoid duplication)
						const auto& EndPoints = EndSegment.GetPathPoints();
						const auto& EndCosts = EndSegment.GetPathPointCosts();
						auto& PathPoints = NavigationPath.GetPathPoints();
						auto& PathCosts = NavigationPath.GetPathPointCosts();
						
						for (int32 i = 1; i < EndPoints.Num(); ++i) // Skip first point
						{
							PathPoints.Add(EndPoints[i]);
							PathCosts.Add(EndCosts[i]);
						}
					}
					else
					{
						UE_LOG(LogNav3D, Warning, TEXT("GetPath: End volume pathfinding failed, adding direct connection"));
						// Fallback to direct connection
						auto& PathPoints = NavigationPath.GetPathPoints();
						auto& PathCosts = NavigationPath.GetPathPointCosts();
						
						PathPoints.Add(FNavPathPoint(ProjectedEndLocation));
						PathCosts.Add(FVector::Dist(ProjectedEndBoundary, ProjectedEndLocation));
					}
				}
				else
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: No end volume data, adding direct connection"));
					// Fallback to direct connection
					auto& PathPoints = NavigationPath.GetPathPoints();
					auto& PathCosts = NavigationPath.GetPathPointCosts();
					
					PathPoints.Add(FNavPathPoint(ProjectedEndLocation));
					PathCosts.Add(FVector::Dist(ProjectedEndBoundary, ProjectedEndLocation));
				}
				
				UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Added cross-volume path with %d points"), NavigationPath.GetPathPoints().Num());
			}

			UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Cross-volume pathfinding completed successfully with %d total path points"), 
			       NavigationPath.GetPathPoints().Num());
			
			FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("CrossVolumePathComplete"), 
				FString::Printf(TEXT("Cross-volume pathfinding completed with %d path points"), NavigationPath.GetPathPoints().Num()),
				EndLocation, true);
			FNav3DCrossVolumeDebugStepper::CompleteDebugSession(DebugSession, true, TEXT("Cross-volume pathfinding successful"));
			
			// Validate path points before marking ready
			ValidatePathPoints(NavigationPath, NavData, NavAgentProperties);
			
			// Add on-ramp (original start) and off-ramp (original end) if missing
			{
				auto& PathPoints = NavigationPath.GetPathPoints();
				auto& PathCosts = NavigationPath.GetPathPointCosts();
				if (PathPoints.Num() > 0)
				{
					if (!PathPoints[0].Location.Equals(StartLocation))
					{
						PathPoints.Insert(FNavPathPoint(StartLocation), 0);
						PathCosts.Insert(0.0f, 0);
					}
					if (!PathPoints.Last().Location.Equals(EndLocation))
					{
						const float TailCost = FVector::Dist(PathPoints.Last().Location, EndLocation);
						PathPoints.Add(FNavPathPoint(EndLocation));
						PathCosts.Add(TailCost);
					}
				}
			}
			
			NavigationPath.MarkReady();
			return ENavigationQueryResult::Success;
		}
		else
		{
			UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Cross-volume pathfinder failed or found no actors - falling back to sanitized path"));
			FNav3DCrossVolumeDebugStepper::AddDebugStep(DebugSession, TEXT("CrossVolumePathfinderFailed"), 
				TEXT("Cross-volume pathfinder failed or found no actors"), StartLocation, false, 
				TEXT("No valid actor path found"));
			FNav3DCrossVolumeDebugStepper::CompleteDebugSession(DebugSession, false, TEXT("Cross-volume pathfinder failed"));
		}
	}
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("GetPath: No world available - falling back to sanitized path"));
	}

	// First sanitize the path into segments
	UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Sanitizing path into segments"));
	FSanitizedPath SanitizedPath = SanitizePath(NavData, StartLocation, EndLocation);

	if (SanitizedPath.Segments.Num() == 0)
	{
		UE_LOG(LogNav3D, Warning, TEXT("GetPath: Sanitized path has no segments - pathfinding failed"));
		return ENavigationQueryResult::Fail;
	}

	UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Sanitized path has %d segments - processing each segment"), SanitizedPath.Segments.Num());

	// For each segment, find path
	for (int32 SegmentIndex = 0; SegmentIndex < SanitizedPath.Segments.Num(); ++SegmentIndex)
	{
		const auto& Segment = SanitizedPath.Segments[SegmentIndex];
		UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Processing segment %d/%d from %s to %s (has nav volume: %s)"), 
		       SegmentIndex + 1, SanitizedPath.Segments.Num(), 
		       *Segment.StartPoint.ToString(), *Segment.EndPoint.ToString(),
		       Segment.NavVolume ? TEXT("Yes") : TEXT("No"));

		// Handle direct movement segments (outside nav volumes)
		if (!Segment.NavVolume)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Segment %d is direct movement (no nav volume)"), SegmentIndex + 1);
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

		UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Segment %d has nav volume - calling GetPathInternal"), SegmentIndex + 1);
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
			UE_LOG(LogNav3D, Warning, TEXT("GetPath: Segment %d failed with result: %d"), SegmentIndex + 1, (int32)Result);
			return Result; // Return any error or fail result immediately
		}

		UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Segment %d successful, appending %d points"), 
		       SegmentIndex + 1, SegmentPath.GetPathPoints().Num());

		// Append points from this segment (skipping duplicate points)
		const auto& SegmentPoints = SegmentPath.GetPathPoints();
		if (SegmentPoints.Num() > 0)
		{
			auto& PathPoints = NavigationPath.GetPathPoints();
			auto& PathCosts = NavigationPath.GetPathPointCosts();

			// Skip first point if not first segment (to avoid duplicates)
			const int32 StartIdx = PathPoints.Num() > 0 ? 1 : 0;

			const auto& SegmentCosts = SegmentPath.GetPathPointCosts();
			for (int32 i = StartIdx; i < SegmentPoints.Num(); i++)
			{
				UE_LOG(LogNav3D, VeryVerbose, TEXT("GetPath: Adding segment point %d from segment %d at %s"), 
				       i, SegmentIndex + 1, *SegmentPoints[i].Location.ToString());
				PathPoints.Add(SegmentPoints[i]);
				// Safety check: ensure costs array has enough elements
				if (i < SegmentCosts.Num())
				{
					PathCosts.Add(SegmentCosts[i]);
				}
				else
				{
					UE_LOG(LogNav3D, Warning, TEXT("GetPath: Costs array size mismatch - points: %d, costs: %d, using default cost"), 
					       SegmentPoints.Num(), SegmentCosts.Num());
					PathCosts.Add(0.0f);
				}
			}
		}
	}

	// Apply path smoothing if enabled
	// DISABLED: Path smoothing creates interpolated points that don't correspond to voxel centers
	// This causes validation failures because smoothed points can't be resolved to node addresses
	/*
	if (NavQueryFilter.IsValid())
	{
		// Get query filter settings to check if smoothing is enabled
		// For now, we'll apply smoothing by default since it's enabled in the default settings
		UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Applying path smoothing with 10 subdivisions"));
		FNav3DPathSmoothing::SmoothPath(NavigationPath, 10);
	}
	*/
	UE_LOG(LogNav3D, Verbose, TEXT("GetPath: Path smoothing disabled - using raw voxel center path points"));

	// Validate path points before marking ready
	ValidatePathPoints(NavigationPath, NavData, NavAgentProperties);
	
	// Add on-ramp (original start) and off-ramp (original end) if missing
	{
		auto& PathPoints = NavigationPath.GetPathPoints();
		auto& PathCosts = NavigationPath.GetPathPointCosts();
		if (PathPoints.Num() > 0)
		{
			if (!PathPoints[0].Location.Equals(StartLocation))
			{
				PathPoints.Insert(FNavPathPoint(StartLocation), 0);
				PathCosts.Insert(0.0f, 0);
			}
			if (!PathPoints.Last().Location.Equals(EndLocation))
			{
				const float TailCost = FVector::Dist(PathPoints.Last().Location, EndLocation);
				PathPoints.Add(FNavPathPoint(EndLocation));
				PathCosts.Add(TailCost);
			}
		}
	}
	
	// Mark path ready
	UE_LOG(LogNav3D, Verbose, TEXT("GetPath: All segments processed successfully, marking path ready with %d total points"), 
	       NavigationPath.GetPathPoints().Num());
	NavigationPath.MarkReady();

	return ENavigationQueryResult::Success;
}

FSanitizedPath FNav3DPathFinder::SanitizePath(
	const ANav3DData& NavData,
	const FVector& OriginalStart,
	const FVector& OriginalEnd)
{
	UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Starting path sanitization from %s to %s"), 
	       *OriginalStart.ToString(), *OriginalEnd.ToString());

	FSanitizedPath Result;
	
	// Use spatial subsystem for efficient chunk actor queries
	const UNav3DWorldSubsystem* Subsystem = NavData.GetSubsystem();
	if (!Subsystem)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: No spatial subsystem available - using linear search fallback"));
		// Fallback to linear search through chunk actors
		ANav3DDataChunkActor* StartChunkActor = nullptr;
		ANav3DDataChunkActor* EndChunkActor = nullptr;
		
		for (ANav3DDataChunkActor* ChunkActor : NavData.GetAllChunkActors())
		{
			if (!ChunkActor) continue;
			
			if (ChunkActor->ContainsPoint(OriginalStart))
			{
				StartChunkActor = ChunkActor;
			}
			if (ChunkActor->ContainsPoint(OriginalEnd))
			{
				EndChunkActor = ChunkActor;
			}
		}
		
		UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Linear search found start chunk: %s, end chunk: %s"), 
		       StartChunkActor ? TEXT("Yes") : TEXT("No"), EndChunkActor ? TEXT("Yes") : TEXT("No"));
		
		// Handle the case where both points are in the same chunk
		if (StartChunkActor && EndChunkActor && StartChunkActor == EndChunkActor)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Both points in same chunk - creating single segment"));
			const FNav3DVolumeNavigationData* VolumeData = StartChunkActor->Nav3DChunks.Num() > 0 ? 
				StartChunkActor->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
			Result.Segments.Add(FPathSegment(OriginalStart, OriginalEnd, VolumeData));
			UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Created 1 segment with volume data: %s"), 
			       VolumeData ? TEXT("Yes") : TEXT("No"));
			return Result;
		}
		
		// For different chunks, create a simple segment
		if (StartChunkActor && EndChunkActor)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Points in different chunks - creating 3 segments"));
			const FNav3DVolumeNavigationData* StartVolumeData = StartChunkActor->Nav3DChunks.Num() > 0 ? 
				StartChunkActor->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
			const FNav3DVolumeNavigationData* EndVolumeData = EndChunkActor->Nav3DChunks.Num() > 0 ? 
				EndChunkActor->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
			
			// Create segments for each chunk
			Result.Segments.Add(FPathSegment(OriginalStart, StartChunkActor->DataChunkActorBounds.GetCenter(), StartVolumeData));
			Result.Segments.Add(FPathSegment(StartChunkActor->DataChunkActorBounds.GetCenter(), EndChunkActor->DataChunkActorBounds.GetCenter(), nullptr));
			Result.Segments.Add(FPathSegment(EndChunkActor->DataChunkActorBounds.GetCenter(), OriginalEnd, EndVolumeData));
			UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Created 3 segments (start->center, center->center, center->end)"));
		}
		else
		{
			UE_LOG(LogNav3D, Warning, TEXT("SanitizePath: Could not find chunk actors for one or both points"));
		}
		
		UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Linear search fallback completed with %d segments"), Result.Segments.Num());
		return Result;
	}
	
	// Use spatial subsystem for efficient queries
	UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Using spatial subsystem for efficient queries"));
	TArray<ANav3DDataChunkActor*> StartCandidates;
	TArray<ANav3DDataChunkActor*> EndCandidates;
	
	Subsystem->QueryActorsInBounds(FBox(OriginalStart, OriginalStart), StartCandidates);
	Subsystem->QueryActorsInBounds(FBox(OriginalEnd, OriginalEnd), EndCandidates);
	
	UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Spatial query found %d start candidates, %d end candidates"), 
	       StartCandidates.Num(), EndCandidates.Num());
	
	ANav3DDataChunkActor* StartChunkActor = StartCandidates.Num() > 0 ? StartCandidates[0] : nullptr;
	ANav3DDataChunkActor* EndChunkActor = EndCandidates.Num() > 0 ? EndCandidates[0] : nullptr;
	
	// Handle the case where both points are in the same chunk
	if (StartChunkActor && EndChunkActor && StartChunkActor == EndChunkActor)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Both points in same chunk (spatial subsystem) - creating single segment"));
		const FNav3DVolumeNavigationData* VolumeData = StartChunkActor->Nav3DChunks.Num() > 0 ? 
			StartChunkActor->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
		Result.Segments.Add(FPathSegment(OriginalStart, OriginalEnd, VolumeData));
		UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Created 1 segment with volume data: %s"), 
		       VolumeData ? TEXT("Yes") : TEXT("No"));
		return Result;
	}

	// For different chunks, use the world partition pathfinder approach
	if (StartChunkActor && EndChunkActor)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Points in different chunks - using cross-volume pathfinder"));
		// Use the existing world partition pathfinder to find the actor path
		TArray<ANav3DDataChunkActor*> ActorPath;
		TArray<FNav3DActorPortal> Portals;
		
		if (FNav3DCrossVolumePathfinder::FindActorPath(NavData.GetWorld(), OriginalStart, OriginalEnd, ActorPath, Portals))
		{
			UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Cross-volume pathfinder found %d actors and %d portals"), 
			       ActorPath.Num(), Portals.Num());
			// Create path segments based on the actor path
			FVector CurrentPoint = OriginalStart;
			
			for (int32 i = 0; i < ActorPath.Num(); ++i)
			{
				ANav3DDataChunkActor* CurrentActor = ActorPath[i];
				const FNav3DVolumeNavigationData* VolumeData = CurrentActor->Nav3DChunks.Num() > 0 ? 
					CurrentActor->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
				
				FVector NextPoint;
				if (i < ActorPath.Num() - 1)
				{
					// Use chunk actor center as connection point
					NextPoint = ActorPath[i + 1]->DataChunkActorBounds.GetCenter();
				}
				else
				{
					NextPoint = OriginalEnd;
				}
				
				Result.Segments.Add(FPathSegment(CurrentPoint, NextPoint, VolumeData));
				CurrentPoint = NextPoint;
			}
			UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Created %d segments based on actor path"), Result.Segments.Num());
		}
		else
		{
			UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Cross-volume pathfinder failed - using fallback segments"));
			// Fallback: create simple segments
			const FNav3DVolumeNavigationData* StartVolumeData = StartChunkActor->Nav3DChunks.Num() > 0 ? 
				StartChunkActor->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
			const FNav3DVolumeNavigationData* EndVolumeData = EndChunkActor->Nav3DChunks.Num() > 0 ? 
				EndChunkActor->Nav3DChunks[0]->GetVolumeNavigationData() : nullptr;
			
			Result.Segments.Add(FPathSegment(OriginalStart, StartChunkActor->DataChunkActorBounds.GetCenter(), StartVolumeData));
			Result.Segments.Add(FPathSegment(StartChunkActor->DataChunkActorBounds.GetCenter(), EndChunkActor->DataChunkActorBounds.GetCenter(), nullptr));
			Result.Segments.Add(FPathSegment(EndChunkActor->DataChunkActorBounds.GetCenter(), OriginalEnd, EndVolumeData));
			UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Created 3 fallback segments"));
		}
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("SanitizePath: Could not find chunk actors for one or both points (spatial subsystem)"));
	}
	
	UE_LOG(LogNav3D, Verbose, TEXT("SanitizePath: Completed with %d segments"), Result.Segments.Num());
	return Result;
}

TSharedPtr<FNav3DPathStepper> FNav3DPathFinder::CreateCrossVolumeDebugStepper(
	FNav3DPathFinderDebugData& DebugData,
	const ANav3DData& NavData,
	const TArray<ANav3DDataChunkActor*>& ActorPath,
	const TArray<FNav3DActorPortal>& Portals,
	const FVector& StartLocation,
	const FVector& EndLocation,
	const FNavAgentProperties& NavAgentProperties,
	const FSharedConstNavQueryFilter& NavQueryFilter)
{
	UE_LOG(LogNav3D, Verbose, TEXT("CreateCrossVolumeDebugStepper: Cross-volume debug steppers not yet implemented"));
	UE_LOG(LogNav3D, Verbose, TEXT("CreateCrossVolumeDebugStepper: Path found with %d actors and %d portals"), ActorPath.Num(), Portals.Num());
	UE_LOG(LogNav3D, Verbose, TEXT("CreateCrossVolumeDebugStepper: Start: %s, End: %s"), *StartLocation.ToString(), *EndLocation.ToString());
	
	// Log the actor path details
	for (int32 i = 0; i < ActorPath.Num(); ++i)
	{
		if (ANav3DDataChunkActor* Actor = ActorPath[i])
		{
			UE_LOG(LogNav3D, Verbose, TEXT("CreateCrossVolumeDebugStepper: Actor %d: %s, bounds: %s"), 
			       i, *Actor->GetName(), *Actor->DataChunkActorBounds.ToString());
		}
	}
	
	// Build the full path to see what points we actually have
	FNav3DPath FullPath;
	const ENavigationQueryResult::Type PathResult = GetPath(FullPath, NavData, StartLocation, EndLocation, NavAgentProperties, NavQueryFilter);
	
	if (PathResult == ENavigationQueryResult::Success)
	{
		const TArray<FNavPathPoint>& PathPoints = FullPath.GetPathPoints();
		UE_LOG(LogNav3D, Verbose, TEXT("CreateCrossVolumeDebugStepper: Full path has %d points"), PathPoints.Num());
		
		for (int32 i = 0; i < PathPoints.Num(); ++i)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("CreateCrossVolumeDebugStepper: Path point %d: %s"), 
			       i, *PathPoints[i].Location.ToString());
		}
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("CreateCrossVolumeDebugStepper: Failed to build full path, result: %d"), (int32)PathResult);
	}
	
	// Cross-volume debug steppers are not yet implemented
	// The debug stepper is designed for single-volume pathfinding
	// For cross-volume scenarios, we would need a different approach
	return nullptr;
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
		FVector NodePosition = BoundsData.GetNodePositionFromAddress(
			AddressWithCost.NodeAddress, false);
		UE_LOG(LogNav3D, VeryVerbose, TEXT("BuildPath: Adding path point %d at %s (Layer %d, Node %d, SubNode %d)"), 
		       Index, *NodePosition.ToString(), 
		       AddressWithCost.NodeAddress.LayerIndex,
		       AddressWithCost.NodeAddress.NodeIndex,
		       AddressWithCost.NodeAddress.SubNodeIndex);
		PathPoints.Emplace(NodePosition);
		PathPointCosts.Add(AddressWithCost.Cost);
	}

	if (AddEndLocation)
	{
		PathPoints.Emplace(Params.EndLocation);
		PathPointCosts.Add(NodeAddresses.Last().Cost);
	}
}

void FNav3DPathFinder::ValidatePathPoints(FNav3DPath& Path, const ANav3DData& NavData, const FNavAgentProperties& AgentProperties)
{
	const auto& PathPoints = Path.GetPathPoints();
	UE_LOG(LogNav3D, Warning, TEXT("ValidatePathPoints: Validating %d path points"), PathPoints.Num());
	
	// Log all path points for debugging
	for (int32 i = 0; i < PathPoints.Num(); ++i)
	{
		const FVector& Point = PathPoints[i].Location;
		UE_LOG(LogNav3D, VeryVerbose, TEXT("ValidatePathPoints: Point %d at %s"), i, *Point.ToString());
	}
	
	int32 InvalidPoints = 0;
	
	for (int32 i = 0; i < PathPoints.Num(); ++i)
	{
		const FVector& Point = PathPoints[i].Location;
		
		// Find which volume contains this point
		const FNav3DVolumeNavigationData* VolumeData = NavData.GetVolumeNavigationDataContainingPoint(Point);
		if (!VolumeData)
		{
			UE_LOG(LogNav3D, Error, TEXT("ValidatePathPoints: Point %d at %s is not in any navigation volume!"), 
			       i, *Point.ToString());
			InvalidPoints++;
			continue;
		}
		
		// Get the node address for this point
		FNav3DNodeAddress NodeAddress;
		if (!VolumeData->GetNodeAddressFromPosition(NodeAddress, Point, 0))
		{
			UE_LOG(LogNav3D, Error, TEXT("ValidatePathPoints: Point %d at %s could not be resolved to a node address!"), 
			       i, *Point.ToString());
			InvalidPoints++;
			continue;
		}
		
		// Check if the node is actually navigable
		bool bIsNavigable = false;
		if (NodeAddress.LayerIndex == 0)
		{
			// For leaf nodes, check if the specific subnode is free
			const auto& LeafNodes = VolumeData->GetData().GetLeafNodes();
			if (LeafNodes.GetLeafNodes().IsValidIndex(NodeAddress.NodeIndex))
			{
				const auto& LeafNode = LeafNodes.GetLeafNode(NodeAddress.NodeIndex);
				bIsNavigable = !LeafNode.IsSubNodeOccluded(NodeAddress.SubNodeIndex);
			}
		}
		else
		{
			// For non-leaf nodes, check if they don't have children (meaning they're free)
			const auto& Node = VolumeData->GetNodeFromAddress(NodeAddress);
			bIsNavigable = !Node.HasChildren();
		}
		
		if (!bIsNavigable)
		{
			UE_LOG(LogNav3D, Error, TEXT("ValidatePathPoints: Point %d at %s is in an OCCLUDED voxel! (Layer %d, Node %d, SubNode %d)"), 
			       i, *Point.ToString(), NodeAddress.LayerIndex, NodeAddress.NodeIndex, NodeAddress.SubNodeIndex);
			InvalidPoints++;
		}
		else
		{
			UE_LOG(LogNav3D, VeryVerbose, TEXT("ValidatePathPoints: Point %d at %s is valid (Layer %d, Node %d, SubNode %d)"), 
			       i, *Point.ToString(), NodeAddress.LayerIndex, NodeAddress.NodeIndex, NodeAddress.SubNodeIndex);
		}
	}
	
	if (InvalidPoints > 0)
	{
		UE_LOG(LogNav3D, Error, TEXT("ValidatePathPoints: Found %d invalid path points out of %d total points!"), 
		       InvalidPoints, PathPoints.Num());
	}
	else
	{
		UE_LOG(LogNav3D, Log, TEXT("ValidatePathPoints: All %d path points are valid!"), PathPoints.Num());
	}
}
