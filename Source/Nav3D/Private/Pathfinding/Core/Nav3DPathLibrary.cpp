#include "Pathfinding/Core/Nav3DPathLibrary.h"

#include "EngineUtils.h"
#include "Nav3D.h"
#include "Nav3DData.h"
#include "NavigationSystem.h"
#include "Pathfinding/Core/Nav3DPath.h"
#include "Pathfinding/Core/Nav3DPathCoordinator.h"
#include "Pathfinding/Core/Nav3DPathingTypes.h"

bool UNav3DPathLibrary::FindNav3DPath(
    const UObject* WorldContextObject,
    FVector StartLocation,
    FVector EndLocation,
    float AgentRadius,
    TArray<FVector>& OutPathPoints)
{
    OutPathPoints.Empty();
    
    UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    if (!World)
    {
        UE_LOG(LogNav3D, Error, TEXT("FindNav3DPathSimple: No world context"));
        return false;
    }
    
    UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(World);
    if (!NavSys)
    {
        UE_LOG(LogNav3D, Error, TEXT("FindNav3DPathSimple: No navigation system"));
        return false;
    }
    
    // Find the FIRST Nav3DData actor in the world
    const ANav3DData* Nav3dData = nullptr;
    for (TActorIterator<ANav3DData> It(World); It; ++It)
    {
        Nav3dData = *It;
        break;
    }
    
    if (!Nav3dData)
    {
        UE_LOG(LogNav3D, Error, TEXT("FindNav3DPathSimple: No Nav3DData actor found in world"));
        return false;
    }
    
    UE_LOG(LogNav3D, Log, TEXT("FindNav3DPathSimple: Found Nav3DData: %s"), *Nav3dData->GetName());
    
    // Build request - use the NavData's own agent properties
    FNav3DPathingRequest Request;
    Request.StartLocation = StartLocation;
    Request.EndLocation = EndLocation;
    Request.NavData = Nav3dData;
    Request.AgentProperties = Nav3dData->GetNavAgentProperties();  // Use NavData's properties!
    Request.AgentProperties.AgentRadius = AgentRadius;  // Override just the radius
    Request.LogVerbosity = ENav3DPathingLogVerbosity::Verbose;
    
    UE_LOG(LogNav3D, Log, TEXT("FindNav3DPathSimple: Calling FindPath from %s to %s with radius %.2f"), 
        *StartLocation.ToString(), *EndLocation.ToString(), AgentRadius);
    
    // Find path
    FNav3DPath Path;
    ENavigationQueryResult::Type Result = FNav3DPathCoordinator::FindPath(Path, Request);
    
    UE_LOG(LogNav3D, Log, TEXT("FindNav3DPathSimple: Result = %d"), (int32)Result);
    
    if (Result == ENavigationQueryResult::Success)
    {
        const TArray<FNavPathPoint>& PathPoints = Path.GetPathPoints();
        UE_LOG(LogNav3D, Log, TEXT("FindNav3DPathSimple: Got %d path points"), PathPoints.Num());
        OutPathPoints.Reserve(PathPoints.Num());
        for (const FNavPathPoint& Point : PathPoints)
        {
            OutPathPoints.Add(Point.Location);
        }
        return true;
    }
    
    return false;
}