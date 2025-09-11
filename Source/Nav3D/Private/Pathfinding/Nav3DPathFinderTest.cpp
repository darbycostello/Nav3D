// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#include "PathFinding/Nav3DPathFinderTest.h"
#include "Nav3DData.h"
#include "PathFinding/Nav3DPathFinder.h"
#include "PathFinding/Search/Nav3DPathFindingSearch.h"
#include <Components/SphereComponent.h>
#include <NavigationSystem.h>

#include "Nav3D.h"
#include "NavFilters/NavigationQueryFilter.h"
#include "Pathfinding/Stepper/Nav3DPathFindingRenderingComponent.h"

#if WITH_EDITOR
#endif

void FNav3DPathFindingSceneProxyData::GatherData(
	const ANav3DPathFinderTest& PathFinderTest)
{
	StartLocation = PathFinderTest.GetStartLocation();
	EndLocation = PathFinderTest.GetEndLocation();
	NavigationPath = PathFinderTest.GetNavigationPath();
	PathFindingResult = TOptional(PathFinderTest.GetPathFindingResult());
}

FNav3DPathFindingSceneProxy::FNav3DPathFindingSceneProxy(
	const UPrimitiveComponent& Component,
	const FNav3DPathFindingSceneProxyData& ProxyData)
	: FDebugRenderSceneProxy(&Component)
{
	DrawType = WireMesh;
	TextWithoutShadowDistance = 1500;
	bWantsSelectionOutline = false;
	ViewFlagName = TEXT("Navigation");
	ViewFlagIndex =
		static_cast<uint32>(FEngineShowFlags::FindIndexByName(*ViewFlagName));

	RenderingComponent =
		MakeWeakObjectPtr(const_cast<UNav3DPathFindingRenderingComponent*>(
			Cast<UNav3DPathFindingRenderingComponent>(&Component)));
	PathFinderTest = RenderingComponent->GetPathFinderTest();
	DebugDrawOptions = PathFinderTest->GetDebugDrawOptions();
	ActorOwner = Component.GetOwner();

	// Check if we have a valid pathfinding result
	if (!ProxyData.PathFindingResult.IsSet() || ProxyData.PathFindingResult.GetValue() != SearchSuccess)
	{
		return;
	}

	// Draw start and end points
	const FVector StartLocation = ProxyData.StartLocation;
	const FVector EndLocation = ProxyData.EndLocation;

	// Start point (green sphere)
	Boxes.Emplace(FBox::BuildAABB(StartLocation, FVector(100.0f)), FColor::Green);
	Texts.Emplace(FText3d(TEXT("Start"), StartLocation + FVector(0.0f, 0.0f, 150.0f), FLinearColor::Green));

	// End point (red sphere)
	Boxes.Emplace(FBox::BuildAABB(EndLocation, FVector(100.0f)), FColor::Red);
	Texts.Emplace(FText3d(TEXT("End"), EndLocation + FVector(0.0f, 0.0f, 150.0f), FLinearColor::Red));

	// Draw the navigation path
	const auto& PathPoints = ProxyData.NavigationPath.GetPathPoints();
	if (PathPoints.Num() > 1)
	{
		// Draw path lines
		for (int32 i = 0; i < PathPoints.Num() - 1; ++i)
		{
			Lines.Emplace(FDebugLine(PathPoints[i], PathPoints[i + 1], FColor::Green, 3.0f));
			ArrowHeadLocations.Emplace(PathPoints[i], PathPoints[i + 1]);
		}
		
		// Draw path points as small spheres
		for (int32 i = 0; i < PathPoints.Num(); ++i)
		{
			FVector Extent(50.0f); // Small sphere size
			Boxes.Emplace(FBox::BuildAABB(PathPoints[i], Extent), FColor::Yellow);
		}
	}

	// Draw direct line between start and end (for reference)
	Lines.Emplace(FDebugLine(StartLocation, EndLocation, FColor::Blue, 1.0f));
}

SIZE_T FNav3DPathFindingSceneProxy::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}

FPrimitiveViewRelevance
FNav3DPathFindingSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	FPrimitiveViewRelevance Result;
	Result.bDrawRelevance =
		IsShown(View) &&
		(!DebugDrawOptions.bDrawOnlyWhenSelected || SafeIsActorSelected());
	Result.bDynamicRelevance = true;
	Result.bSeparateTranslucency = Result.bNormalTranslucency = IsShown(View);
	return Result;
}

void FNav3DPathFindingSceneProxy::GetDynamicMeshElements(
	const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily,
	const uint32 VisibilityMap, FMeshElementCollector& Collector) const
{
	FDebugRenderSceneProxy::GetDynamicMeshElements(Views, ViewFamily,
	                                               VisibilityMap, Collector);

	for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
	{
		FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);
		if (VisibilityMap & 1 << ViewIndex)
		{
			for (const auto& Pair : ArrowHeadLocations)
			{
				DrawArrowHead(PDI, Pair.Value, Pair.Key, 50.f, FColor::Red, SDPG_World,
				              10.0f);
			}
		}
	}
}

bool FNav3DPathFindingSceneProxy::SafeIsActorSelected() const
{
	if (ActorOwner)
	{
		return ActorOwner->IsSelected();
	}

	return false;
}

UNav3DPathFindingRenderingComponent::UNav3DPathFindingRenderingComponent()
{
}

FPrimitiveSceneProxy* UNav3DPathFindingRenderingComponent::CreateSceneProxy()
{
	FNav3DPathFindingSceneProxyData ProxyData;
	ProxyData.GatherData(*GetPathFinderTest());

	if (FNav3DPathFindingSceneProxy* NewSceneProxy =
		new FNav3DPathFindingSceneProxy(*this, ProxyData))
	{
		return NewSceneProxy;
	}

	return nullptr;
}

FBoxSphereBounds UNav3DPathFindingRenderingComponent::CalcBounds(
	const FTransform& LocalToWorld) const
{
	FBoxSphereBounds Result = FBoxSphereBounds();

	if (const auto* Owner = GetPathFinderTest())
	{
		FVector Center, Extent;
		Owner->GetActorBounds(false, Center, Extent);
		Result = FBoxSphereBounds(FBox::BuildAABB(Center, Extent));
	}

	return Result;
}

ANav3DPathFinderTest::ANav3DPathFinderTest()
{
	PrimaryActorTick.bCanEverTick = false;
	PrimaryActorTick.bStartWithTickEnabled = false;

	SphereComponent =
		CreateDefaultSubobject<USphereComponent>(TEXT("SphereComponent"));
	RootComponent = SphereComponent;

#if WITH_EDITORONLY_DATA
	RenderingComponent =
		CreateEditorOnlyDefaultSubobject<UNav3DPathFindingRenderingComponent>(
			TEXT("RenderingComponent"));
	if (RenderingComponent != nullptr)
	{
		RenderingComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}
#endif

	NavAgentProperties = FNavAgentProperties::DefaultProperties;
	bUpdatePathAfterMoving = false;
	PathFindingResult = SearchFail;
}

#if WITH_EDITOR
void ANav3DPathFinderTest::PreEditChange(FProperty* PropertyAboutToChange)
{
	static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED(ANav3DPathFinderTest, OtherActor);

	if (PropertyAboutToChange != nullptr &&
		PropertyAboutToChange->GetFName() == NAME_OtherActor &&
		OtherActor != nullptr && OtherActor->OtherActor == this)
	{
		OtherActor->OtherActor = nullptr;
		OtherActor->NavigationPath.ResetForRepath();
		NavigationPath.ResetForRepath();
#if WITH_EDITORONLY_DATA
		OtherActor->RenderingComponent->MarkRenderStateDirty();
		RenderingComponent->MarkRenderStateDirty();
#endif
	}

	Super::PreEditChange(PropertyAboutToChange);
}

void ANav3DPathFinderTest::PostEditChangeProperty(
	FPropertyChangedEvent& PropertyChangedEvent)
{
	static const FName NAME_NavigationQueryFilter = GET_MEMBER_NAME_CHECKED(
		ANav3DPathFinderTest, NavigationQueryFilter);
	static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED(ANav3DPathFinderTest, OtherActor);
	static const FName NAME_UpdatePathAfterMoving = GET_MEMBER_NAME_CHECKED(
		ANav3DPathFinderTest, bUpdatePathAfterMoving);

	if (PropertyChangedEvent.Property != nullptr)
	{
		const FName PropertyName = PropertyChangedEvent.MemberProperty->GetFName();
		if (PropertyName == NAME_NavigationQueryFilter)
		{
			FindPath();
		}
		else if (PropertyName == NAME_OtherActor)
		{
			if (OtherActor != nullptr)
			{
				auto* OtherActorsOldOtherActor = OtherActor->OtherActor;

				OtherActor->OtherActor = this;

#if WITH_EDITORONLY_DATA
				RenderingComponent->MarkRenderStateDirty();
#endif

				if (OtherActorsOldOtherActor != nullptr)
				{
					OtherActorsOldOtherActor->OtherActor = nullptr;
					OtherActorsOldOtherActor->NavigationPath.ResetForRepath();
#if WITH_EDITORONLY_DATA
					OtherActorsOldOtherActor->RenderingComponent->MarkRenderStateDirty();
#endif
				}
			}
		}
		else if (PropertyName == NAME_UpdatePathAfterMoving)
		{
			if (bUpdatePathAfterMoving && OtherActor != nullptr)
			{
				OtherActor->bUpdatePathAfterMoving = false;
			}
		}
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}

void ANav3DPathFinderTest::PostEditMove(const bool IsFinished)
{
	Super::PostEditMove(IsFinished);

	if (OtherActor != nullptr)
	{
		if (bUpdatePathAfterMoving)
		{
			FindPath();
		}
		else if (OtherActor->bUpdatePathAfterMoving)
		{
			OtherActor->FindPath();
		}
	}
}
#endif

void ANav3DPathFinderTest::BeginDestroy()
{
	NavigationPath.ResetForRepath();

	if (OtherActor != nullptr && OtherActor->OtherActor == this)
	{
		OtherActor->OtherActor = nullptr;
		OtherActor->NavigationPath.ResetForRepath();
	}

	Super::BeginDestroy();
}

void ANav3DPathFinderTest::UpdateDrawing() const
{
#if WITH_EDITORONLY_DATA
	if (HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	if (RenderingComponent != nullptr && RenderingComponent->GetVisibleFlag())
	{
		RenderingComponent->MarkRenderStateDirty();

#if WITH_EDITOR
		if (GEditor != nullptr)
		{
			GEditor->RedrawLevelEditingViewports();
		}
#endif // WITH_EDITOR
	}
#endif // WITH_EDITORONLY_DATA
}

void ANav3DPathFinderTest::FindPath()
{
	if (OtherActor == nullptr)
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindPath: No OtherActor set"));
		return;
	}

	UWorld* World = GetWorld();
	UNavigationSystemV1* NavigationSystem = UNavigationSystemV1::GetCurrent(World);
	if (!NavigationSystem)
	{
		UE_LOG(LogNav3D, Error, TEXT("FindPath: No navigation system found"));
		return;
	}

	auto* NavigationData = NavigationSystem->GetNavDataForProps(NavAgentProperties);
	if (!NavigationData)
	{
		UE_LOG(LogNav3D, Error, TEXT("FindPath: No navigation data found for agent properties"));
		return;
	}

	const auto* N3dNavigationData = Cast<ANav3DData>(NavigationData);
	if (!N3dNavigationData)
	{
		UE_LOG(LogNav3D, Error, TEXT("FindPath: Navigation data is not Nav3D data"));
		return;
	}

	const FVector PathStart = GetActorLocation();
	const FVector PathEnd = OtherActor->GetActorLocation();

	UE_LOG(LogNav3D, Log, TEXT("FindPath: Finding path from %s to %s"), 
	       *PathStart.ToString(), *PathEnd.ToString());

	const auto QueryFilter = UNavigationQueryFilter::GetQueryFilter(
		*N3dNavigationData, this, NavigationQueryFilter);

	// Reset the path and debug info
	NavigationPath.ResetForRepath();
	PathFinderDebugInfos.Reset();
	PathFindingResult = SearchFail;

	// Perform pathfinding
	const ENavigationQueryResult::Type Result = FNav3DPathFinder::GetPath(
		NavigationPath,
		*N3dNavigationData,
		PathStart,
		PathEnd,
		NavAgentProperties,
		QueryFilter);

	// Convert result
	if (Result == ENavigationQueryResult::Success)
	{
		PathFindingResult = SearchSuccess;
		UE_LOG(LogNav3D, Log, TEXT("FindPath: Pathfinding successful! Found %d path points"), 
		       NavigationPath.GetPathPoints().Num());
	}
	else
	{
		PathFindingResult = SearchFail;
		UE_LOG(LogNav3D, Warning, TEXT("FindPath: Pathfinding failed with result: %d"), (int32)Result);
	}

	UpdateDrawing();
}

void ANav3DPathFinderTest::ClearPaths()
{
    NavigationPath.ResetForRepath();
    PathFinderDebugInfos.Reset();
    PathFindingResult = SearchFail;
    if (OtherActor && OtherActor->OtherActor == this)
    {
        OtherActor->NavigationPath.ResetForRepath();
        OtherActor->PathFinderDebugInfos.Reset();
        OtherActor->PathFindingResult = SearchFail;
#if WITH_EDITORONLY_DATA
        if (OtherActor->RenderingComponent)
        {
            OtherActor->RenderingComponent->MarkRenderStateDirty();
        }
#endif
    }
#if WITH_EDITORONLY_DATA
    if (RenderingComponent)
    {
        RenderingComponent->MarkRenderStateDirty();
    }
#endif
}

// Removed redundant stepper-related method implementations
