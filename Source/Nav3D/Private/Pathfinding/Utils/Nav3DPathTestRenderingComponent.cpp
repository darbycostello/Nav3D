#include "Pathfinding/Utils/Nav3DPathTestRenderingComponent.h"
#include "PrimitiveViewRelevance.h"
#include "Engine/Engine.h"

void FNav3DPathTestSceneProxyData::GatherData(
	const ANav3DPathTest& PathTest)
{
	StartLocation = PathTest.GetStartLocation();
	EndLocation = PathTest.GetEndLocation();
	NavigationPath = PathTest.GetNavigationPath();
	PathFindingResult = PathTest.GetPathFindingResult();
}

FNav3DPathTestSceneProxy::FNav3DPathTestSceneProxy(
	const UPrimitiveComponent& Component,
	const FNav3DPathTestSceneProxyData& ProxyData)
	: FDebugRenderSceneProxy(&Component)
{
	DrawType = WireMesh;
	TextWithoutShadowDistance = 1500;
	bWantsSelectionOutline = false;
	ViewFlagName = TEXT("Navigation");
	ViewFlagIndex =
		static_cast<uint32>(FEngineShowFlags::FindIndexByName(*ViewFlagName));

	RenderingComponent =
		MakeWeakObjectPtr(const_cast<UNav3DPathTestRenderingComponent*>(
			Cast<UNav3DPathTestRenderingComponent>(&Component)));
	PathTest = RenderingComponent->GetPathTest();
	DebugDrawOptions = PathTest->GetDebugDrawOptions();
	ActorOwner = Component.GetOwner();

	// Check if we have a valid pathfinding result
	if (ProxyData.PathFindingResult != ENavigationQueryResult::Success)
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
			Lines.Emplace(FDebugLine(PathPoints[i].Location, PathPoints[i + 1].Location, FColor::Green, 3.0f));
			ArrowHeadLocations.Emplace(PathPoints[i].Location, PathPoints[i + 1].Location);
		}
		
		// Draw path points as small spheres
		for (int32 i = 0; i < PathPoints.Num(); ++i)
		{
			FVector Extent(50.0f); // Small sphere size
			Boxes.Emplace(FBox::BuildAABB(PathPoints[i].Location, Extent), FColor::Yellow);
		}
	}

	// Draw direct line between start and end (for reference)
	Lines.Emplace(FDebugLine(StartLocation, EndLocation, FColor::Blue, 1.0f));
}

SIZE_T FNav3DPathTestSceneProxy::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}

FPrimitiveViewRelevance
FNav3DPathTestSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	FPrimitiveViewRelevance Result;
	Result.bDrawRelevance =
		IsShown(View) &&
		(!DebugDrawOptions.bDrawOnlyWhenSelected || SafeIsActorSelected());
	Result.bDynamicRelevance = true;
	Result.bSeparateTranslucency = Result.bNormalTranslucency = IsShown(View);
	return Result;
}

void FNav3DPathTestSceneProxy::GetDynamicMeshElements(
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

bool FNav3DPathTestSceneProxy::SafeIsActorSelected() const
{
	if (ActorOwner)
	{
		return ActorOwner->IsSelected();
	}

	return false;
}

UNav3DPathTestRenderingComponent::UNav3DPathTestRenderingComponent()
{
}

FPrimitiveSceneProxy* UNav3DPathTestRenderingComponent::CreateSceneProxy()
{
	FNav3DPathTestSceneProxyData ProxyData;
	ProxyData.GatherData(*GetPathTest());

	if (FNav3DPathTestSceneProxy* NewSceneProxy =
		new FNav3DPathTestSceneProxy(*this, ProxyData))
	{
		return NewSceneProxy;
	}

	return nullptr;
}

FBoxSphereBounds UNav3DPathTestRenderingComponent::CalcBounds(
	const FTransform& LocalToWorld) const
{
	FBoxSphereBounds Result = FBoxSphereBounds();

	if (const auto* Owner = GetPathTest())
	{
		FVector Center, Extent;
		Owner->GetActorBounds(false, Center, Extent);
		Result = FBoxSphereBounds(FBox::BuildAABB(Center, Extent));
	}

	return Result;
}


