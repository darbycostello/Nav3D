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
#include "Pathfinding/Stepper/Nav3DPathStepper.h"

#if WITH_EDITOR
#endif

void FNav3DPathFindingSceneProxyData::GatherData(
	const ANav3DPathFinderTest& PathFinderTest)
{
	StartLocation = PathFinderTest.GetStartLocation();
	EndLocation = PathFinderTest.GetEndLocation();
	DebugInfos = PathFinderTest.GetPathFinderDebugInfos();
	Stepper = PathFinderTest.GetStepper();

	if (PathFinderTest.GetStepperLastStatus() ==
		ENav3DPathStepperStatus::IsStopped)
	{
		PathFindingResult =
			TOptional(PathFinderTest.GetPathFindingResult());
	}
	else
	{
		PathFindingResult.Reset();
	}
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

	if (!ProxyData.Stepper.IsValid())
	{
		return;
	}

	const auto AddText = [Texts = &Texts](
		const FNav3DPathFinderDebugNodeCost& DebugNodeCost)
	{
		Texts->Emplace(FText3d(
			FString::SanitizeFloat(DebugNodeCost.Cost),
			FVector(0.0f, 0.0f, 50.0f) +
			(DebugNodeCost.From.Location + DebugNodeCost.To.Location) / 2.0f,
			FLinearColor::White));
	};

	const auto VisualizeDebugNodeCost =
		[this, AddText,
			ProxyData](const FNav3DPathFinderDebugNodeCost& DebugNodeCost,
			           const FColor& Color)
	{
		if (!DebugNodeCost.From.NodeAddress.IsValid() ||
			!DebugNodeCost.To.NodeAddress.IsValid())
		{
			return;
		}

		const auto& VolumeNavigationData =
			ProxyData.Stepper->GetParameters().VolumeNavigationData;

		if (DebugDrawOptions.bDrawNodes)
		{
			const auto FromNodeExtent =
				VolumeNavigationData.GetNodeExtentFromNodeAddress(DebugNodeCost.From.NodeAddress);
			Boxes.Emplace(FBox::BuildAABB(DebugNodeCost.From.Location,
			                              FVector(FromNodeExtent)),
			              Color);

			const auto ToNodeExtent =
				VolumeNavigationData.GetNodeExtentFromNodeAddress(
					DebugNodeCost.To.NodeAddress);
			Boxes.Emplace(
				FBox::BuildAABB(DebugNodeCost.To.Location, FVector(ToNodeExtent)),
				Color);
		}

		if (DebugDrawOptions.bDrawConnections)
		{
			Lines.Emplace(FDebugLine(DebugNodeCost.From.Location,
			                         DebugNodeCost.To.Location, FColor::Blue,
			                         2.0f));
		}

		if (DebugDrawOptions.bDrawCosts)
		{
			AddText(ProxyData.DebugInfos.LastProcessedSingleNode);
		}
	};

	if (DebugDrawOptions.bDrawLastProcessedNode)
	{
		VisualizeDebugNodeCost(ProxyData.DebugInfos.LastProcessedSingleNode,
		                       FColor::Blue);
	}

	if (DebugDrawOptions.bDrawLastProcessedNeighbours)
	{
		for (const auto& Neighbour : ProxyData.DebugInfos.ProcessedNeighbours)
		{
			VisualizeDebugNodeCost(Neighbour, Neighbour.bIsClosed
				                                  ? FColor::Orange
				                                  : FColor::Green);
		}
	}

	if (ProxyData.PathFindingResult.Get(SearchFail) == SearchSuccess ||
		DebugDrawOptions.bDrawBestPath)
	{
		const auto& BestPathPoints =
			ProxyData.DebugInfos.CurrentBestPath.GetPathPoints();

		ArrowHeadLocations.Reserve(BestPathPoints.Num());

		for (auto Index = 0; Index < BestPathPoints.Num() - 1; Index++)
		{
			const auto From = BestPathPoints[Index];
			const auto To = BestPathPoints[Index + 1];

			Lines.Emplace(From, To, FColor::Blue, 3.0f);
			Boxes.Emplace(FBox::BuildAABB(From, FVector(20.0f)), FColor::Cyan);
			ArrowHeadLocations.Emplace(From, To);
		}
	}
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
		if (VisibilityMap & (1 << ViewIndex))
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
	AutoStepTimer = 0.2f;
	bAutoComplete = false;
	bUpdatePathAfterMoving = false;
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
			InitPathFinding();
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
			InitPathFinding();
			AutoCompleteInstantly();
		}
		else if (OtherActor->bUpdatePathAfterMoving)
		{
			OtherActor->InitPathFinding();
			OtherActor->AutoCompleteInstantly();
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

void ANav3DPathFinderTest::InitPathFinding()
{
	Stepper.Reset();

	UWorld* World = GetWorld();
	UNavigationSystemV1* NavigationSystem = UNavigationSystemV1::GetCurrent(World);
	if (const UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(World))
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Navigation System has %d supported agents"),
		       NavSys->GetSupportedAgents().Num());
	}

	if (NavigationSystem)
	{
		// Log the nav agent properties
		UE_LOG(LogNav3D, Log, TEXT("Nav Agent Properties:"));
		UE_LOG(LogNav3D, Log, TEXT("  - AgentRadius: %f"), NavAgentProperties.AgentRadius);
		UE_LOG(LogNav3D, Log, TEXT("  - AgentHeight: %f"), NavAgentProperties.AgentHeight);

		const auto SupportedAgents = NavigationSystem->GetSupportedAgents();
		UE_LOG(LogNav3D, Log, TEXT("SupportedAgents: %d"), SupportedAgents.Num());


		if (auto* NavigationData = NavigationSystem->GetNavDataForProps(NavAgentProperties))
		{
			// Log all available nav data
			UE_LOG(LogNav3D, Log, TEXT("Available Nav Data:"));
			for (const ANavigationData* NavData : NavigationSystem->NavDataSet)
			{
				UE_LOG(LogNav3D, Log, TEXT("  - %s (%s)"),
				       *GetNameSafe(NavData),
				       *GetNameSafe(NavData->GetClass()));
			}

			if (const auto* N3dNavigationData = Cast<ANav3DData>(NavigationData))
			{
				if (OtherActor != nullptr)
				{
					const auto PathStart = GetActorLocation();
					const auto PathEnd = OtherActor->GetActorLocation();

					const auto QueryFilter = UNavigationQueryFilter::GetQueryFilter(
						*N3dNavigationData, this, NavigationQueryFilter);
					Stepper = FNav3DPathFinder::GetDebugPathStepper(
						PathFinderDebugInfos,
						*N3dNavigationData,
						PathStart, PathEnd,
						NavAgentProperties,
						QueryFilter);

					if (!Stepper.IsValid())
					{
						return;
					}

					PathFinderDebugInfos.Reset();
					PathFinderDebugInfos.StartNodeAddress =
						Stepper->GetParameters().StartNodeAddress.ToString();
					PathFinderDebugInfos.EndNodeAddress =
						Stepper->GetParameters().EndNodeAddress.ToString();
					NavigationPath.ResetForRepath();
					LastStatus = ENav3DPathStepperStatus::MustContinue;
					PathFindingResult = SearchFail;
					bAutoComplete = false;

					UpdateDrawing();
					return;
				}
			}
		}
		else
		{
			UE_LOG(LogNav3D, Error,
			       TEXT("InitPathFinding failed - GetNavDataForProps did not return nav data for NavAgentProperties"));
		}
	}

	ensureAlwaysMsgf(false, TEXT("Impossible to get the Nav3D navigation data. Check NavAgentProperties"));
}

void ANav3DPathFinderTest::InitPathFindingIfNotDone()
{
	if (Stepper.IsValid())
	{
		return;
	}

	InitPathFinding();
}

void ANav3DPathFinderTest::ResetPathFinding() { InitPathFinding(); }

void ANav3DPathFinderTest::Step()
{
	if (!Stepper.IsValid())
	{
		return;
	}

	if (LastStatus != ENav3DPathStepperStatus::IsStopped)
	{
		LastStatus = Stepper->Step(PathFindingResult);
		if (LastStatus == ENav3DPathStepperStatus::MustContinue)
		{
			UpdateDrawing();

			if (bAutoComplete)
			{
				GetWorld()->GetTimerManager().SetTimer(AutoCompleteTimerHandle, this,
				                                       &ANav3DPathFinderTest::Step,
				                                       AutoStepTimer, false);
				return;
			}
		}
		else if (PathFindingResult == SearchSuccess)
		{
			UpdateDrawing();
		}
	}

	bAutoComplete = false;
	GetWorld()->GetTimerManager().ClearAllTimersForObject(this);
}

void ANav3DPathFinderTest::AutoCompleteStepByStep()
{
	InitPathFinding();
	bAutoComplete = true;
	Step();
}

void ANav3DPathFinderTest::AutoCompleteUntilNextNode()
{
	InitPathFindingIfNotDone();

	if (!Stepper.IsValid())
	{
		return;
	}

	if (LastStatus != ENav3DPathStepperStatus::IsStopped)
	{
		do
		{
			LastStatus = Stepper->Step(PathFindingResult);
		}
		while (
			LastStatus == ENav3DPathStepperStatus::MustContinue &&
			Stepper->GetState() != ENav3DPathFindingState::ProcessNode);

		UpdateDrawing();
	}

	GetWorld()->GetTimerManager().ClearAllTimersForObject(this);
}

void ANav3DPathFinderTest::AutoCompleteInstantly()
{
	InitPathFindingIfNotDone();

	if (!Stepper.IsValid())
	{
		return;
	}

	if (LastStatus != ENav3DPathStepperStatus::IsStopped)
	{
		do
		{
			LastStatus = Stepper->Step(PathFindingResult);
		}
		while (LastStatus ==
			ENav3DPathStepperStatus::MustContinue);

		UpdateDrawing();
	}

	GetWorld()->GetTimerManager().ClearAllTimersForObject(this);
}

void ANav3DPathFinderTest::PauseAutoCompletion() { bAutoComplete = false; }
