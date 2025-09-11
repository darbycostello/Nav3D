// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#include "Raycasting/Nav3DRaycasterTest.h"
#include "Nav3DData.h"
#include "Raycasting/Nav3DRaycaster.h"
#include "Nav3D.h"
#include <Components/SphereComponent.h>
#include <Debug/DebugDrawService.h>
#include <NavigationSystem.h>

void FNav3DRaycasterSceneProxyData::GatherData(
	const ANav3DRaycasterTest& RaycasterTest)
{
	DebugInfos = RaycasterTest.GetDebugInfos();
}

FNav3DRaycasterSceneProxy::FNav3DRaycasterSceneProxy(
	const UPrimitiveComponent& Component,
	const FNav3DRaycasterSceneProxyData& ProxyData)
	: FDebugRenderSceneProxy(&Component)
{
	DrawType = SolidAndWireMeshes;
	RaycasterTest = Cast<ANav3DRaycasterTest>(Component.GetOwner());

	const auto& DebugDrawOptions = RaycasterTest->GetDebugDrawOptions();

	if (ProxyData.DebugInfos.NavigationData == nullptr)
	{
		return;
	}

	if (!DebugDrawOptions.bEnableDebugDraw)
	{
		return;
	}

	Lines.Emplace(ProxyData.DebugInfos.RaycastStartLocation,
	              ProxyData.DebugInfos.RaycastEndLocation,
	              ProxyData.DebugInfos.Result ? FColor::Red : FColor::Green,
	              5.0f);

	const auto LayerCount = ProxyData.DebugInfos.NavigationData->GetData().GetLayerCount();
	const auto CorrectedLayerIndex = FMath::Clamp(
		static_cast<int>(DebugDrawOptions.LayerIndexToDraw), 0, LayerCount - 1);

	const auto DrawMortonCoords = [&DebugDrawOptions,
			this](const FVector& Location,
			      const FNav3DNodeAddress NodeAddress)
	{
		if (DebugDrawOptions.bDrawMortonCode)
		{
			Texts.Emplace(FString::Printf(TEXT("%i:%i:%i"), NodeAddress.LayerIndex,
			                              NodeAddress.NodeIndex,
			                              NodeAddress.SubNodeIndex),
			              Location + FVector(0.0f, 0.0f, 40.0f), FLinearColor::Black);
		}
	};

	if (DebugDrawOptions.bDrawLayerNodes)
	{
		for (const auto& TraversedNode : ProxyData.DebugInfos.TraversedNodes)
		{
			if (TraversedNode.NodeAddress.LayerIndex != CorrectedLayerIndex)
			{
				continue;
			}

			const auto NodePosition =
				ProxyData.DebugInfos.NavigationData->GetNodePositionFromAddress(
					TraversedNode.NodeAddress, false);
			const auto NodeExtent =
				ProxyData.DebugInfos.NavigationData->GetData()
				         .GetLayer(TraversedNode.NodeAddress.LayerIndex)
				         .GetNodeExtent();

			Boxes.Emplace(FBox::BuildAABB(NodePosition, FVector(NodeExtent)),
			              TraversedNode.bIsOccluded ? FColor::Orange : FColor::Green);
			DrawMortonCoords(NodePosition, TraversedNode.NodeAddress);
		}
	}
}

SIZE_T FNav3DRaycasterSceneProxy::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}

FPrimitiveViewRelevance
FNav3DRaycasterSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	FPrimitiveViewRelevance Result;
	Result.bDrawRelevance = IsShown(View);
	Result.bDynamicRelevance = true;
	Result.bSeparateTranslucency = Result.bNormalTranslucency = IsShown(View);
	return Result;
}

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST

void FNav3DRaycasterDebugDrawDelegateHelper::InitDelegateHelper(
	const FNav3DRaycasterSceneProxy* SceneProxy)
{
	Super::InitDelegateHelper(SceneProxy);
}

void FNav3DRaycasterDebugDrawDelegateHelper::
RegisterDebugDrawDelegateInternal()
{
	if (State == RegisteredState)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Debug draw delegate is already registered"));
		return;
	}
	if (State == InitializedState)
	{
		DebugTextDrawingDelegate = FDebugDrawDelegate::CreateRaw(
			this, &FNav3DRaycasterDebugDrawDelegateHelper::DrawDebugLabels);
		DebugTextDrawingDelegateHandle = UDebugDrawService::Register(
			TEXT("Navigation"), DebugTextDrawingDelegate);
		State = RegisteredState;
	}
}

void FNav3DRaycasterDebugDrawDelegateHelper::UnregisterDebugDrawDelegate()
{
	if (State == RegisteredState)
	{
		check(DebugTextDrawingDelegate.IsBound());
		UDebugDrawService::Unregister(DebugTextDrawingDelegateHandle);
		State = InitializedState;
	}
}
#endif

ANav3DRaycasterTest*
UNav3DRaycasterRenderingComponent::GetRaycasterTest() const
{
	return Cast<ANav3DRaycasterTest>(GetOwner());
}

void UNav3DRaycasterRenderingComponent::CreateRenderState_Concurrent(
	FRegisterComponentContext* Context)
{
	Super::CreateRenderState_Concurrent(Context);

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
	DebugDrawDelegateManager.RequestRegisterDebugDrawDelegate(Context);
#endif
}

void UNav3DRaycasterRenderingComponent::DestroyRenderState_Concurrent()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
	DebugDrawDelegateManager.UnregisterDebugDrawDelegate();
#endif

	Super::DestroyRenderState_Concurrent();
}

FPrimitiveSceneProxy* UNav3DRaycasterRenderingComponent::CreateSceneProxy()
{
	FNav3DRaycasterSceneProxyData ProxyData;
	ProxyData.GatherData(*GetRaycasterTest());

	if (FNav3DRaycasterSceneProxy* NewSceneProxy =
		new FNav3DRaycasterSceneProxy(*this, ProxyData))
	{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
		DebugDrawDelegateManager.InitDelegateHelper(NewSceneProxy);
		DebugDrawDelegateManager.ReregisterDebugDrawDelegate();
#endif

		return NewSceneProxy;
	}

	return nullptr;
}

FBoxSphereBounds UNav3DRaycasterRenderingComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	if (const auto* Owner = GetRaycasterTest())
	{
		return Owner->GetBoundingBoxContainingOtherActorAndMe();
	}

	return FBoxSphereBounds();
}

ANav3DRaycasterTest::ANav3DRaycasterTest()
{
	SphereComponent =
		CreateDefaultSubobject<USphereComponent>(TEXT("SphereComponent"));
	SphereComponent->InitSphereRadius(100.0f);
	RootComponent = SphereComponent;

#if WITH_EDITORONLY_DATA
	RenderingComponent =
		CreateEditorOnlyDefaultSubobject<UNav3DRaycasterRenderingComponent>(
			TEXT("RenderingComponent"));
	if (RenderingComponent != nullptr)
	{
		RenderingComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}
#endif

	PrimaryActorTick.bCanEverTick = false;

	Raycaster = NewObject<UNav3DRaycaster>();
	NavAgentProperties.PreferredNavData = ANav3DData::StaticClass();
	NavAgentProperties.AgentRadius = 100.0f;
	bUpdatePathAfterMoving = true;
}

FBoxSphereBounds ANav3DRaycasterTest::GetBoundingBoxContainingOtherActorAndMe() const
{
	TArray<FVector> Points;
	Points.Reserve(2);

	Points.Add(GetActorLocation());

	if (OtherActor != nullptr)
	{
		Points.Add(OtherActor->GetActorLocation());
	}

	return FBoxSphereBounds(FBox(Points));
}

#if WITH_EDITOR
void ANav3DRaycasterTest::PreEditChange(FProperty* PropertyAboutToChange)
{
	static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED(ANav3DRaycasterTest, OtherActor);

	if (PropertyAboutToChange != nullptr &&
		PropertyAboutToChange->GetFName() == NAME_OtherActor &&
		OtherActor != nullptr && OtherActor->OtherActor == this)
	{
		OtherActor->OtherActor = nullptr;
#if WITH_EDITORONLY_DATA
		OtherActor->RenderingComponent->MarkRenderStateDirty();
		RenderingComponent->MarkRenderStateDirty();
#endif
	}

	Super::PreEditChange(PropertyAboutToChange);
}

void ANav3DRaycasterTest::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED(ANav3DRaycasterTest, OtherActor);
	static const FName NAME_UpdatePathAfterMoving = GET_MEMBER_NAME_CHECKED(ANav3DRaycasterTest, bUpdatePathAfterMoving);

	if (PropertyChangedEvent.Property != nullptr)
	{
		if (const FName PropertyName = PropertyChangedEvent.MemberProperty->GetFName(); PropertyName == NAME_OtherActor)
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

void ANav3DRaycasterTest::PostEditMove(const bool IsFinished)
{
	Super::PostEditMove(IsFinished);

	if (OtherActor != nullptr)
	{
		if (bUpdatePathAfterMoving)
		{
			DoRaycast();
		}
		else if (OtherActor->bUpdatePathAfterMoving)
		{
			OtherActor->DoRaycast();
		}
	}
}
#endif

void ANav3DRaycasterTest::BeginDestroy()
{
	if (OtherActor != nullptr && OtherActor->OtherActor == this)
	{
		OtherActor->OtherActor = nullptr;
	}

	Super::BeginDestroy();
}

void ANav3DRaycasterTest::UpdateDrawing() const
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

void ANav3DRaycasterTest::DoRaycast()
{
	if (OtherActor == nullptr)
	{
		return;
	}

	if (Raycaster == nullptr)
	{
		return;
	}

	if (auto* NavigationSystem = UNavigationSystemV1::GetCurrent(this))
	{
		if (auto* NavigationData = NavigationSystem->GetNavDataForProps(NavAgentProperties))
		{
			if (const auto* N3dNavigationData = Cast<ANav3DData>(NavigationData))
			{
				const auto From = GetActorLocation();
				const auto To = OtherActor->GetActorLocation();

				if (const auto* VolumeNavigationData =
					N3dNavigationData->GetVolumeNavigationDataContainingPoints({From, To}))
				{
					Raycaster->SetProcessor(
						MakeShared<FNav3DRaycasterProcessor_GenerateDebugInfos>(
							RaycasterDebugInfos));
					auto _ = Raycaster->Trace(*VolumeNavigationData, From, To);
				}
			}
		}
	}

	UpdateDrawing();
}
