#include "Pathfinding/Core/Nav3DPathTest.h"
#include "Nav3DData.h"
#include "Pathfinding/Core/Nav3DPathCoordinator.h"
#include "Pathfinding/Core/Nav3DPath.h"
#include "Nav3DUtils.h"
#include "Nav3D.h"
#include "Components/SphereComponent.h"
#include "Pathfinding/Utils/Nav3DPathTestRenderingComponent.h"

#if WITH_EDITOR
#include "Editor.h"
#endif

ANav3DPathTest::ANav3DPathTest()
{
	PrimaryActorTick.bCanEverTick = false;
	PrimaryActorTick.bStartWithTickEnabled = false;

    Sphere = CreateDefaultSubobject<USphereComponent>(TEXT("Sphere"));
    Sphere->InitSphereRadius(500.0f);
    Sphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Sphere->SetGenerateOverlapEvents(false);
    RootComponent = Sphere;

#if WITH_EDITORONLY_DATA
    RenderingComponent = CreateEditorOnlyDefaultSubobject<UNav3DPathTestRenderingComponent>(TEXT("RenderingComponent"));
    if (RenderingComponent != nullptr)
    {
        RenderingComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    }
#endif

	NavAgentProperties = FNavAgentProperties::DefaultProperties;
	bUpdatePathAfterMoving = false;
	LastResult = ENavigationQueryResult::Invalid;
}

void ANav3DPathTest::BeginPlay()
{
	Super::BeginPlay();
    UpdateReciprocalLink();
}

#if WITH_EDITOR
void ANav3DPathTest::PreEditChange(FProperty* PropertyAboutToChange)
{
	static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED(ANav3DPathTest, OtherActor);

	if (PropertyAboutToChange != nullptr &&
		PropertyAboutToChange->GetFName() == NAME_OtherActor &&
		OtherActor != nullptr && OtherActor->OtherActor == this)
	{
		OtherActor->OtherActor = nullptr;
		OtherActor->LastPath.ResetForRepath();
		LastPath.ResetForRepath();
#if WITH_EDITORONLY_DATA
		OtherActor->RenderingComponent->MarkRenderStateDirty();
		RenderingComponent->MarkRenderStateDirty();
#endif
	}

	Super::PreEditChange(PropertyAboutToChange);
}

void ANav3DPathTest::PostEditChangeProperty(
	FPropertyChangedEvent& PropertyChangedEvent)
{
	static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED(ANav3DPathTest, OtherActor);
	static const FName NAME_UpdatePathAfterMoving = GET_MEMBER_NAME_CHECKED(
		ANav3DPathTest, bUpdatePathAfterMoving);

	if (PropertyChangedEvent.Property != nullptr)
	{
		const FName PropertyName = PropertyChangedEvent.MemberProperty->GetFName();
		if (PropertyName == NAME_OtherActor)
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
					OtherActorsOldOtherActor->LastPath.ResetForRepath();
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

void ANav3DPathTest::PostEditMove(const bool IsFinished)
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

void ANav3DPathTest::BeginDestroy()
{
	LastPath.ResetForRepath();

	if (OtherActor != nullptr && OtherActor->OtherActor == this)
	{
		OtherActor->OtherActor = nullptr;
		OtherActor->LastPath.ResetForRepath();
	}

	Super::BeginDestroy();
}

void ANav3DPathTest::UpdateDrawing() const
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

void ANav3DPathTest::FindPath()
{
	if (LastPath.IsValid())
	{
		ClearPath();
	}
	TryUpdatePath();
	UpdateDrawing();
}

void ANav3DPathTest::TryUpdatePath()
{
	if (OtherActor == nullptr)
	{
		UE_LOG(LogNav3D, Warning, TEXT("FindPath: No OtherActor set"));
		return;
	}
	
	UWorld* World = GetWorld();
	
	// Find Nav3DData directly using utility function
	// Nav3DData supports multiple agent sizes through hierarchical layers, so we don't need exact property matching
	// The agent properties used to build the Nav3DData are automatically used
	const ANav3DData* Nav3dData = FNav3DUtils::GetNav3DData(World);
	if (!Nav3dData)
	{
		UE_LOG(LogNav3D, Error, TEXT("FindPath: No Nav3DData found in navigation system. Make sure Nav3D data has been built."));
		return;
	}
	
	FSharedConstNavQueryFilter QueryFilter = FNav3DUtils::GetNav3DQueryFilter(
		Nav3dData, nullptr, this);
	
	FNav3DPathingRequest Request;
    const AActor* EffectiveStart = this;
    const AActor* EffectiveEnd = OtherActor;
	
    if (!EffectiveEnd)
    {
        LastResult = ENavigationQueryResult::Invalid;
        LastPath.ResetForRepath();
        return;
    }

    Request.StartLocation = EffectiveStart->GetActorLocation();
    Request.EndLocation = EffectiveEnd->GetActorLocation();
	Request.NavData = Nav3dData;
	// Use agent properties from Nav3DData (which were used to build it)
	// Nav3DData automatically supports multiple agent sizes through hierarchical layers
	Request.AgentProperties = Nav3dData->GetNavAgentProperties();
	
	// If custom agent properties are specified and valid, use them (for testing different agent sizes)
	// This allows testing pathfinding for agents larger/smaller than the base agent
	if (NavAgentProperties.IsValid())
	{
		Request.AgentProperties = NavAgentProperties;
	}
	
	Request.Algorithm = Algorithm;
    Request.LogVerbosity = ENav3DPathingLogVerbosity::Standard;

    LastRequestedStart = Request.StartLocation;
    LastRequestedEnd = Request.EndLocation;
    LastPath.ResetForRepath();
    LastResult = FNav3DPathCoordinator::FindPath(LastPath, Request);
    bLastPathReachedTarget = false;
    if (LastResult == ENavigationQueryResult::Success)
    {
        const TArray<FNavPathPoint>& Pts = LastPath.GetPathPoints();
        if (Pts.Num() > 0)
        {
            const FVector End = Pts.Last().Location;
            bLastPathReachedTarget = End.Equals(LastRequestedEnd, 1.0f);
        }
    }

#if WITH_EDITORONLY_DATA
    if (RenderingComponent)
    {
        RenderingComponent->MarkRenderStateDirty();
    }
#endif
}


void ANav3DPathTest::UpdateReciprocalLink()
{
    if (OtherActor)
    {
        if (OtherActor->OtherActor != this)
        {
            OtherActor->OtherActor = this;
        }
    }
}

void ANav3DPathTest::ClearPath()
{
    LastPath.ResetForRepath();
    LastResult = ENavigationQueryResult::Invalid;
    if (OtherActor && OtherActor->OtherActor == this)
    {
        OtherActor->LastPath.ResetForRepath();
        OtherActor->LastResult = ENavigationQueryResult::Invalid;
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
