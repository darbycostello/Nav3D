#include "Nav3DComponent.h"
#include "Nav3DFindCoverTask.h"
#include "Nav3DFindPathTask.h"
#include "Nav3DStructs.h"
#include "Nav3DVolume.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"

UNav3DComponent::UNav3DComponent(const FObjectInitializer& ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	Nav3DPath = MakeShareable<FNav3DPath>(new FNav3DPath());
	Nav3DCoverLocation = MakeShareable<FNav3DCoverLocation>(new FNav3DCoverLocation());
	bWantsInitializeComponent = true;
}

void UNav3DComponent::BeginPlay()
{
	Super::BeginPlay();
}

bool UNav3DComponent::VolumeContainsOctree() const
{	
	return Volume && Volume->OctreeValid();
}

bool UNav3DComponent::VolumeContainsOwner() const
{	
	return GetOwner() && Volume->IsWithinBounds(GetOwner()->GetActorLocation());
}

bool UNav3DComponent::VolumeCoverMapEnabled() const
{	
	return Volume->bEnableCoverMap;
}

bool UNav3DComponent::VolumeCoverMapExists() const
{	
	return Volume->CoverMapValid();
}

bool UNav3DComponent::FindVolume()
{
	TArray<AActor*> Volumes;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANav3DVolume::StaticClass(), Volumes);

	for (auto& NavVolume : Volumes)
	{
		ANav3DVolume* CurrentVolume = Cast<ANav3DVolume>(NavVolume);
		if (CurrentVolume) {
			
			if (CurrentVolume->IsWithinBounds(GetOwner()->GetActorLocation())) {
				Volume = CurrentVolume;
				return true;
			}
		}
	}
	return false;
}

void UNav3DComponent::TickComponent(const float DeltaTime, const ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UNav3DComponent::FindPath(
	const FVector& StartLocation,
	const FVector& TargetLocation,
	const bool bCheckLineOfSight,
	FFindPathTaskCompleteDynamicDelegate OnComplete,
	ENav3DPathFindingCallResult &Result) {
	
	FNav3DOctreeEdge StartEdge;
	FNav3DOctreeEdge TargetEdge;
	FVector LegalStart = StartLocation;
	FVector LegalTarget = TargetLocation;

	// Error checking before task start
	if (!VolumeContainsOctree() || !VolumeContainsOwner()) FindVolume();
	if (!VolumeContainsOwner()) {
		Result = ENav3DPathFindingCallResult::NoVolume;
		
#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: Pathfinding cannot initialise. Nav3D component owner is not inside a Nav3D volume"), *GetOwner()->GetName());
#endif

		return;
	}

	if (bCheckLineOfSight) {
		// If there is a line of sight plus clearance with the target then no path finding is required
		FCollisionQueryParams CollisionQueryParams;
		CollisionQueryParams.bTraceComplex = true;
		CollisionQueryParams.TraceTag = "Nav3DLineOfSightCheck";
		FHitResult HitResult;
		float Radius = FMath::Max(50.f ,GetOwner()->GetComponentsBoundingBox(true).GetExtent().GetMax());
		GetWorld()->SweepSingleByChannel(
            HitResult,
            StartLocation,
            TargetLocation,
            FQuat::Identity,
            Volume->CollisionChannel,
            FCollisionShape::MakeSphere(Radius),
            CollisionQueryParams
        );

		if (!HitResult.bBlockingHit) {
			Result = ENav3DPathFindingCallResult::Reachable;
			
#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: Pathfinding unnecessary. Nav3D component owner has a clear line of sight to target"), *GetOwner()->GetName());
#endif
			
			return;
		}	
	}
	
	// Check that an octree has been found
	if (!VolumeContainsOctree()) {
		Result = ENav3DPathFindingCallResult::NoOctree;
		
#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: Pathfinding cannot initialise. Nav3D octree has not been built"), *GetOwner()->GetName());
#endif
		
		return;
	}
	
	if (!Volume->GetEdge(StartLocation, StartEdge))
	{
		Result = ENav3DPathFindingCallResult::NoStart;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Warning, TEXT("%s: Failed to find start edge. Searching nearby..."), *GetOwner()->GetName());
#endif
		
		if (!Volume->FindAccessibleEdge(LegalStart, StartEdge)) {

#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: No accessible adjacent edge found"), *GetOwner()->GetName());
#endif
			
			return;	
		}

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Found legal start location"), *GetOwner()->GetName());
#endif
		
	}

	if (!Volume->GetEdge(TargetLocation, TargetEdge))
	{
		Result = ENav3DPathFindingCallResult::NoTarget;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Warning, TEXT("%s: Failed to find target edge. Searching nearby..."), *GetOwner()->GetName());
#endif
		
		if (!Volume->FindAccessibleEdge(LegalTarget, TargetEdge)) {

#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: No accessible edges found near target"), *GetOwner()->GetName());
#endif
			
			return;	
		}

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Found accessible target location"), *GetOwner()->GetName());
#endif
		
	}

	FNav3DPathFindingConfig Config;
	Config.Heuristic = Heuristic;
	Config.EstimateWeight = HeuristicWeight;
	Config.NodeSizePreference = NodeSizePreference;
	Config.PathPruning = PathPruning;
	Config.PathSmoothing = PathSmoothing;
	FNav3DPath& Path = *Nav3DPath;

	(new FAutoDeleteAsyncTask<FNav3DFindPathTask>(
		this,
		StartEdge,
		TargetEdge,
		LegalStart,
		LegalTarget,
		Config,
		Path,
		OnComplete))->StartBackgroundTask();
	Result = ENav3DPathFindingCallResult::Success;

#if WITH_EDITOR
	if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Pathfinding task called successfully"), *GetOwner()->GetName());
#endif

}

void UNav3DComponent::FindCover(
	FVector SearchOrigin,
	float MaxRadius,
	AActor* Target,
	ENav3DCoverSearchType SearchType,
	FFindCoverTaskCompleteDynamicDelegate OnComplete,
	ENav3DFindCoverCallResult& Result) {
	
	TArray<AActor*> TargetActors = {Target};

	// Error checking before task start
	if (!VolumeContainsOctree() || !VolumeContainsOwner()) FindVolume();
	if (!VolumeContainsOwner()) {
		Result = ENav3DFindCoverCallResult::NoVolume;
		
#if WITH_EDITOR
		if (bDebugFindCover) UE_LOG(LogTemp, Error, TEXT("%s: Find cover cannot initialise. Nav3D component owner is not inside a Nav3D volume"), *GetOwner()->GetName());
#endif

		return;
	}
	if (!VolumeCoverMapEnabled()) {
		Result = ENav3DFindCoverCallResult::CoverMapNotEnabled;
		
#if WITH_EDITOR
		if (bDebugFindCover) UE_LOG(LogTemp, Error, TEXT("%s: Find cover cannot initialise. Nav3D volume cover map is not enabled"), *GetOwner()->GetName());
#endif

		return;
	}
	if (!VolumeCoverMapExists()) {
		Result = ENav3DFindCoverCallResult::CoverMapInvalid;
		
#if WITH_EDITOR
		if (bDebugFindCover) UE_LOG(LogTemp, Error, TEXT("%s: Find cover cannot initialise. Nav3D volume cover map has no entried or is not valid"), *GetOwner()->GetName());
#endif

		return;
	}

	TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypes;
	ObjectTypes.Add(UEngineTypes::ConvertToObjectType(Volume->CollisionChannel));
	FNav3DCoverLocation& CoverLocation = *Nav3DCoverLocation;
	(new FAutoDeleteAsyncTask<FNav3DFindCoverTask>(
		this,
		SearchOrigin,
		MaxRadius,
		TargetActors,
		ObjectTypes,
		SearchType,
		CoverLocation,
		OnComplete))->StartBackgroundTask();

	Result = ENav3DFindCoverCallResult::Success;
}

void UNav3DComponent::FindCoverMultiple(
	FVector SearchOrigin,
	float MaxRadius,
	TArray<AActor*> Opponents,
	ENav3DCoverSearchType SearchType,
	FFindCoverTaskCompleteDynamicDelegate OnComplete,
	ENav3DFindCoverCallResult& Result) {
	
	// Error checking before task start
	if (!VolumeContainsOctree() || !VolumeContainsOwner()) FindVolume();
	if (!VolumeContainsOwner()) {
		Result = ENav3DFindCoverCallResult::NoVolume;
		
#if WITH_EDITOR
		if (bDebugFindCover) UE_LOG(LogTemp, Error, TEXT("%s: Find cover cannot initialise. Nav3D component owner is not inside a Nav3D volume"), *GetOwner()->GetName());
#endif

		return;
	}
	if (!VolumeCoverMapEnabled()) {
		Result = ENav3DFindCoverCallResult::CoverMapNotEnabled;
		
#if WITH_EDITOR
		if (bDebugFindCover) UE_LOG(LogTemp, Error, TEXT("%s: Find cover cannot initialise. Nav3D volume cover map is not enabled"), *GetOwner()->GetName());
#endif

		return;
	}
	if (!VolumeCoverMapExists()) {
		Result = ENav3DFindCoverCallResult::CoverMapInvalid;
		
#if WITH_EDITOR
		if (bDebugFindCover) UE_LOG(LogTemp, Error, TEXT("%s: Find cover cannot initialise. Nav3D volume cover map has no entried or is not valid"), *GetOwner()->GetName());
#endif

		return;
	}

	TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypes;
	ObjectTypes.Add(UEngineTypes::ConvertToObjectType(Volume->CollisionChannel));
	FNav3DCoverLocation& CoverLocation = *Nav3DCoverLocation;
	(new FAutoDeleteAsyncTask<FNav3DFindCoverTask>(
		this,
		SearchOrigin,
		MaxRadius,
		Opponents,
		ObjectTypes,
		SearchType,
		CoverLocation,
		OnComplete))->StartBackgroundTask();

	Result = ENav3DFindCoverCallResult::Success;
}

void UNav3DComponent::ExecuteFindCover(
	const FVector Location,
	const float Radius,
	const TArray<AActor*> Opponents,
	const TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypes,
	const ENav3DCoverSearchType SearchType,
	FNav3DCoverLocation& CoverLocation) const {

	CoverLocation = FNav3DCoverLocation();
	if (Opponents.Num() == 0) return;
	if (!Opponents[0]) return;
	
	TArray<UPrimitiveComponent*> OverlappedComponents;
	UKismetSystemLibrary::SphereOverlapComponents(
		GetWorld(),
        Location,
        Radius,
        ObjectTypes,
        UPrimitiveComponent::StaticClass(),
        {},
        OverlappedComponents);
	
	if (OverlappedComponents.Num() == 0) return;
	FVector OpponentLocation = Opponents[0]->GetActorLocation();
	// For multiple opponents, take the median location of them
	if (Opponents.Num() > 1) {
		for (int32 I = 1; I < Opponents.Num(); I++) {
			if (Opponents[I]) OpponentLocation += Opponents[I]->GetActorLocation() * 0.5f;
		}
	}
	
	TArray<FNav3DCoverLocation> CoverLocations;
	TArray<float> Distances;
	for (auto& Component: OverlappedComponents) {
		if (!IsValid(Component)) continue;
		const FName ActorName = Component->GetOwner()->GetFName();
		if (!Volume->CoverMapContainsActor(ActorName)) continue;
		
		FVector CollisionPoint;
		Component->GetClosestPointOnCollision(OpponentLocation, CollisionPoint);
		const int32 NormalIndex = Volume->GetCoverNormalIndex((CollisionPoint - OpponentLocation).GetSafeNormal());
		
		TArray<FVector> FoundLocations = Volume->GetCoverMapNodeLocations(ActorName, NormalIndex);
		for (auto& FoundLocation: FoundLocations) {
			CoverLocations.Add(FNav3DCoverLocation(Component->GetOwner(), FoundLocation, NormalIndex));
			Distances.Add(FVector::DistSquared(FoundLocation, GetOwner()->GetActorLocation()));
		}
	}
	if (Distances.Num() == 0) return;
	int32 Index;
	float SquareDistance;
	switch (SearchType) {
		default: UKismetMathLibrary::MinOfFloatArray(Distances, Index, SquareDistance); break;
		case ENav3DCoverSearchType::Furthest: UKismetMathLibrary::MaxOfFloatArray(Distances, Index, SquareDistance); break;
		case ENav3DCoverSearchType::Random: Index = UKismetMathLibrary::RandomIntegerInRange(0, Distances.Num()-1); break;
	}
	if (Index != -1) CoverLocation = CoverLocations[Index];
}

void UNav3DComponent::ExecutePathFinding(
	const FNav3DOctreeEdge& StartEdge,
	const FNav3DOctreeEdge& TargetEdge,
	const FVector& StartLocation,
	const FVector& TargetLocation,
	FNav3DPathFindingConfig Config,
	FNav3DPath& Path) {

	// Initialise
	TSet<FNav3DOctreeEdge> OpenSet;
	TSet<FNav3DOctreeEdge> ClosedSet;
	TMap<FNav3DOctreeEdge, FNav3DOctreeEdge> Parent;
	TMap<FNav3DOctreeEdge, float> G;
	TMap<FNav3DOctreeEdge, float> F;
	FNav3DOctreeEdge CurrentEdge = FNav3DOctreeEdge();
	Path.Empty();

	// Greedy A*
	OpenSet.Add(StartEdge);
	Parent.Add(StartEdge, StartEdge);
	G.Add(StartEdge, 0);
	F.Add(StartEdge, HeuristicScore(StartEdge, TargetEdge, Config));
	int32 I = 0;
	while (OpenSet.Num() > 0) {
		float LowestScore = FLT_MAX;
		for (FNav3DOctreeEdge& Edge : OpenSet) {
			if (!F.Contains(Edge) || F[Edge] < LowestScore) {
				LowestScore = F[Edge];
				CurrentEdge = Edge;
			}
		}
		OpenSet.Remove(CurrentEdge);
		ClosedSet.Add(CurrentEdge);

		if (CurrentEdge.NodeIndex == TargetEdge.NodeIndex) {
			FNav3DPathPoint PathPoint;

			while (Parent.Contains(CurrentEdge) && !(CurrentEdge == Parent[CurrentEdge])) {
				CurrentEdge = Parent[CurrentEdge];
				Volume->GetEdgeLocation(CurrentEdge, PathPoint.Location);
				Path.Points.Add(PathPoint);
				const FNav3DOctreeNode& Node = Volume->GetNode(CurrentEdge);
				if (CurrentEdge.GetLayerIndex() == 0) {
					if (!Node.HasChildren()) Path.Points[Path.Points.Num() - 1].Layer = 1;
					else Path.Points[Path.Points.Num() - 1].Layer = 0;
				} else {
					Path.Points[Path.Points.Num() - 1].Layer = CurrentEdge.GetLayerIndex() + 1;
				}
			}

			if (Path.Points.Num() > 1) {
				Path.Points[0].Location = TargetLocation;
				Path.Points[Path.Points.Num() - 1].Location = StartLocation;
			} else {
				if (Path.Points.Num() == 0) Path.Points.Emplace();
				Path.Points[0].Location = TargetLocation;
				Path.Points.Emplace(StartLocation, StartEdge.GetLayerIndex());
			}
			return;
		}

		const FNav3DOctreeNode& CurrentNode = Volume->GetNode(CurrentEdge);
		TArray<FNav3DOctreeEdge> AdjacentEdges;

		if (CurrentEdge.GetLayerIndex() == 0 && CurrentNode.FirstChild.IsValid()) {
			Volume->GetAdjacentLeafs(CurrentEdge, AdjacentEdges);
		} else {
			Volume->GetAdjacentEdges(CurrentEdge, AdjacentEdges);
		}

		for (auto& AdjacentEdge : AdjacentEdges) {
			if (AdjacentEdge.IsValid()) {
				if (ClosedSet.Contains(AdjacentEdge)) {
					continue;
				}
				if (!OpenSet.Contains(AdjacentEdge)) {
					OpenSet.Add(AdjacentEdge);
				}

				float GScore = FLT_MAX;
				if (G.Contains(CurrentEdge))
				{
					FVector CurrentLocation(0.f), AdjacentLocation(0.f);
					Volume->GetEdgeLocation(CurrentEdge, CurrentLocation);
					Volume->GetEdgeLocation(AdjacentEdge, AdjacentLocation);
					float Cost;
					Volume->GetPathCost(AdjacentLocation, Cost);
					Cost -= static_cast<float>(TargetEdge.GetLayerIndex()) / static_cast<float>(Volume->NumLayers) * Config.NodeSizePreference;
					if (Config.Heuristic == ENav3DHeuristic::Euclidean) {
						Cost *= (CurrentLocation - AdjacentLocation).Size();
					}
					GScore = G[CurrentEdge] + Cost;
				}
				else {
					G.Add(CurrentEdge, FLT_MAX);
				}

				if (GScore >= (G.Contains(AdjacentEdge) ? G[AdjacentEdge] : FLT_MAX)) continue;
				Parent.Add(AdjacentEdge, CurrentEdge);
				G.Add(AdjacentEdge, GScore);
				// Greedy A* multiplies the heuristic score by the estimate weight
				F.Add(AdjacentEdge, G[AdjacentEdge] + Config.EstimateWeight * HeuristicScore(AdjacentEdge, TargetEdge, Config));
			}
		}
		I++;
	}
	
#if WITH_EDITOR
	if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: Pathfinding failed after %i iterations"), *GetOwner()->GetName(), I);
#endif

}

float UNav3DComponent::HeuristicScore(const FNav3DOctreeEdge StartEdge, const FNav3DOctreeEdge TargetEdge, const FNav3DPathFindingConfig Config) const
{
	FVector StartLocation, TargetLocation;
	Volume->GetEdgeLocation(StartEdge, StartLocation);
	Volume->GetEdgeLocation(TargetEdge, TargetLocation);

	// Divide euclidean distance into ten steps then sample points for modifier volume path cost updates
	const FVector UnitStep = (TargetLocation - StartLocation).GetSafeNormal() * (StartLocation - TargetLocation).Size() * 0.1;
	float Cost = 0.f;
	for (int32 I = 0; I < 10; I++) {
		FVector Point = StartLocation + UnitStep * I;
		for (auto& Modifier: Volume->GetModifierVolumes()) {
			if (!Modifier) continue; 
			if (Modifier->GetBoundingBox().IsInside(Point)) {
				Cost += Modifier->GetPathCost() * UnitStep.Size();
			} 
		}
	}
	if (Config.Heuristic == ENav3DHeuristic::Manhattan) {
		return Cost + FMath::Abs(TargetLocation.X - StartLocation.X) + FMath::Abs(TargetLocation.Y - StartLocation.Y) + FMath::Abs(TargetLocation.Z - StartLocation.Z);	
	}
	return ((StartLocation - TargetLocation).Size() + Cost) * (1.0f - (static_cast<float>(TargetEdge.LayerIndex) / static_cast<float>(Volume->NumLayers)) * Config.NodeSizePreference);	
}

void UNav3DComponent::AddPathStartLocation(FNav3DPath& Path) const {
	if (Path.Points.Num() == 0) return;
	const FVector Location = GetOwner()->GetActorLocation();
	if (!Location.Equals(Path.Points[Path.Points.Num() - 1].Location), 50.0f) {
		Path.Points.Add(FNav3DPathPoint(Location, Path.Points[Path.Points.Num() - 1].Layer));
	}
}

void UNav3DComponent::ApplyPathPruning(FNav3DPath& Path, const FNav3DPathFindingConfig Config) const {
	if (!GetWorld() || Config.PathPruning == ENav3DPathPruning::None || Path.Points.Num() < 3) return;
	FNav3DPath PrunedPath;
	PrunedPath.Add(Path.Points[0]);
	int32 CurrentPoint = 0;
	float Radius = 0;
	if (Config.PathPruning == ENav3DPathPruning::WithClearance) {
		Radius = FMath::Max(50.f ,GetOwner()->GetComponentsBoundingBox(true).GetExtent().GetMax());
	}
	while (CurrentPoint < Path.Points.Num()) {

		for (int32 I = CurrentPoint; I < Path.Points.Num(); I++) {
			if (I >= Path.Points.Num() - 2) {
				PrunedPath.Add(Path.Points[Path.Points.Num() - 1]);
				CurrentPoint = Path.Points.Num();
				break;
			}
			FCollisionQueryParams CollisionQueryParams;
			CollisionQueryParams.bTraceComplex = true;
			CollisionQueryParams.TraceTag = "Nav3DPathPrune";
			FHitResult HitResult;
			FVector Start = Path.Points[CurrentPoint].Location;
			FVector End = Path.Points[I+2].Location;

			if (Config.PathPruning == ENav3DPathPruning::WithClearance) {
				GetWorld()->SweepSingleByChannel(
	                HitResult,
	                Start,
	                End,
	                FQuat::Identity,
	                Volume->CollisionChannel,
	                FCollisionShape::MakeSphere(Radius),
	                CollisionQueryParams
	            );
			} else {
				GetWorld()->LineTraceSingleByChannel(
					HitResult,
	                Start,
	                End,
	                Volume->CollisionChannel,
	                CollisionQueryParams
				);
			}
			
			if (HitResult.bBlockingHit) {
				PrunedPath.Add(Path.Points[I + 1]);
				CurrentPoint = I + 1;
				break;
			}
		}
	}
	Path = PrunedPath;
}

// Apply Catmull-Rom smoothing to the path
void UNav3DComponent::ApplyPathSmoothing(FNav3DPath& Path, const FNav3DPathFindingConfig Config) {
	if (Config.PathSmoothing < 1 || Path.Points.Num() < 3) return;
	TArray<FVector> PathPoints;
	Path.GetPath(PathPoints);
	
	// Duplicate the start and end points to ensure a smooth curve
	PathPoints.Insert(Path.Points[0].Location, 0);
	PathPoints.Add(Path.Points[Path.Points.Num() - 1].Location);
	FNav3DPath SplinePoints;

	// Add the first path point to the spline
	SplinePoints.Add(Path.Points[0]);
	for (int32 Index = 0; Index < PathPoints.Num(); Index++) {
		if (Index == 0 || Index == PathPoints.Num() - 2 || Index == PathPoints.Num() - 1) continue;
		FVector P0 = PathPoints[Index - 1];
		FVector P1 = PathPoints[Index];
		FVector P2 = PathPoints[Index + 1];
		FVector P3 = PathPoints[Index + 2];
		for (int I = 1; I <= Config.PathSmoothing; I++) {
			const float T = I * (1.f / Config.PathSmoothing);
			const FVector A = 2.f * P1;
			const FVector B = P2 - P0;
			const FVector C = 2.f * P0 - 5.f * P1 + 4.f * P2 - P3;
			const FVector D = -P0 + 3.f * P1 - 3.f * P2 + P3;
			SplinePoints.Add(FNav3DPathPoint(0.5f * (A + (B * T) + (C * T * T) + (D * T * T * T)), Path.GetPoints()[Index - 1].Layer));
		}
	}

	// Add the final path point to the spline
	SplinePoints.Add(Path.Points[Path.Points.Num()-1]);
	Path = SplinePoints;
}

#if WITH_EDITOR

void UNav3DComponent::RequestNavPathDebugDraw(const FNav3DPath Path) const {
	if (!GetWorld() || !Volume || Path.Points.Num() < 2 || !bDebugDrawNavPath) return;
	FNav3DDebugPath DebugPath;
	for (auto& Point: Path.Points) DebugPath.Points.Add(Point.Location);
	DebugPath.Colour = DebugPathColour;
	DebugPath.LineScale = DebugPathLineScale;
	Volume->AddDebugNavPath(DebugPath);
}

#endif