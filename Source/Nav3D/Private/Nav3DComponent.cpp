#include "Nav3DComponent.h"
#include "Nav3DFindPathTask.h"
#include "Nav3DStructs.h"
#include "Nav3DVolume.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"

UNav3DComponent::UNav3DComponent(const FObjectInitializer& ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	Nav3DPath = MakeShareable<FNav3DPath>(new FNav3DPath());
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
	FFindPathTaskCompleteDynamicDelegate OnComplete,
	ENav3DPathFindingCallResult &Result) {
	
	FNav3DOctreeEdge StartEdge;
	FNav3DOctreeEdge TargetEdge;

	// Error checking before task start
	if (!VolumeContainsOctree() || !VolumeContainsOwner()) FindVolume();
	if (!VolumeContainsOwner()) {
		Result = ENav3DPathFindingCallResult::NoVolume;
		UE_LOG(LogTemp, Error, TEXT("Pathfinding cannot initialise. Nav3D component owner is not inside a Nav3D volume"));
		return;
	}
	
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
		UE_LOG(LogTemp, Error, TEXT("Pathfinding unnecessary. Nav3D component owner has a clear line of sight to target"));
		return;
	}

	// Check that an octree has been found
	if (!VolumeContainsOctree()) {
		Result = ENav3DPathFindingCallResult::NoOctree;
		UE_LOG(LogTemp, Error, TEXT("Pathfinding cannot initialise. Nav3D octree has not been built"));
		return;
	}

	if (!Volume->GetEdge(StartLocation, StartEdge))
	{
		Result = ENav3DPathFindingCallResult::NoStart;
		UE_LOG(LogTemp, Error, TEXT("Failed to find start edge"));
		return;
	}
	
	if (!Volume->GetEdge(TargetLocation, TargetEdge))
	{
		Result = ENav3DPathFindingCallResult::NoTarget;
		UE_LOG(LogTemp, Error, TEXT("Failed to find target edge"));
		return;
	}

	FNav3DPathFindingConfig Config;
	Config.Heuristic = Heuristic;
	Config.EstimateWeight = HeuristicWeight;
	Config.NodeSizePreference = NodeSizePreference;
	Config.PathPruning = PathPruning;
	Config.PathSmoothing = PathSmoothing;
	FNav3DPath& Path = *Nav3DPath;

	(new FAutoDeleteAsyncTask<FNav3DFindPathTask>(this, StartEdge, TargetEdge, StartLocation, TargetLocation, Config, Path, OnComplete))->StartBackgroundTask();
	Result = ENav3DPathFindingCallResult::Success;
	UE_LOG(LogTemp, Display, TEXT("Pathfinding task called successfully"));
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
				Volume->GetEdgeLocation(CurrentEdge, PathPoint.PointLocation);
				Path.Points.Add(PathPoint);
				const FNav3DOctreeNode& Node = Volume->GetNode(CurrentEdge);
				if (CurrentEdge.GetLayerIndex() == 0) {
					if (!Node.HasChildren()) Path.Points[Path.Points.Num() - 1].PointLayer = 1;
					else Path.Points[Path.Points.Num() - 1].PointLayer = 0;
				} else {
					Path.Points[Path.Points.Num() - 1].PointLayer = CurrentEdge.GetLayerIndex() + 1;
				}
			}

			if (Path.Points.Num() > 1) {
				Path.Points[0].PointLocation = TargetLocation;
				Path.Points[Path.Points.Num() - 1].PointLocation = StartLocation;
			} else {
				if (Path.Points.Num() == 0) Path.Points.Emplace();
				Path.Points[0].PointLocation = TargetLocation;
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
					float Cost = 1.0f - static_cast<float>(TargetEdge.GetLayerIndex()) / static_cast<float>(Volume->NumLayers) * Config.NodeSizePreference;
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
	UE_LOG(LogTemp, Display, TEXT("Pathfinding failed, iterations : %i"), I);
}

float UNav3DComponent::HeuristicScore(const FNav3DOctreeEdge StartEdge, const FNav3DOctreeEdge TargetEdge, const FNav3DPathFindingConfig Config) const
{
	FVector StartLocation, TargetLocation;
	Volume->GetEdgeLocation(StartEdge, StartLocation);
	Volume->GetEdgeLocation(TargetEdge, TargetLocation);
	if (Config.Heuristic == ENav3DHeuristic::Manhattan) {
		return FMath::Abs(TargetLocation.X - StartLocation.X) + FMath::Abs(TargetLocation.Y - StartLocation.Y) + FMath::Abs(TargetLocation.Z - StartLocation.Z);	
	}
	
	return (StartLocation - TargetLocation).Size() * (1.0f - (static_cast<float>(TargetEdge.LayerIndex) / static_cast<float>(Volume->NumLayers)) * Config.NodeSizePreference);	
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
			FVector Start = Path.Points[CurrentPoint].PointLocation;
			FVector End = Path.Points[I+2].PointLocation;

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
	PathPoints.Insert(Path.Points[0].PointLocation, 0);
	PathPoints.Add(Path.Points[Path.Points.Num() - 1].PointLocation);
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
			SplinePoints.Add(FNav3DPathPoint(0.5f * (A + (B * T) + (C * T * T) + (D * T * T * T)), Path.GetPoints()[Index - 1].PointLayer));
		}
	}

	// Add the final path point to the spline
	SplinePoints.Add(Path.Points[Path.Points.Num()-1]);
	Path = SplinePoints;
}

#if WITH_EDITOR

void UNav3DComponent::RequestNavPathDebugDraw(const FNav3DPath& Path) const {
	if (!GetWorld() || !Volume || Path.Points.Num() < 2 || !bDebugDrawNavPath) return;
	FNav3DDebugPath DebugPath;
	for (auto& Point: Path.Points) DebugPath.Points.Add(Point.PointLocation);
	DebugPath.Colour = DebugPathColour;
	DebugPath.LineScale = DebugPathLineScale;
	Volume->AddDebugNavPath(DebugPath);
}

#endif