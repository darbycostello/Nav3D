#include "Nav3DComponent.h"
#include "Nav3DFindPathTask.h"
#include "Nav3DStructs.h"
#include "Nav3DVolume.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"

UNav3DComponent::UNav3DComponent(const FObjectInitializer& ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	Nav3DPath = MakeShareable<FNav3DPath>(new FNav3DPath());
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
	ENav3DPathFindingCallResult &Result)
{
	FNav3DOctreeEdge StartEdge;
	FNav3DOctreeEdge TargetEdge;

	// Error checking before task start
	if (!VolumeContainsOctree() || !VolumeContainsOwner()) FindVolume();
	if (!VolumeContainsOwner()) {
		Result = ENav3DPathFindingCallResult::NoVolume;
		UE_LOG(LogTemp, Error, TEXT("Pathfinding cannot initialise. Nav3D component owner is not inside a Nav3D volume"));
		return;
	}
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
	Config.EstimateWeight = HeuristicWeight;
	Config.NodeSizePreference = NodeSizePreference;
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

// Apply Catmull-Rom smoothing to the Nav3DPath shared path
void UNav3DComponent::ApplyPathSmoothing(FNav3DPath& Path) const {
	if (PathSmoothing < 1 || Path.Points.Num() < 3) return;
	const FNav3DPathPoint Start = FNav3DPathPoint(
		Path.Points[1].PointLocation - Path.Points[0].PointLocation,
		Path.Points[0].PointLayer);

	Path.Points.Insert(FNav3DPathPoint(Path.Points[0].PointLocation - Start.PointLocation, Path.Points[0].PointLayer), 0);  

	const FNav3DPathPoint End = FNav3DPathPoint(
		Path.Points[Path.Points.Num() - 1].PointLocation - Path.Points[Path.Points.Num() - 2].PointLocation,
		Path.Points[Path.Points.Num() - 1].PointLayer);

	Path.Points.Add(
		FNav3DPathPoint(
			Path.Points[Path.Points.Num() - 1].PointLocation + End.PointLocation,
			Path.Points[Path.Points.Num() - 1].PointLayer));

	TArray<FNav3DPathPoint> SplinePoints;
	for (int32 I = 0; I < Path.Points.Num() - 3; I++) {
		for (int32 J = 0; J < PathSmoothing; J++) {
			const float T = 1 / PathSmoothing * J;
			SplinePoints.Add(FNav3DPathPoint(0.5f * (2.f * Path.Points[I+1].PointLocation + (-1.f * Path.Points[I].PointLocation + Path.Points[I+2].PointLocation) * T
                + (2.0f * Path.Points[I].PointLocation - 5.f * Path.Points[I+1].PointLocation + 4.f * Path.Points[I+2].PointLocation - Path.Points[I+3].PointLocation) * FMath::Pow(T, 2.f)
                + (-1.f * Path.Points[I].PointLocation + 3.f * Path.Points[I+1].PointLocation - 3.f * Path.Points[I+2].PointLocation + Path.Points[I+3].PointLocation) * FMath::Pow(T, 3.f)
            ), Path.Points[I].PointLayer));
		}
	}
	SplinePoints.Add(Path.Points[Path.Points.Num() - 2]);
	Path.SetPoints(SplinePoints);
}

void UNav3DComponent::DebugDrawNavPath(const FNav3DPath& Path) const {
	if (!GetWorld() || Path.Points.Num() < 2) return;
	DrawDebugSphere(GetWorld(), Path.Points[0].PointLocation,DebugPathLineScale * 2.f, 12, DebugPathColour,true,-1.f,0, DebugPathLineScale);
	for (int32 I = 1; I < Path.Points.Num(); I++) {
		DrawDebugLine(GetWorld(), Path.Points[I - 1].PointLocation, Path.Points[I].PointLocation, DebugPathColour, true, -1.f, 0, DebugPathLineScale);
		DrawDebugSphere( GetWorld(), Path.Points[I].PointLocation, DebugPathLineScale * 2.f, 12, DebugPathColour, true, -1.f, 0, DebugPathLineScale);		
	}
}