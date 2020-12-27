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
	LastLocation = FNav3DOctreeEdge(0, 0, 0);
	Nav3DPath = MakeShareable<FNav3DPath>(new FNav3DPath());
}

void UNav3DComponent::BeginPlay()
{
	Super::BeginPlay();
}

bool UNav3DComponent::VolumeContainsOctree() const
{
	return Volume && GetOwner() && Volume->EncompassesPoint(GetPawnLocation()) && Volume->OctreeValid();
}

bool UNav3DComponent::FindVolume()
{
	TArray<AActor*> Volumes;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANav3DVolume::StaticClass(), Volumes);

	for (auto& NavVolume : Volumes)
	{
		ANav3DVolume* CurrentVolume = Cast<ANav3DVolume>(NavVolume);
		if (CurrentVolume && Volume->EncompassesPoint(GetOwner()->GetActorLocation()))
		{
			Volume = CurrentVolume;
			return true;
		}
	}
	return false;
}

void UNav3DComponent::TickComponent(const float DeltaTime, const ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!VolumeContainsOctree()) FindVolume();
	else if (bDebugCurrentPosition) DebugLocalPosition();
}
FNav3DOctreeEdge UNav3DComponent::GetEdgeAtLocation() const
{
	FNav3DOctreeEdge Edge;
	if (VolumeContainsOctree())
	{
		Volume->GetEdge(GetOwner()->GetActorLocation(), Edge);
		if (Edge == LastLocation) return Edge;
		LastLocation = Edge;

		if (bDebugCurrentPosition)
		{
			FVector NodePosition;
			const bool bIsValid = Volume->GetEdgeLocation(Edge, NodePosition);
			DrawDebugLine(GetWorld(), GetPawnLocation(), NodePosition, bIsValid ? FColor::Green : FColor::Red, false, -1.f, 0, 10.f);
			DrawDebugString(GetWorld(), GetPawnLocation() + FVector(0.f, 0.f, -50.f), Edge.ToString(), nullptr, FColor::Yellow, 0.01f);
		}
	}
	return Edge;
}

void UNav3DComponent::FindPath(const FVector& StartLocation, const FVector& TargetLocation, FFindPathTaskCompleteDynamicDelegate OnComplete, bool &bSuccess)
{
	bSuccess = false;
	UE_LOG(LogTemp, Log, TEXT("Finding path from %s and %s"), *StartLocation.ToString(), *TargetLocation.ToString());
	FNav3DOctreeEdge StartEdge;
	FNav3DOctreeEdge TargetEdge;
	if (VolumeContainsOctree())
	{
		if (!Volume->GetEdge(StartLocation, StartEdge))
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to find start edge"));
			return;
		}

		if (!Volume->GetEdge(TargetLocation, TargetEdge))
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to find target edge"));
			return;
		}

		FNav3DPathFindingConfig Config;
		Config.EstimateWeight = HeuristicWeight;
		Config.NodeSizeCompensation = NodeSizeCompensation;
		Config.PathSmoothing = PathSmoothing;

		(new FAutoDeleteAsyncTask<FNav3DFindPathTask>(this, StartEdge, TargetEdge, StartLocation, TargetLocation, Config, &Nav3DPath, OnComplete))->StartBackgroundTask();
		bSuccess = true;
	}

	UE_LOG(LogTemp, Error, TEXT("Pawn is not inside a Nav3DVolume, or octree has not been generated"));
}

void UNav3DComponent::DebugLocalPosition() const
{
	if (VolumeContainsOctree())
	{
		for (int32 I = 0; I < Volume->NumLayers - 1; I++)
		{
			FIntVector Location;
			Volume->GetVolumeExtents(GetPawnLocation(), I, Location);
			FString CodeString = FString::FromInt(morton3D_64_encode(Location.X, Location.Y, Location.Z));
			DrawDebugString(GetWorld(), GetPawnLocation() + FVector(0.f, 0.f, I * 50.0f), Location.ToString() + " - " + CodeString, nullptr, FColor::White, 0.01f);
		}
	}
}

FVector UNav3DComponent::GetPawnLocation() const {
	FVector Location;
	AController* Controller = Cast<AController>(GetOwner());
	if (Controller) {
		if (APawn* Pawn = Controller->GetPawn()) Location = Pawn->GetActorLocation();
	}
	return Location;
}

void UNav3DComponent::ExecutePathFinding(
	const FNav3DOctreeEdge& StartEdge,
	const FNav3DOctreeEdge& TargetEdge,
	const FVector& StartLocation,
	const FVector& TargetLocation,
	FNav3DPathFindingConfig Config,
	FNav3DPathSharedPtr* Path) {

	TArray<FNav3DOctreeEdge> OpenSet;
	TSet<FNav3DOctreeEdge> ClosedSet;
	TMap<FNav3DOctreeEdge, FNav3DOctreeEdge> Parent;
	TMap<FNav3DOctreeEdge, float> G;
	TMap<FNav3DOctreeEdge, float> F;
	FNav3DOctreeEdge CurrentEdge = FNav3DOctreeEdge();

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

		if (CurrentEdge == TargetEdge) {
			FNav3DPathPoint PathPoint;
			TArray<FNav3DPathPoint> PathPoints;
			if (!Path || !Path->IsValid()) return;

			while (Parent.Contains(CurrentEdge) && !(CurrentEdge == Parent[CurrentEdge])) {
				CurrentEdge = Parent[CurrentEdge];
				Volume->GetEdgeLocation(CurrentEdge, PathPoint.PointLocation);
				PathPoints.Add(PathPoint);
				const FNav3DOctreeNode& Node = Volume->GetNode(CurrentEdge);
				if (CurrentEdge.GetLayerIndex() == 0) {
					if (!Node.HasChildren()) PathPoints[PathPoints.Num() - 1].PointLayer = 1;
					else PathPoints[PathPoints.Num() - 1].PointLayer = 0;
				} else {
					PathPoints[PathPoints.Num() - 1].PointLayer = CurrentEdge.GetLayerIndex() + 1;
				}
			}

			if (PathPoints.Num() > 1) {
				PathPoints[0].PointLocation = TargetLocation;
				PathPoints[PathPoints.Num() - 1].PointLocation = StartLocation;
			} else {
				if (PathPoints.Num() == 0) PathPoints.Emplace();
				PathPoints[0].PointLocation = TargetLocation;
				PathPoints.Emplace(StartLocation, StartEdge.GetLayerIndex());
			}

			for (auto& Point : PathPoints) {
				Path->Get()->GetPathPoints().Add(Point);
			}
			UE_LOG(LogTemp, Display, TEXT("Pathfinding complete, iterations : %i"), I);
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
				if (ClosedSet.Contains(AdjacentEdge)) continue;
				if (!OpenSet.Contains(AdjacentEdge)) OpenSet.Add(AdjacentEdge);

				float GScore = FLT_MAX;
				if (G.Contains(CurrentEdge))
				{
					FVector CurrentLocation(0.f), AdjacentLocation(0.f);
					Volume->GetEdgeLocation(CurrentEdge, CurrentLocation);
					Volume->GetEdgeLocation(AdjacentEdge, AdjacentLocation);
					float Cost = (CurrentLocation - AdjacentLocation).Size() * (1.0f - (static_cast<float>(TargetEdge.GetLayerIndex()) / static_cast<float>(Volume->NumLayers)) * Config.NodeSizeCompensation);
					GScore = G[CurrentEdge] + Cost;
				}
				else G.Add(CurrentEdge, FLT_MAX);

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
	// Use Euclidean distance as heuristic
	return (StartLocation - TargetLocation).Size() * (1.0f - (static_cast<float>(TargetEdge.GetLayerIndex()) / static_cast<float>(Volume->NumLayers)) * Config.NodeSizeCompensation);
}

// Apply Catmull-Rom smoothing to the Nav3DPath shared path
void UNav3DComponent::ApplyPathSmoothing() const {
	if (PathSmoothing < 1) return;
	TArray<FNav3DPathPoint> PathPoints = Nav3DPath.Get()->GetPathPoints();
	if (PathPoints.Num() < 3) return;
	const FNav3DPathPoint Start = FNav3DPathPoint(PathPoints[1].PointLocation - PathPoints[0].PointLocation, PathPoints[0].PointLayer);
	PathPoints.Insert(FNav3DPathPoint(PathPoints[0].PointLocation - Start.PointLocation, PathPoints[0].PointLayer), 0);  
	const FNav3DPathPoint End = FNav3DPathPoint(PathPoints[PathPoints.Num() - 1].PointLocation - PathPoints[PathPoints.Num() - 2].PointLocation, PathPoints[PathPoints.Num() - 1].PointLayer);
	PathPoints.Add(FNav3DPathPoint(PathPoints[PathPoints.Num() - 1].PointLocation + End.PointLocation, PathPoints[PathPoints.Num() - 1].PointLayer));

	TArray<FNav3DPathPoint> SplinePoints;
	for (int32 I = 0; I < PathPoints.Num() - 3; I++) {
		for (int32 J = 0; J < PathSmoothing; J++) {
			const float T = 1 / PathSmoothing * J;
			SplinePoints.Add(FNav3DPathPoint(0.5f * (2.f * PathPoints[I+1].PointLocation + (-1.f * PathPoints[I].PointLocation + PathPoints[I+2].PointLocation) * T
                + (2.0f * PathPoints[I].PointLocation - 5.f * PathPoints[I+1].PointLocation + 4.f * PathPoints[I+2].PointLocation - PathPoints[I+3].PointLocation) * FMath::Pow(T, 2.f)
                + (-1.f * PathPoints[I].PointLocation + 3.f * PathPoints[I+1].PointLocation - 3.f * PathPoints[I+2].PointLocation + PathPoints[I+3].PointLocation) * FMath::Pow(T, 3.f)
            ), PathPoints[I].PointLayer));
		}
	}
	SplinePoints.Add(PathPoints[PathPoints.Num() - 2]);
	Nav3DPath->SetPathPoints(SplinePoints);
}

void UNav3DComponent::DebugDrawNavPath() const
{
	if (!bDebugDrawNavPath) return;
	const UWorld* World = GetWorld();
	if (!World || !Volume) return;
	for (auto& Point: Nav3DPath.Get()->PathPoints) {
		const float Size = Point.PointLayer == 0 ? Volume->GetVoxelScale(Point.PointLayer) * 0.25f : Volume->GetVoxelScale(Point.PointLayer) * 0.5f;
		DrawDebugBox(World, Point.PointLocation, FVector(Size), DebugDrawNavPathColour, true, -1.f, 0, 30.f);
		DrawDebugSphere(World, Point.PointLocation, 30.f, 20, FColor::Cyan, true, -1.f, 0, 100.f);
	}
}