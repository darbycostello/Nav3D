#include "Nav3DVolume.h"
#include "Components/BrushComponent.h"
#include "Components/LineBatchComponent.h"
#include "Engine/CollisionProfile.h"
#include "GameFramework/PlayerController.h"
// #include "Engine/Public/DrawDebugHelpers.h"
#include "DrawDebugHelpers.h"
#include "Engine/OverlapResult.h"

#include "chrono"
#if WITH_EDITOR
#include "Editor.h"
#include "Builders/CubeBuilder.h"
#endif
#include "Nav3DUpdateOctreeTask.h"
#include "Async/ParallelFor.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"

using namespace std::chrono;

ANav3DVolume::ANav3DVolume(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer),
	VolumeOrigin(FVector::ZeroVector),
	VolumeExtent(FVector::ZeroVector) {
	GetBrushComponent()->Mobility = EComponentMobility::Movable;
	PrimaryActorTick.bCanEverTick = true;

	RegisterAllActorTickFunctions(true, false);
	CollisionQueryParams.bFindInitialOverlaps = true;
	CollisionQueryParams.bTraceComplex = false;
	CollisionQueryParams.TraceTag = "Nav3DRasterize";
}

void ANav3DVolume::OnConstruction(const FTransform &Transform)
{
	Super::OnConstruction(Transform);

#if WITH_EDITOR
	DebugDrawOctree();
#endif

}

void ANav3DVolume::Initialise()
{
	Octree.Reset();
	Occluded.Empty();
	CoverMap.Reset();
	NumBytes = 0;

#if WITH_EDITOR
	FlushDebugDraw();

    UpdateVolume();
#endif
}

void ANav3DVolume::UpdateTaskComplete() {
	UnlockOctree();

#if WITH_EDITOR
	// Run the debug draw on the game thread
	AsyncTask(ENamedThreads::GameThread, [=,this]() {
	    DebugDrawOctree();
    });	
#endif
	
}

#if WITH_EDITOR

void ANav3DVolume::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) {
	Super::PostEditChangeProperty(PropertyChangedEvent);
	FProperty* Property = PropertyChangedEvent.Property;
	const FString PropertyName = Property != nullptr ? Property->GetFName().ToString() : "";

	const TSet<FString> CriticalProperties = {
		"VolumeSize",
        "VoxelSize",
		"bEnableCoverMap",
		"MinimumObjectRadius",
		"MinimumDensity"};
	const TSet<FString> DebugProperties = {
		"bDisplayVolumeBounds",
		"VolumeBoundsColor",
		"bDisplayLayers",
		"bDisplayLeafs",
		"bDisplayLeafOcclusion",
		"bDisplayEdgeAdjacency",
		"LineScale",
		"LayerColours",
		"LeafOcclusionColour",
		"EdgeAdjacencyColour",
		"bDisplayMortonCodes",
		"MortonCodeColour",
		"MortonCodeScale",
		"bDisplayCoverMap"};
	if (CriticalProperties.Contains(PropertyName)) {
		Initialise();
		DebugDrawOctree();
	} else if (DebugProperties.Contains(PropertyName)) {
		DebugDrawOctree();
	}
}

void ANav3DVolume::PostEditUndo() {
	Super::PostEditUndo();
	Initialise();
}

void ANav3DVolume::FlushDebugDraw() const {
	if (!GetWorld()) return;
	FlushPersistentDebugLines(GetWorld());
	FlushDebugStrings(GetWorld());
}

void ANav3DVolume::RequestOctreeDebugDraw() {
	bDebugDrawRequested = true;
	if (!bOctreeLocked) {
		DebugDrawOctree();
		bDebugDrawRequested = false;
	}
}

void ANav3DVolume::DebugDrawOctree() {
	GetWorld()->PersistentLineBatcher->SetComponentTickEnabled(false);
	FlushDebugDraw();
	if (OctreeValid()) {
		for (int32 I = 0; I < Octree.Layers.Num(); I++) {
			for (int32 J = 0; J < Octree.Layers[I].Num(); J++) {
				FVector NodeLocation;
				GetNodeLocation(I, Octree.Layers[I][J].MortonCode, NodeLocation);
				if (I == 0 && bDisplayLeafs || I > 0 && bDisplayLayers) {
					DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[I]), GetLayerColour(I));
				}
				if (bDisplayMortonCodes) {
					DebugDrawMortonCode(NodeLocation, FString::FromInt(I) + ":" + FString::FromInt(Octree.Layers[I][J].MortonCode), MortonCodeColour);
				}
			}	
		}
		
		// Edges must be rebuilt to draw edge adjacency
		if (bDisplayEdgeAdjacency) {
			DebugEdges.Empty();
			for (int32 I = NumLayers - 2; I >= 0; I--) BuildEdges(I);
			DebugDrawEdgeAdjacency();
		}

		if (bDisplayLeafOcclusion) DebugDrawLeafOcclusion();
		DebugDrawNavPaths();
		DebugDrawNavLocations();
		VerifyModifierVolumes();
		DebugDrawModifierVolumes();

		if (bDisplayCoverMap) {
			DebugDrawCoverMapLocations();
			DebugDrawOcclusionComponentCover();
		}
	}
}

void ANav3DVolume::DebugDrawVolume() const
{
	if (!GetWorld()) return;
	if (bDisplayVolumeBounds)
	{
		const FBox Box = GetBoundingBox();
		DebugDrawBoundsMesh(Box, VolumeBoundsColour);
	}
}

void ANav3DVolume::DebugDrawEdgeAdjacency() const {
	if (!GetWorld()) return;
	for (auto& Edge: DebugEdges) {
		DrawDebugLine(GetWorld(), Edge.Start, Edge.End, GetLayerColour(Edge.LayerIndex), true, -1, 0, LineScale);
	}
}

void ANav3DVolume::DebugDrawLeafOcclusion() {
	for (uint_fast32_t I = 0; I < static_cast<uint_fast32_t>(Octree.Leafs.Num()); I++) {
		for (uint8 J = 0; J < 64; J++) {
			if (Octree.Leafs[I].GetSubNode(J)) {
				const FNav3DOctreeEdge Edge { 0, I, J };
				FVector NodeLocation;
				GetNodeLocation(Edge, NodeLocation);
				DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[0] * 0.25f), LeafOcclusionColour);
			}
		}
	}
}

void ANav3DVolume::DebugDrawNavPaths() const {
	for (auto& DebugPath: DebugPaths) {
		DrawDebugSphere(GetWorld(), DebugPath.Points[0],DebugPath.LineScale * 2.f, 12, DebugPath.Colour,true,-1.f,0, DebugPath.LineScale);
		for (int32 I = 1; I < DebugPath.Points.Num(); I++) {
			DrawDebugLine(GetWorld(), DebugPath.Points[I - 1], DebugPath.Points[I], DebugPath.Colour, true, -1.f, 0, DebugPath.LineScale);
			DrawDebugSphere(GetWorld(), DebugPath.Points[I], DebugPath.LineScale * 2.f, 12, DebugPath.Colour, true, -1.f, 0, DebugPath.LineScale);		
		}
	}
}

void ANav3DVolume::DebugDrawNavLocations() const {
	for (auto& DebugLocation: DebugLocations) {
		DrawDebugSphere(GetWorld(), DebugLocation.Location,DebugLocation.LineScale * 2.f, 12, DebugLocation.Colour,true,-1.f,0, DebugLocation.LineScale);
	}
}

void ANav3DVolume::DebugDrawModifierVolumes() const {
    for (auto& ModifierVolume : ModifierVolumes) {
        if (!ModifierVolume) return;
        ModifierVolume->DebugDrawModifierVolume();
    }
}

void ANav3DVolume::ClearAllDebugNavPaths()
{
    DebugPaths.Empty();
}

void ANav3DVolume::AddDebugNavPath(const FNav3DDebugPath DebugPath) {
	DebugPaths.Add(DebugPath);
	RequestOctreeDebugDraw();
}

void ANav3DVolume::AddDebugLocation(const FNav3DDebugLocation DebugLocation) {
	DebugLocations.Add(DebugLocation);
	RequestOctreeDebugDraw();
}

void ANav3DVolume::DebugDrawCoverMapLocations() const {
	for (auto& Node: CoverMap.Nodes) {
		for (auto& Index : Node.Value.Locations) {
			for (auto& Location: Index.Value) {
				FColor Colour = FColor(
				FMath::Max(255*CoverNormals[Index.Key].X, 64.f),
				FMath::Max(255*CoverNormals[Index.Key].Y, 64.f),
				FMath::Max(255*CoverNormals[Index.Key].Z, 64.f));
				FVector End = Location + CoverNormals[Index.Key] * (VoxelSize + Clearance);
				DrawDebugDirectionalArrow(GetWorld(), Location, End, 1000.0f, Colour, true, -1, 0, 20.f);		
			}
		}
	}
}

void ANav3DVolume::DebugDrawOcclusionComponentCover() const {
	for (auto& OcclusionComponent: OcclusionComponents) {
		if (!IsValid(OcclusionComponent)) return;
		if (!OcclusionComponent->GetCoverEnabled()) return;
		for (int32 I = 0; I < 26; I++) {
			TArray<FVector> Locations;
			OcclusionComponent->GetCoverLocations(I, Locations);	
			for (auto& Location: Locations) {
				FColor Colour = FColor(
                FMath::Max(255*CoverNormals[I].X, 64.f),
                FMath::Max(255*CoverNormals[I].Y, 64.f),
                FMath::Max(255*CoverNormals[I].Z, 64.f));
				FVector End = Location + CoverNormals[I] * VoxelSize;
				DrawDebugDirectionalArrow(GetWorld(), Location, End, 1000.0f, Colour, true, -1, 0, 20.f);		
			}
		}
	}
}

void ANav3DVolume::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
	Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ANav3DVolume::EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
	Super::EditorApplyRotation(DeltaRotation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ANav3DVolume::EditorApplyScale( const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown )
{
	Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ANav3DVolume::UpdateVolume() {

	// Calculate the nearest integer exponent to fit the voxel size perfectly within the volume extents
	VoxelExponent = FMath::RoundToInt(FMath::Log2(VolumeSize / (VoxelSize * 4)));
	NumLayers = VoxelExponent + 1;

	// Build a list of voxel half-scale sizes for each layer
	VoxelHalfSizes.Reset();
	VoxelHalfSizes.Reserve(NumLayers);
	for (int32 I = 0; I < NumLayers; I++) VoxelHalfSizes.Add(GetVoxelScale(I) * 0.5f);

	ActualVolumeSize = GetActualVolumeSize();
    UCubeBuilder* CubeBuilder = Cast<UCubeBuilder>(GEditor->FindBrushBuilder(UCubeBuilder::StaticClass()));
	CubeBuilder->X = ActualVolumeSize;
	CubeBuilder->Y = ActualVolumeSize;
	CubeBuilder->Z = ActualVolumeSize;
	CubeBuilder->Build(GetWorld(), this);

	const FBox Bounds = GetComponentsBoundingBox(true);
	Bounds.GetCenterAndExtents(VolumeOrigin, VolumeExtent);
}

#endif

TArray<AActor*> ANav3DVolume::GatherOcclusionActors() {
	TArray<AActor*> Actors, OcclusionActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), Actors);
	for (auto& Actor: Actors) {
		UNav3DOcclusionComponent* OcclusionComponent = Cast<UNav3DOcclusionComponent>(Actor->GetComponentByClass(UNav3DOcclusionComponent::StaticClass()));
		if (OcclusionComponent) {
			OcclusionActors.Add(Actor);
			OcclusionComponents.Add(OcclusionComponent);
		}
	}
	return OcclusionActors;
}

// Regenerate the entire sparse voxel octree
bool ANav3DVolume::BuildOctree() {

	// Initial setup
	Initialise();

	// Gather any occlusion component owners and add them to the collision ignore list
	CollisionQueryParams.AddIgnoredActors(GatherOcclusionActors());

#if WITH_EDITOR
	const auto StartTime = high_resolution_clock::now();
#endif

	// Build the octree
	RasterizeInitial();
	Octree.Leafs.AddDefaulted(Occluded[0].Num() * 8 * 0.25f);
	for (int32 I = 0; I < NumLayers; I++) RasterizeLayer(I);
	for (int32 I = NumLayers - 2; I >= 0; I--) BuildEdges(I);

	// Clean up and cache the octree
	NumBytes = Octree.GetSize();
	CollisionQueryParams.ClearIgnoredActors();
	CachedOctree = Octree;

#if WITH_EDITOR
	const float Duration = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - StartTime).count() / 1000.0f;

	// Octree info
	int32 NumNodes = 0;
	for (int32 I = 0; I < NumLayers; I++) NumNodes += Octree.Layers[I].Num();

	// Cover map info
	const int32 NumCoverMapBytes = CoverMap.GetSize();
	int32 NumCoverActors = 0;
	int32 NumCoverLocations = 0;
	for (auto& Node: CoverMap.Nodes) {
		NumCoverActors++;
		for (auto& Index : Node.Value.Locations) {
			for (auto& Location: Index.Value) {
				NumCoverLocations++;
			}
		}
	}

	UE_LOG(LogTemp, Display, TEXT("Generation Time : %f seconds"), Duration);
	UE_LOG(LogTemp, Display, TEXT("Desired Volume Size : %fcm"), VolumeSize);
	UE_LOG(LogTemp, Display, TEXT("Actual Volume Size : %fcm"), ActualVolumeSize);
	UE_LOG(LogTemp, Display, TEXT("Voxel Size : %fcm"), VoxelSize);
	UE_LOG(LogTemp, Display, TEXT("Voxel Exponent: %i"), VoxelExponent);
	UE_LOG(LogTemp, Display, TEXT("Total Layers : %i"), NumLayers);
	UE_LOG(LogTemp, Display, TEXT("Total Nodes : %i"), NumNodes);
	UE_LOG(LogTemp, Display, TEXT("Total Leafs : %i"), Octree.Leafs.Num());
	UE_LOG(LogTemp, Display, TEXT("Total Octree Bytes : %i"), NumBytes);
	UE_LOG(LogTemp, Display, TEXT("Total Cover Map Actors : %i"), NumCoverActors);
	UE_LOG(LogTemp, Display, TEXT("Total Cover Map Locations : %i"), NumCoverLocations);
	UE_LOG(LogTemp, Display, TEXT("Total Cover Map Bytes : %i"), NumCoverMapBytes);
	DebugDrawOctree();
#endif

	return true;
}

FColor ANav3DVolume::GetLayerColour(const int32 LayerIndex) const {
	const float Ratio = LayerColours.Num() / static_cast<float>(NumLayers) * LayerIndex;
	const int32 FirstIndex = FMath::FloorToInt(Ratio);
	const int32 LastIndex = FMath::Min(FMath::CeilToInt(Ratio), LayerColours.Num() - 1);
	const float Lerp = FMath::Fmod(Ratio, 1);
	return FColor(
        FMath::Lerp(LayerColours[FirstIndex].R, LayerColours[LastIndex].R, Lerp),
        FMath::Lerp(LayerColours[FirstIndex].G, LayerColours[LastIndex].G, Lerp),
        FMath::Lerp(LayerColours[FirstIndex].B, LayerColours[LastIndex].B, Lerp)
	);
}
 
FBox ANav3DVolume::GetBoundingBox() const
{
	const FBoxSphereBounds Bounds = GetBrushComponent()->CalcBounds(ActorToWorld());
	return Bounds.GetBox();
}

bool ANav3DVolume::GetNodeLocation(const uint8 LayerIndex, const uint_fast64_t MortonCode, FVector& Location) const
{
	const float Scale = VoxelHalfSizes[LayerIndex] * 2;
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);
	Location = VolumeOrigin - VolumeExtent + Scale * FVector(X, Y, Z) + FVector(Scale * 0.5f);
	return true;
}

bool ANav3DVolume::GetEdgeLocation(const FNav3DOctreeEdge& Edge, FVector& Location) const
{
	const FNav3DOctreeNode& Node = GetLayer(Edge.GetLayerIndex())[Edge.GetNodeIndex()];
	GetNodeLocation(Edge.GetLayerIndex(), Node.MortonCode, Location);
	if (Edge.GetLayerIndex() == 0 && Node.FirstChild.IsValid())
	{
		uint_fast32_t X, Y, Z;
		morton3D_64_decode(Edge.GetSubNodeIndex(), X, Y, Z);
		const float Scale = VoxelHalfSizes[0];
		Location += FVector(X * Scale * 0.25f, Y * Scale * 0.25f, Z * Scale * 0.25f) - FVector(Scale * 0.375);
		const FNav3DOctreeLeaf& Leaf = Octree.Leafs[Node.FirstChild.NodeIndex];
		return !Leaf.GetSubNode(Edge.GetSubNodeIndex());
	}
	return true;
}

const FNav3DOctreeNode& ANav3DVolume::GetNode(const FNav3DOctreeEdge& Edge) const
{
	return Octree.Layers[Edge.LayerIndex][Edge.NodeIndex];
}

bool ANav3DVolume::EdgeNodeIsValid(const FNav3DOctreeEdge& Edge) const {
	if (Edge.LayerIndex >= Octree.Layers.Num()) return false;
	return Edge.IsValid() && static_cast<int32>(Edge.NodeIndex) < Octree.Layers[Edge.LayerIndex].Num();
}

void ANav3DVolume::GetAdjacentLeafs(const FNav3DOctreeEdge& Edge, TArray<FNav3DOctreeEdge>& AdjacentEdges) const
{
	const uint_fast64_t LeafIndex = Edge.SubNodeIndex;
	if (EdgeNodeIsValid(Edge)) {
		const FNav3DOctreeNode& Node = GetNode(Edge);
		if (static_cast<int32>(Node.FirstChild.NodeIndex) >= Octree.Leafs.Num()) return;
		const FNav3DOctreeLeaf& FirstLeaf = Octree.Leafs[Node.FirstChild.NodeIndex];

		uint_fast32_t X = 0, Y = 0, Z = 0;
		morton3D_64_decode(LeafIndex, X, Y, Z);
		for (int32 I = 0; I < 6; I++)
		{
			int32 SignedX = X + Directions[I].X;
			int32 SignedY = Y + Directions[I].Y;
			int32 SignedZ = Z + Directions[I].Z;
			if (SignedX >= 0 && SignedX < 4
				&& SignedY >= 0 && SignedY < 4
				&& SignedZ >= 0 && SignedZ < 4) {
				
				uint_fast64_t Index = morton3D_64_encode(SignedX, SignedY, SignedZ);

				if (FirstLeaf.GetSubNode(Index)) continue;
				AdjacentEdges.Emplace(0, Edge.NodeIndex, Index);
				continue;
			}
			const FNav3DOctreeEdge& AdjacentEdge = Node.AdjacentEdges[I];

			if (EdgeNodeIsValid(AdjacentEdge)) {
				
				const FNav3DOctreeNode& AdjacentNode = GetNode(AdjacentEdge);
				if (!AdjacentNode.FirstChild.IsValid())
				{
					AdjacentEdges.Add(AdjacentEdge);
					continue;
				}
				
				if (AdjacentNode.FirstChild.NodeIndex < static_cast<uint_fast32_t>(Octree.Leafs.Num())) {
					const FNav3DOctreeLeaf& Leaf = Octree.Leafs[AdjacentNode.FirstChild.NodeIndex];

					if (!Leaf.IsOccluded()) {
						if (SignedX < 0) SignedX = 3;
						else if (SignedX > 3) SignedX = 0;
						else if (SignedY < 0) SignedY = 3;
						else if (SignedY > 3) SignedY = 0;
						else if (SignedZ < 0) SignedZ = 3;
						else if (SignedZ > 3) SignedZ = 0;
						const uint_fast64_t SubCode = morton3D_64_encode(SignedX, SignedY, SignedZ);
						if (!Leaf.GetSubNode(SubCode)) AdjacentEdges.Emplace(0, AdjacentNode.FirstChild.NodeIndex, SubCode);	
					}
				}
			}
		}
	}
}

void ANav3DVolume::GetAdjacentEdges(const FNav3DOctreeEdge& Edge, TArray<FNav3DOctreeEdge>& AdjacentEdges) const
{
	if (!EdgeNodeIsValid(Edge)) return;
	const FNav3DOctreeNode& Node = GetNode(Edge);
	for (int32 I = 0; I < 6; I++) {
		const FNav3DOctreeEdge& AdjacentEdge = Node.AdjacentEdges[I];
		if (!AdjacentEdge.IsValid()) continue;
		if (!EdgeNodeIsValid(AdjacentEdge)) continue;
		const FNav3DOctreeNode& AdjacentNode = GetNode(AdjacentEdge);
		if (!AdjacentNode.HasChildren()) {
			AdjacentEdges.Add(AdjacentEdge);
			continue;
		}

		TArray<FNav3DOctreeEdge> Edges;
		Edges.Push(AdjacentEdge);

		while (Edges.Num() > 0) {
			FNav3DOctreeEdge CurrentEdge = Edges.Pop();
			const FNav3DOctreeNode& CurrentNode = GetNode(CurrentEdge);
			if (!CurrentNode.HasChildren()) {
				AdjacentEdges.Add(CurrentEdge);
				continue;
			}
			if (CurrentEdge.GetLayerIndex() > 0) {
				for (const int32& ChildIndex : NodeOffsets[I]) {
					FNav3DOctreeEdge EdgeChild = CurrentNode.FirstChild;
					EdgeChild.NodeIndex += ChildIndex;
					const FNav3DOctreeNode& NodeChild = GetNode(EdgeChild);
					if (NodeChild.HasChildren()) Edges.Emplace(EdgeChild);
					else AdjacentEdges.Emplace(EdgeChild);
				}
			} else {
				for (const int32& LeafIndex : LeafOffsets[I]) {
					FNav3DOctreeEdge EdgeChild = AdjacentNode.FirstChild;
					const FNav3DOctreeLeaf& Leaf = Octree.Leafs[EdgeChild.NodeIndex];
					EdgeChild.SubNodeIndex = LeafIndex;
					if (!Leaf.GetSubNode(LeafIndex)) {
						AdjacentEdges.Emplace(EdgeChild);
					}
				}
			}
		}
	}
}

void ANav3DVolume::Serialize(FArchive& Ar) {
	Super::Serialize(Ar);
	Ar << Octree;
	Ar << VoxelHalfSizes;
    Ar << VolumeOrigin;
	Ar << VolumeExtent;
	Ar << CoverMap;
	NumBytes = Octree.GetSize();
}

float ANav3DVolume::GetVoxelScale(const uint8 LayerIndex) const {
	return VolumeExtent.X / FMath::Pow(2.0f, VoxelExponent) * FMath::Pow(2.0f, LayerIndex + 1);
}

int32 ANav3DVolume::GetLayerNodeCount(const uint8 LayerIndex) const {
	return FMath::Pow(8.f, VoxelExponent - LayerIndex);
}

int32 ANav3DVolume::GetSegmentNodeCount(const uint8 LayerIndex) const {
	return FMath::Pow(2.f, VoxelExponent - LayerIndex);
}

void ANav3DVolume::BeginPlay() {
	CachedOctree = Octree;
	OnUpdateComplete.BindUFunction(this, FName("UpdateTaskComplete"));
	SetActorTickInterval(TickInterval);
}

void ANav3DVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!bOctreeLocked) {
		if (bUpdateRequested) {

			// Prevent any further requests until update task is complete
			LockOctree();

			// Execute UpdateOctree as background task
			(new FAutoDeleteAsyncTask<FNav3DUpdateOctreeTask>(this, OnUpdateComplete))->StartBackgroundTask();
		
			// Update complete
			bUpdateRequested = false;

#if WITH_EDITOR		
		} else if (bDebugDrawRequested) {
			DebugDrawOctree();
			bDebugDrawRequested = false;
#endif

		}
	}
}

void ANav3DVolume::PostRegisterAllComponents() {
	Super::PostRegisterAllComponents();
}

void ANav3DVolume::PostUnregisterAllComponents() {
	Super::PostUnregisterAllComponents();
}

void ANav3DVolume::BuildEdges(const uint8 LayerIndex) {
	if (Octree.Layers.Num() == 0) return;
	TArray<FNav3DOctreeNode>& Layer = GetLayer(LayerIndex);
	for (int32 I = 0; I < Layer.Num(); I++) {
		FNav3DOctreeNode& Node = Layer[I];
		FVector NodeLocation;
		GetNodeLocation(LayerIndex, Node.MortonCode, NodeLocation);

		for (int32 Direction = 0; Direction < 6; Direction++) {
			int32 NodeIndex = I;
            FNav3DOctreeEdge& Edge = Node.AdjacentEdges[Direction];
			uint8 CurrentLayer = LayerIndex;
            while (!FindEdge(CurrentLayer, NodeIndex, Direction, Edge, NodeLocation) && CurrentLayer < Octree.Layers.Num() - 2) {
                FNav3DOctreeEdge& ParentEdge = GetLayer(CurrentLayer)[NodeIndex].Parent;
            	if (ParentEdge.IsValid()) {
                    NodeIndex = ParentEdge.NodeIndex;
                    CurrentLayer = ParentEdge.LayerIndex;
                } else {
                    CurrentLayer++;
                    GetNodeIndex(CurrentLayer, Node.MortonCode >> 3, NodeIndex);
                }
            }
		}
	}
}

bool ANav3DVolume::FindEdge(const uint8 LayerIndex, const int32 NodeIndex, const uint8 Direction, FNav3DOctreeEdge& Edge, const FVector& NodeLocation) {
	const int32 MaxCoordinate = GetSegmentNodeCount(LayerIndex);
	
	TArray<FNav3DOctreeNode>& Layer = GetLayer(LayerIndex);
	FNav3DOctreeNode& TargetNode = GetLayer(LayerIndex)[NodeIndex];
	
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(TargetNode.MortonCode, X, Y, Z);
	
	// Create a signed vector from the X Y and Z values
	FIntVector S( static_cast<int32>(X), static_cast<int32>(Y), static_cast<int32>(Z) );
	S += Directions[Direction];
	if (S.X < 0 || S.X >= MaxCoordinate ||
		S.Y < 0 || S.Y >= MaxCoordinate ||
		S.Z < 0 || S.Z >= MaxCoordinate) {
		Edge.Invalidate();
		return true;
	}
	X = S.X;
	Y = S.Y;
	Z = S.Z;
	
	const uint_fast64_t AdjacentCode = morton3D_64_encode(X, Y, Z);
	int32 Stop = Layer.Num();
	int32 NodeDelta = 1;
	if (AdjacentCode < TargetNode.MortonCode)
	{
		NodeDelta = -1;
		Stop = -1;
	}

	for (int32 I = NodeIndex + NodeDelta; I != Stop; I += NodeDelta)
	{
		FNav3DOctreeNode& Node = Layer[I];
		if (Node.MortonCode == AdjacentCode)
		{
			if (LayerIndex == 0 && 
                Node.HasChildren() && 
                Octree.Leafs[Node.FirstChild.NodeIndex].IsOccluded()) {
				
				Edge.Invalidate();
				return true;
			}
			Edge.SetLayerIndex(LayerIndex);
			if (I >= Layer.Num() || I < 0) break; 
			Edge.SetNodeIndex(I);
			FVector AdjacentLocation;
			GetNodeLocation(LayerIndex, AdjacentCode, AdjacentLocation);
			
#if WITH_EDITOR
			DebugEdges.Add(FNav3DDebugEdge(NodeLocation, AdjacentLocation, LayerIndex));
#endif
			
			return true;
		}
		if (NodeDelta == -1 && Node.MortonCode < AdjacentCode || NodeDelta == 1 && Node.MortonCode > AdjacentCode)
		{
			return false;
		}
	}
	return false;
}

bool ANav3DVolume::IsOccluded(const FVector& Location, const float Size, TArray<FOverlapResult>& OverlapResults) const {
	return GetWorld()->OverlapMultiByChannel(
		OverlapResults,
        Location,
        FQuat::Identity,
        CollisionChannel,
        FCollisionShape::MakeBox(FVector(Size + Clearance)),
        CollisionQueryParams
    );
}

bool ANav3DVolume::IsOccluded(const FVector& Location, const float Size) const {
	return GetWorld()->OverlapBlockingTestByChannel(
        Location,
        FQuat::Identity,
        CollisionChannel,
        FCollisionShape::MakeBox(FVector(Size + Clearance)),
        CollisionQueryParams
    );
}

bool ANav3DVolume::InDebugRange(const FVector Location) const {
	if (!GetWorld()) return true;
	if (GetWorld()->ViewLocationsRenderedLastFrame.Num() == 0) return true;
	return FVector::Dist(GetWorld()->ViewLocationsRenderedLastFrame[0], Location) < DebugDistance;
}

bool ANav3DVolume::GetNodeIndex(const uint8 LayerIndex, const uint_fast64_t NodeMortonCode, int32& NodeIndex) const
{
	const auto& OctreeLayer = Octree.Layers[LayerIndex];
	int32 Start = 0;
	int32 End = OctreeLayer.Num() - 1;
	int32 Mean = (Start + End) * 0.5f;

	// Binary search by Morton code
	while (Start <= End)
	{
		if (OctreeLayer[Mean].MortonCode < NodeMortonCode) Start = Mean + 1;
		else if (OctreeLayer[Mean].MortonCode == NodeMortonCode) {
			NodeIndex = Mean;
			return true;
		} else End = Mean - 1;
		Mean = (Start + End) * 0.5f;
	}
	return false;
}

int32 ANav3DVolume::GetInsertIndex(const uint8 LayerIndex, const uint_fast64_t MortonCode) const
{
	const auto& OctreeLayer = Octree.Layers[LayerIndex];
	int32 Start = 0;
	int32 End = OctreeLayer.Num() - 1;
	int32 Mean = (Start + End) * 0.5f;

	// Binary search by Morton code
	while (Start <= End)
	{
		if (OctreeLayer[Mean].MortonCode < MortonCode) Start = Mean + 1;
		else End = Mean - 1;
		Mean = (Start + End) * 0.5f;
	}
	return Start;
}

void ANav3DVolume::RasterizeInitial() {
	Occluded.Emplace();
	for (int32 I = 0; I < GetLayerNodeCount(1); I++) {
		FVector Location;
		GetNodeLocation(1, I, Location);
		if (IsOccluded(Location, VoxelHalfSizes[1])) {
			Occluded[0].Add(I);
		}
	}

	for (int32 I = 0; I < VoxelExponent; I++) {
		Occluded.Emplace();
		for (uint_fast64_t& MortonCode : Occluded[I]) {
			Occluded[I + 1].Add(MortonCode >> 3);
		}
	}
}

void ANav3DVolume::RasterizeLayer(const uint8 LayerIndex)
{
	Octree.Layers.Emplace();
	int32 LeafIndex = 0;
	if (LayerIndex == 0)
	{
		Octree.Leafs.Reserve(Occluded[0].Num() * 8);
		Octree.Layers[0].Reserve(Occluded[0].Num() * 8);
		const int32 NumNodes = GetLayerNodeCount(0);
		for (int32 I = 0; I < NumNodes; I++)
		{			
			if (Occluded[0].Contains(I >> 3))
			{
				const int32 Index = GetLayer(0).Emplace();
				FNav3DOctreeNode& NewNode = Octree.Layers[0][Index];
				NewNode.MortonCode = I;
				FVector NodeLocation;
				GetNodeLocation(0, I, NodeLocation);
				if (IsOccluded(NodeLocation, VoxelHalfSizes[0])) {
					RasterizeLeaf(NodeLocation, LeafIndex);
					NewNode.FirstChild.SetLayerIndex(0);
					NewNode.FirstChild.SetNodeIndex(LeafIndex);
					NewNode.FirstChild.SetSubNodeIndex(0);
					LeafIndex++;
				} else {
					Octree.Leafs.AddDefaulted(1);
					LeafIndex++;
					NewNode.FirstChild.Invalidate();
				}
			}
		}
	} else if (GetLayer(LayerIndex - 1).Num() > 0) {
		Octree.Layers[LayerIndex].Reserve(Occluded[LayerIndex].Num() * 8);
		const int32 NumNodes = GetLayerNodeCount(LayerIndex);
		for (int32 I = 0; I < NumNodes; I++) {
			if (Occluded[LayerIndex].Contains(I >> 3)) {
				const int32 Index = GetLayer(LayerIndex).Emplace();
				FNav3DOctreeNode& NewNode = GetLayer(LayerIndex)[Index];
				NewNode.MortonCode = I;
				int32 ChildIndex = 0;
				if (GetNodeIndex(LayerIndex - 1, NewNode.MortonCode << 3, ChildIndex))
				{
					NewNode.FirstChild.SetLayerIndex(LayerIndex - 1);
					NewNode.FirstChild.SetNodeIndex(ChildIndex);
					for (int32 C = 0; C < 8; C++) {
						GetLayer(NewNode.FirstChild.GetLayerIndex())[NewNode.FirstChild.GetNodeIndex() + C].Parent.SetLayerIndex(LayerIndex);
						GetLayer(NewNode.FirstChild.GetLayerIndex())[NewNode.FirstChild.GetNodeIndex() + C].Parent.SetNodeIndex(Index);
					}
				} else {
					NewNode.FirstChild.Invalidate();
				}
			}
		}
	}
}

void ANav3DVolume::RasterizeLeaf(const FVector NodeLocation, const int32 LeafIndex)
{
	const FVector Location = NodeLocation - VoxelHalfSizes[0];
	const float VoxelScale = VoxelHalfSizes[0] * 0.5f;
	for (int32 I = 0; I < 64; I++) {
        uint_fast32_t X, Y, Z;
        morton3D_64_decode(I, X, Y, Z);
        const FVector VoxelLocation = Location + FVector(X * VoxelScale, Y * VoxelScale, Z * VoxelScale) + VoxelScale * 0.5f;
        if (LeafIndex >= Octree.Leafs.Num() - 1) Octree.Leafs.AddDefaulted(1);

		if (bEnableCoverMap) {
			// In addition to blocking hit test, retrieve the results of any occlusion overlaps to determine leaf cell normals
			TArray<FOverlapResult> Overlaps;
			if (IsOccluded(VoxelLocation, VoxelScale * 0.5f, Overlaps)) Octree.Leafs[LeafIndex].SetSubNode(I);
			UpdateCoverMap(VoxelLocation, Overlaps);	
		} else {
			if (IsOccluded(VoxelLocation, VoxelScale * 0.5f)) Octree.Leafs[LeafIndex].SetSubNode(I);
		}
    }
}

void ANav3DVolume::UpdateCoverMap(const FVector Location, TArray<FOverlapResult>& Overlaps) {
	if (!bEnableCoverMap) return;
	if (Overlaps.Num() > 0) {
		for (auto& Result: Overlaps) {
			if (Result.bBlockingHit) {
				if (Result.GetActor()->GetComponentsBoundingBox().GetExtent().GetMax() < MinimumObjectRadius) continue;
				FVector HitPoint;
				Result.Component->GetClosestPointOnCollision(Location, HitPoint);
				FVector Normal = (Location - HitPoint).GetSafeNormal();
				if (Normal == FVector::ZeroVector) continue;
				const int32 NormalIndex = GetCoverNormalIndex(Normal);
				
				FNav3DCoverMapNode Node;
				if (CoverMap.Nodes.Contains(Result.GetActor()->GetFName())) {
					Node.Locations = CoverMap.Nodes[Result.GetActor()->GetFName()].Locations;
				}
				
				TArray<FVector> CoverLocations;
				bool bLocationLegal = true;
				if (Node.Locations.Contains(NormalIndex)) {
					CoverLocations = Node.Locations[NormalIndex];
					for (auto& CoverLocation: CoverLocations) {
						if (FVector::Dist(CoverLocation, Location) < MinimumDensity) {
							bLocationLegal = false;
							break;
						}
					}
				}
				if (!bLocationLegal) continue;
				CoverLocations.Add(Location);
				Node.Locations.Add(NormalIndex, CoverLocations);
				CoverMap.Nodes.Add(Result.GetActor()->GetFName(), Node);
			}
		}
	}
}

void ANav3DVolume::UpdateOcclusionComponentCover(const FVector Location, TArray<FOverlapResult>& Overlaps) const {
	if (!bEnableCoverMap) return;
	if (Overlaps.Num() > 0) {
		for (auto& Result: Overlaps) {
			if (Result.bBlockingHit) {
				UActorComponent* ActorComponent = Result.GetActor()->GetComponentByClass(UNav3DOcclusionComponent::StaticClass());
				if (!ActorComponent) continue;
				UNav3DOcclusionComponent* UOcclusionComponent = Cast<UNav3DOcclusionComponent>(ActorComponent);  
				if (UOcclusionComponent->GetCoverEnabled()) {
					FVector HitPoint;
					Result.Component->GetClosestPointOnCollision(Location, HitPoint);
					FVector Normal = (Location - HitPoint).GetSafeNormal();
					if (Normal == FVector::ZeroVector) continue;
					const int32 NormalIndex = GetCoverNormalIndex(Normal);
					
					TArray<FVector> CoverLocations;
					bool bLocationLegal = true;
					if (UOcclusionComponent->GetCoverLocations(NormalIndex, CoverLocations)) {
						for (auto& CoverLocation: CoverLocations) {
							if (FVector::Dist(CoverLocation, Location) < MinimumDensity) {
								bLocationLegal = false;
								break;
							}
						}
					}
					if (!bLocationLegal) continue;
					CoverLocations.Add(Location);
					UOcclusionComponent->AddCoverLocations(NormalIndex, CoverLocations);
				}
			}
		}
	}
}

int32 ANav3DVolume::GetCoverNormalIndex(const FVector Normal) const {
	TArray<float> DotProducts;
	for (int32 I = 0; I < 26; I++) {
		DotProducts.Add(FVector::DotProduct(Normal, CoverNormals[I]));
	}
	int32 CoverNormalIndex;
	float MaxDotProduct;
	UKismetMathLibrary::MaxOfFloatArray(DotProducts, CoverNormalIndex, MaxDotProduct);
	return CoverNormalIndex;
} 

void ANav3DVolume::GetVolumeExtents(const FVector& Location, const int32 LayerIndex, FIntVector& Extents) const
{
	const FBox ComponentBox = GetComponentsBoundingBox(true);
	FVector ComponentOrigin;
	FVector ComponentExtent;
	ComponentBox.GetCenterAndExtents(ComponentOrigin, ComponentExtent);
	const FVector LocationLocal = Location - ComponentOrigin - ComponentExtent;
	const float Scale = VoxelHalfSizes[LayerIndex];
	Extents.X = FMath::FloorToInt(LocationLocal.X / Scale);
	Extents.Y = FMath::FloorToInt(LocationLocal.Y / Scale);
	Extents.Z = FMath::FloorToInt(LocationLocal.Z / Scale);
}

void ANav3DVolume::GetMortonVoxel(const FVector& Location, const int32 LayerIndex, FIntVector& MortonLocation) const
{
	const FVector LocationLocal = Location - (VolumeOrigin - VolumeExtent);
	const float Size = VoxelHalfSizes[LayerIndex] * 2;
	MortonLocation.X = FMath::FloorToInt(LocationLocal.X / Size);
	MortonLocation.Y = FMath::FloorToInt(LocationLocal.Y / Size);
	MortonLocation.Z = FMath::FloorToInt(LocationLocal.Z / Size);
}

bool ANav3DVolume::FindAccessibleEdge(FVector& Location, FNav3DOctreeEdge& Edge) {
	for (int32 I = 1; I < 4; I++) {
		for (int32 J = 0; J < 6; J++) {
			FVector OffsetLocation = Location + FVector(Directions[J] * Clearance * I);
			if (GetEdge(OffsetLocation, Edge)) {
				Location = OffsetLocation;
				return true;
			}	
		}
	}
	return false;
}

void ANav3DVolume::GetPathCost(FVector& Location, float& Cost) {
	Cost = 1.f;
	if (ModifierVolumes.Num() == 0) return;
	for (auto& ModifierVolume: ModifierVolumes) {
		if (!ModifierVolume) continue;
		if (ModifierVolume->GetBoundingBox().IsInside(Location))
		{
			Cost += ModifierVolume->GetPathCost();
		}
	}
}

bool ANav3DVolume::GetCoverLocationValid(FVector& Location) const {
	if (ModifierVolumes.Num() == 0) return true;
	for (auto& ModifierVolume: ModifierVolumes) {
		if (!ModifierVolume) continue;
		if (ModifierVolume->GetBoundingBox().IsInside(Location) && ModifierVolume->bInvalidateCoverLocations)
		{
			return false;
		}
	}
	return true;
}

bool ANav3DVolume::GetEdge(const FVector& Location, FNav3DOctreeEdge& Edge)
{
	if (!IsWithinBounds(Location)) return false;
	
	int32 LayerIndex = NumLayers - 1;
	while (LayerIndex >= 0)
	{
		const TArray<FNav3DOctreeNode>& Layer = GetLayer(LayerIndex);
		FIntVector Voxel;
		GetMortonVoxel(Location, LayerIndex, Voxel);
		const uint_fast64_t MortonCode = morton3D_64_encode(Voxel.X, Voxel.Y, Voxel.Z);
		int32 NodeIndex;
		if (GetNodeIndex(LayerIndex, MortonCode, NodeIndex)) {
			const FNav3DOctreeNode Node = Layer[NodeIndex];

			if (!Node.FirstChild.IsValid())
			{
				Edge.SetLayerIndex(LayerIndex);
				Edge.SetNodeIndex(NodeIndex);
				Edge.SetSubNodeIndex(0);
				return true;
			}

			if (LayerIndex == 0)
			{
				const FNav3DOctreeLeaf Leaf = Octree.Leafs[Node.FirstChild.NodeIndex];
				const float VoxelHalfSize = VoxelHalfSizes[LayerIndex];
				
				FVector NodeLocation;
				GetNodeLocation(LayerIndex, Node.MortonCode, NodeLocation);
				const FVector NodeOrigin = NodeLocation - FVector(VoxelHalfSize);
				const float LeafSize = VoxelHalfSize * 0.5f;
				const float LeafHalfSize = LeafSize * 0.5f;

				bool bFound = false;
				int32 LeafIndex;
				for (LeafIndex = 0; LeafIndex < 64; LeafIndex ++)
				{
					uint_fast32_t x, y, z;
					morton3D_64_decode(LeafIndex, x, y, z);
					const FVector LeafLocation = NodeOrigin + FVector(x * LeafSize, y * LeafSize, z * LeafSize) + FVector(LeafHalfSize);
					if (Location.X >= LeafLocation.X - LeafHalfSize && Location.X <= LeafLocation.X + LeafHalfSize &&
						Location.Y >= LeafLocation.Y - LeafHalfSize && Location.Y <= LeafLocation.Y + LeafHalfSize &&
						Location.Z >= LeafLocation.Z - LeafHalfSize && Location.Z <= LeafLocation.Z + LeafHalfSize) {
						bFound = true;
						break;
					}
				}

				if (!bFound) return false;
				Edge.SetLayerIndex(LayerIndex);
				Edge.SetNodeIndex(NodeIndex);
				if (Leaf.GetSubNode(LeafIndex)) return false;
				Edge.SetSubNodeIndex(LeafIndex);
				return true;
			} 
			LayerIndex = Node.FirstChild.LayerIndex;
		} else if (LayerIndex == 0) {
			break;
		}	
	}
	return false;
}

TArray<FNav3DOctreeEdge> ANav3DVolume::CalculateVolatileEdges(const AActor* Actor) const
{
	TArray<FNav3DOctreeEdge> QueryEdges, VolatileEdges;
	if (!IsValid(Actor)) return VolatileEdges;
	const FBox ActorBounds = Actor->GetComponentsBoundingBox(false).ExpandBy(Clearance);
	QueryEdges.Emplace(VoxelExponent, 0, 0);
	while (QueryEdges.Num() > 0) {
		FNav3DOctreeEdge QueryEdge = QueryEdges.Pop();
		if (!EdgeNodeIsValid(QueryEdge)) continue;
		const FNav3DOctreeNode QueryNode = GetNode(QueryEdge);
		FVector NodeLocation;
		GetNodeLocation(QueryEdge.LayerIndex, QueryNode.MortonCode, NodeLocation);
		FBox NodeBounds = FBox::BuildAABB(NodeLocation, FVector(VoxelHalfSizes[QueryEdge.LayerIndex]));
		if (NodeBounds.Intersect(ActorBounds)) {
			if (QueryNode.HasChildren() && QueryEdge.LayerIndex > 0) {
				for (uint32 I = 0; I < 8; I++) {
					auto ChildEdge = QueryNode.FirstChild;
					ChildEdge.NodeIndex += I;
					QueryEdges.Emplace(ChildEdge);
				}
			} else {
				VolatileEdges.Emplace(QueryEdge);
			}
		}
	}
	return VolatileEdges;
}

void ANav3DVolume::UpdateOctree()
{		

#if WITH_EDITOR
	const auto UpdateStartTime = high_resolution_clock::now();
#endif

	Octree = CachedOctree;

	// Gather the volatile edges from each Occlusion component and reset any cover data
	TSet<FNav3DOctreeEdge> UpdateEdges;
	// Update resets octree to the cached state, so edges from all of the components are required
	for (const auto OcclusionComponent : OcclusionComponents)
	{
		if (!IsValid(OcclusionComponent)) continue;
		
		if (OcclusionComponent->UpdateVolatileEdges()) {		
			UpdateEdges.Append(OcclusionComponent->GetVolatileEdges());
		}
		
		if (bEnableCoverMap) {
			OcclusionComponent->ResetCoverLocations();
		}
	}

	// Gather the morton codes for each updated edge
	TArray<TPair<uint8, uint_fast64_t>> LayerMortonCodes;
	LayerMortonCodes.Reserve(UpdateEdges.Num());
	for (const auto& Edge : UpdateEdges) {
		const FNav3DOctreeNode& Node = GetNode(Edge);
		LayerMortonCodes.Emplace(Edge.LayerIndex, Node.MortonCode);
	}

	// Update each node
	for (const auto& LayerMortonCode : LayerMortonCodes) {
		int32 NodeIndex;
		if (GetNodeIndex(LayerMortonCode.Key, LayerMortonCode.Value, NodeIndex)) {
			UpdateNode(FNav3DOctreeEdge(LayerMortonCode.Key, NodeIndex, 0));
		}
	}

	// Find any nodes that have become isolated
	LayerMortonCodes.Reset();
	for (int32 I = NumLayers - 2; I >= 0; I--) {
		for (int32 J = 0; J < Octree.Layers[I].Num(); J++) {
			FNav3DOctreeNode& Node = Octree.Layers[I][J];
			int32 NodeIndex;
			if (!GetNodeIndex(I + 1, Node.MortonCode >> 3, NodeIndex)) {
				LayerMortonCodes.Emplace(static_cast<uint8>(I), Node.MortonCode);
				J += 7;
			}
		}
	}

	// Remove the isolated nodes that were found
	for (auto& LayerMortonCode : LayerMortonCodes) {
		int32 NodeIndex;
		if (GetNodeIndex(LayerMortonCode.Key, LayerMortonCode.Value, NodeIndex)) {
			const int32 First = NodeIndex - (NodeIndex % 8);
			Octree.Layers[LayerMortonCode.Key].RemoveAt(First, 8);				
			if (LayerMortonCode.Key == 0) Octree.Leafs.RemoveAt(First, 8);
		}
	}
	
	// Repair the node parent edges
	for (int32 I = NumLayers - 2; I >= 0; I--) {
		for (int32 J = 0; J < Octree.Layers[I].Num(); J++) {
			FNav3DOctreeNode& Node = Octree.Layers[I][J];
			int32 NodeIndex;
			if (!GetNodeIndex(I + 1, Node.MortonCode >> 3, NodeIndex)) continue;
			Node.Parent = FNav3DOctreeEdge(I + 1, NodeIndex, 0);
			if (J % 8 == 0) Octree.Layers[I + 1][NodeIndex].FirstChild = FNav3DOctreeEdge(I, J, 0);
		}
	}

	// Repair the node child edges
	for (int32 I = 0; I < Octree.Layers[0].Num(); I++) {
		FNav3DOctreeEdge& Child = Octree.Layers[0][I].FirstChild;
		Child.SetLayerIndex(0);
		Child.SetNodeIndex(I);
		Child.SetSubNodeIndex(0);
	}

	// Rebuild the edges
	for (int32 I = NumLayers - 2; I >= 0; I--) BuildEdges(I);

#if WITH_EDITOR
	const float UpdateDuration = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - UpdateStartTime).count() / 1000.0f;
	//UE_LOG(LogTemp, Warning, TEXT("Update took %f seconds"), UpdateDuration);
#endif

}

void ANav3DVolume::UpdateNode(const FNav3DOctreeEdge Edge)
{
	if (!EdgeNodeIsValid(Edge)) return;
	const FNav3DOctreeNode& Node = GetNode(Edge);

	FVector Location;
	GetNodeLocation(Edge.LayerIndex, Node.MortonCode, Location);

	if (IsOccluded(Location, VoxelHalfSizes[Edge.LayerIndex]))
	{
		if (Edge.LayerIndex == 0)
		{
			UpdateLeaf(Location - FVector(VoxelHalfSizes[0]), Edge.NodeIndex);
		}
		else
		{
			TArray<FNav3DOctreeEdge> QueryEdges;
			QueryEdges.Emplace(Edge);

			while (QueryEdges.Num() > 0)
			{
				FNav3DOctreeEdge CurrentEdge = QueryEdges.Pop();
				if (!EdgeNodeIsValid(CurrentEdge)) continue;
				FNav3DOctreeNode CurrentNode = GetNode(CurrentEdge);
				GetNodeLocation(CurrentEdge.LayerIndex, CurrentNode.MortonCode, Location);
				if (IsOccluded(Location, VoxelHalfSizes[CurrentEdge.LayerIndex])) {
					if (CurrentEdge.LayerIndex == 0) {
						UpdateLeaf(Location - FVector(VoxelHalfSizes[0]), CurrentEdge.NodeIndex);
					} else {
						if (CurrentNode.HasChildren()) {
							for (int32 I = 0; I < 8; I++) {
								FNav3DOctreeEdge& ChildEdge = CurrentNode.FirstChild;
								ChildEdge.NodeIndex += I;
								QueryEdges.Emplace(ChildEdge);
							}
						} else {
							uint8 CurrentLayer = CurrentEdge.LayerIndex - 1;
							const int32 NodeIndex = GetInsertIndex(CurrentLayer, CurrentNode.MortonCode << 3);
							Octree.Layers[CurrentLayer].InsertDefaulted(NodeIndex, 8);
							CurrentNode.FirstChild = FNav3DOctreeEdge(CurrentLayer, NodeIndex, 0);
							for (int32 I = 0; I < 8; I++) {
								auto& ChildNode = Octree.Layers[CurrentLayer][NodeIndex + I];
								ChildNode.Parent = CurrentEdge;
								ChildNode.MortonCode = (CurrentNode.MortonCode << 3) + I;
								QueryEdges.Emplace(CurrentLayer, NodeIndex + I, 0);
							}
							if (CurrentEdge.LayerIndex == 1) {
								Octree.Leafs.InsertDefaulted(NodeIndex, 8);
							}
						}
					}
				}
			}
		}
	}
}

void ANav3DVolume::UpdateLeaf(const FVector& Location, const int32 LeafIndex)
{
	const float VoxelScale = VoxelHalfSizes[0] * 0.5f;
	ParallelFor(64, [&](const int32 I) {
		uint_fast32_t X, Y, Z;
        morton3D_64_decode(I, X, Y, Z);
        const FVector VoxelLocation = Location + VoxelScale * FVector(X, Y, Z) + FVector(VoxelScale * 0.5f);
        if (IsOccluded(VoxelLocation, VoxelScale * 0.5f)) {
            Octree.Leafs[LeafIndex].SetSubNode(I);
        }
	});

	// Perform cover map updates for any dynamic occlusion components
	if (bEnableCoverMap) {
		for (int32 I = 0; I < 64; I++) {
			uint_fast32_t X, Y, Z;
			morton3D_64_decode(I, X, Y, Z);
			const FVector VoxelLocation = Location + VoxelScale * FVector(X, Y, Z) + FVector(VoxelScale * 0.5f);
			// In addition to blocking hit test, retrieve the results of any occlusion overlaps to determine leaf cell normals
			TArray<FOverlapResult> Overlaps;
			if (IsOccluded(VoxelLocation, VoxelScale * 0.5f, Overlaps)) Octree.Leafs[LeafIndex].SetSubNode(I);
			UpdateOcclusionComponentCover(VoxelLocation, Overlaps);	
		}
	} else {
		// If cover map is not in use this can be performed in parallel
		ParallelFor(64, [&](const int32 I) {
	        uint_fast32_t X, Y, Z;
	        morton3D_64_decode(I, X, Y, Z);
	        const FVector VoxelLocation = Location + VoxelScale * FVector(X, Y, Z) + FVector(VoxelScale * 0.5f);
	        if (IsOccluded(VoxelLocation, VoxelScale * 0.5f)) {
	            Octree.Leafs[LeafIndex].SetSubNode(I);
	        }
	    });
	}
}


void ANav3DVolume::RequestOctreeUpdate(UNav3DOcclusionComponent* OcclusionComponent)
{
	if (!OcclusionComponents.Contains(OcclusionComponent)) {
		OcclusionComponents.Add(OcclusionComponent);
	} 
	bUpdateRequested = true;
}

bool ANav3DVolume::GetNodeLocation(const FNav3DOctreeEdge Edge, FVector& Location)
{
	const FNav3DOctreeNode& Node = Octree.Layers[Edge.LayerIndex][Edge.NodeIndex];
	GetNodeLocation(Edge.LayerIndex, Node.MortonCode, Location);
	if (Edge.LayerIndex == 0 && Node.FirstChild.IsValid()) {
		const float Size = VoxelHalfSizes[0] * 2;
		uint_fast32_t X, Y, Z;
		morton3D_64_decode(Edge.SubNodeIndex, X, Y, Z);
		Location += FVector(X * Size / 4, Y * Size / 4, Z * Size / 4) - FVector(Size * 0.375f);
		const FNav3DOctreeLeaf& Leaf = Octree.Leafs[Node.FirstChild.NodeIndex];
		return !Leaf.GetSubNode(Edge.SubNodeIndex);
	}
	return true;
}

#if WITH_EDITOR

void ANav3DVolume::DebugDrawVoxel(const FVector Location, const FVector Extent, const FColor Colour) const {
	if (!InDebugRange(Location)) return;
	DrawDebugBox(GetWorld(), Location, Extent, FQuat::Identity, Colour, true, -1.f, 0, LineScale);		
}

void ANav3DVolume::DebugDrawSphere(const FVector Location, const float Radius, const FColor Colour) const {
	if (!InDebugRange(Location)) return;
	DrawDebugSphere(GetWorld(), Location, Radius, 12, Colour, true, -1.f, 0, LineScale);		
}

void ANav3DVolume::DebugDrawBoundsMesh(const FBox Box, const FColor Colour) const { 
	const TArray<FVector> Vertices = {
		{Box.Min.X, Box.Min.Y, Box.Min.Z}, {Box.Max.X, Box.Min.Y, Box.Min.Z}, {Box.Max.X, Box.Min.Y, Box.Max.Z}, {Box.Min.X, Box.Min.Y, Box.Max.Z},
		{Box.Min.X, Box.Max.Y, Box.Min.Z}, {Box.Max.X, Box.Max.Y, Box.Min.Z}, {Box.Max.X, Box.Max.Y, Box.Max.Z}, {Box.Min.X, Box.Max.Y, Box.Max.Z}};
	const TArray<int32> Indices = { 0, 1, 2, 0, 2, 3, 5, 4, 7, 5, 7, 6, 3, 2, 6, 3, 6, 7, 5, 4, 0, 5, 0, 1, 0, 4, 7, 0, 7, 3, 5, 1, 2, 5, 2, 6};
	DrawDebugMesh(GetWorld(), Vertices, Indices, Colour, true, -1.0, 0);
}

void ANav3DVolume::DebugDrawMortonCode(const FVector Location, const FString String, const FColor Colour) const {
	if (!InDebugRange(Location)) return;

	FVector Start, End;
	const float ScaleFactor = VolumeSize * 0.001f * MortonCodeScale;
	const FVector Scale = FVector(1.f, 3.0f, 6.0f) * ScaleFactor;
	const float Tracking = 6.f * ScaleFactor;

	// Draw the background box
	const FVector Extents = FVector(0, (String.Len() * Scale.Y + (String.Len() - 1) * Tracking) * 0.5f + ScaleFactor * 4, Scale.Z + ScaleFactor * 2);
	const FColor BoxColour = FColor(Colour.R * 0.25f, Colour.G * 0.25f, Colour.B * 0.25f, Colour.A); 
	const TArray<FVector> Vertices = {
		{Location.X + 1.f, Location.Y - Extents.Y, Location.Z + Extents.Z},
		{Location.X + 1.f, Location.Y + Extents.Y, Location.Z + Extents.Z},
		{Location.X + 1.f, Location.Y + Extents.Y, Location.Z - Extents.Z},
		{Location.X + 1.f, Location.Y - Extents.Y, Location.Z - Extents.Z}};
	const TArray<int32> Indices = {0, 1, 2, 0, 2, 3};
	DrawDebugMesh(GetWorld(), Vertices, Indices, BoxColour, true, -1.0, 0);

	// Draw the morton code string with drawn lines, like a calculator interface
	for (int32 I = 0; I < String.Len(); I++) {
		TArray<bool> Layout;
		switch (String[I]) {
			case 0x30: Layout = {true, true, true, false, true, true, true }; break;
			case 0x31: Layout = {false, false, true, false, false, true, false }; break;
			case 0x32: Layout = {true, false, true, true, true, false, true }; break;
			case 0x33: Layout = {true, false, true, true, false, true, true }; break;
			case 0x34: Layout = {false, true, true, true, false, true, false }; break;
			case 0x35: Layout = {true, true, false, true, false, true, true }; break;
			case 0x36: Layout = {true, true, false, true, true, true, true }; break;
			case 0x37: Layout = {true, false, true, false, false, true, false }; break;
			case 0x38: Layout = {true, true, true, true, true, true, true }; break;
			case 0x39: Layout = {true, true, true, true, false, true, true }; break;
			default: Layout = {false, false, false, true, false, false, false }; break;
		}
		for (int32 J = 0; J < 7; J++) {
			if (!Layout[J]) continue;
            switch (J) {
                case 0: Start = FVector(0, -1, 1); End = FVector(0, 1, 1); break;
                case 1: Start = FVector(0, -1, 1); End = FVector(0, -1, 0); break;
                case 2: Start = FVector(0, 1, 1); End = FVector(0, 1, 0); break;
                case 3: Start = FVector(0, -1, 0); End = FVector(0, 1, 0); break;
                case 4: Start = FVector(0, -1, 0); End = FVector(0, -1, -1); break;
                case 5: Start = FVector(0, 1, 0); End = FVector(0, 1, -1); break;
                case 6: Start = FVector(0, -1, -1); End = FVector(0, 1, -1); break;
                default: break;
            }
            const float YOffset = (String.Len() * Scale.Y + (String.Len()-1) * Tracking) * -0.5f + (Scale.Y + Tracking) * I;
            Start = Start * Scale + FVector(0, YOffset, 0) + Location;
            End = End * Scale + FVector(0, YOffset, 0) + Location;
			DrawDebugLine(GetWorld(), Start, End, Colour, true, -1.0f, 0, ScaleFactor);	
		}
	}
}

#endif

void ANav3DVolume::VerifyModifierVolumes() {
	TArray<int32> RemoveVolumes;
	for (int32 I = 0; I < ModifierVolumes.Num(); I++) {
		if (!ModifierVolumes[I]) RemoveVolumes.Add(I);
	}
	if (RemoveVolumes.Num() > 0) {
		for (int32 J = RemoveVolumes.Num()-1; J >= 0; J--) {
			ModifierVolumes.RemoveAt(RemoveVolumes[J]);
        }
	}
}

void ANav3DVolume::AddModifierVolume(ANav3DModifierVolume* ModifierVolume) {
	if (!ModifierVolumes.Contains(ModifierVolume)) {
		ModifierVolumes.Add(ModifierVolume);
	}
}
