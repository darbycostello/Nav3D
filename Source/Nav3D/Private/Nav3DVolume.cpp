#include "Nav3DVolume.h"
#include "Components/BrushComponent.h"
#include "Components/LineBatchComponent.h"
#include "Engine/CollisionProfile.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Public/DrawDebugHelpers.h"
#include "chrono"
#include "Editor.h"
#include "Builders/CubeBuilder.h"
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
}

void ANav3DVolume::OnConstruction(const FTransform &Transform)
{
	Super::OnConstruction(Transform);
	DebugDrawOctree();
}

void ANav3DVolume::Initialise()
{
	Octree.Reset();
	Occluded.Empty();
	DebugEdges.Empty();
	NumBytes = 0;

#if WITH_EDITOR
	FlushDebugDraw();
#endif

	UpdateVolume();
	UpdateModifierVolumes();
}

#if WITH_EDITOR

void ANav3DVolume::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) {
	Super::PostEditChangeProperty(PropertyChangedEvent);
	FProperty* Property = PropertyChangedEvent.Property;
	const FString PropertyName = Property != nullptr ? Property->GetFName().ToString() : "";

	const TSet<FString> CriticalProperties = {
		"VolumeSize",
        "VoxelSize",
		"DefaultLeafLOD"};
	const TSet<FString> DebugProperties = {
		"bDisplayVolumeBounds",
		"VolumeBoundsColor",
		"bDisplayLayers",
		"bDisplayLeafs",
		"bDisplayLeafOcclusion",
		"bDisplayDynamicOcclusion",
		"bDisplayEdgeAdjacency",
		"LineScale",
		"LayerColours",
		"LeafOcclusionColour",
		"EdgeAdjacencyColour",
		"bDisplayMortonCodes",
		"MortonCodeColour",
		"MortonCodeScale" };
	if (CriticalProperties.Contains(PropertyName)) {
		Initialise();
	} else if (DebugProperties.Contains(PropertyName)) {
		FlushDebugDraw();
		DebugDrawOctree();
	}
	UpdateModifierVolumes();
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

void ANav3DVolume::DebugDrawOctree() {
	GetWorld()->PersistentLineBatcher->SetComponentTickEnabled(false);
	DebugDrawVolume();
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
		if (bDisplayLeafOcclusion) DebugDrawOccludedLeafs();
		if (bDisplayDynamicOcclusion) DebugDrawVolatileNodes();
		if (bDisplayEdgeAdjacency) DebugDrawEdgeAdjacency();
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

void ANav3DVolume::DebugDrawOccludedLeafs() {
	
	for (uint_fast32_t I = 0; I < static_cast<uint_fast32_t>(Octree.Leafs.Num()); I++) {

		switch (Octree.Leafs[I].GetLOD())
		{
			// LOD 0
			default:
            for (uint8 J = 0; J < 64; J++) {
            	if (Octree.Leafs[I].GetSubNode(J)) {
            		const FNav3DOctreeEdge Edge { 0, I, J };
            		FVector NodeLocation;
            		GetQuantizedLocation(Edge, 0, NodeLocation);
            		DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[0] * 0.25f), LeafOcclusionColour);
            	}
            }
			break;

			case 1:
	        for (uint8 J = 0; J < 8; J++) {
        		const int32 Index = J * 8;
        		if (Octree.Leafs[I].GetSubNode(Index)) {
        			const FNav3DOctreeEdge Edge { 0, I, static_cast<uint8>(Index) };
        			FVector NodeLocation;
        			GetQuantizedLocation(Edge, 1, NodeLocation);
        			DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[0] * 0.5f), LeafOcclusionColour);
        		}
	        }
			break;

			case 2:
	        if (Octree.Leafs[I].GetSubNode(0)) {
        		const FNav3DOctreeEdge Edge { 0, I, 0 };
        		FVector NodeLocation;
	        	GetQuantizedLocation(Edge, 2, NodeLocation);
        		DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[0]), LeafOcclusionColour);
	        }
			break;
		}
	}
}

void ANav3DVolume::DebugDrawVolatileNodes() {
	TSet<uint8> VolatileNodeMortonCodes;
	for (UNav3DOcclusionComponent* OcclusionComponent: DirtyOcclusionComponents) {
		if (!IsValid(OcclusionComponent)) continue;
		TArray<FNav3DOctreeEdge> VolatileEdges = OcclusionComponent->GetVolatileEdges();
		if (VolatileEdges.Num() > 0) {
			for (FNav3DOctreeEdge VolatileEdge : VolatileEdges) {
				const FNav3DOctreeNode VolatileNode = GetNode(VolatileEdge);
				if (!VolatileNodeMortonCodes.Contains(VolatileNode.MortonCode)) {
					FVector NodeLocation;
					GetNodeLocation(VolatileEdge, NodeLocation);
					DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[VolatileEdge.LayerIndex]), DynamicOcclusionColour);
					VolatileNodeMortonCodes.Add(VolatileNode.MortonCode);
				}
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

#endif

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

// Regenerate the entire sparse voxel octree
bool ANav3DVolume::BuildOctree() {
	Initialise();
	
#if WITH_EDITOR
	const auto StartTime = high_resolution_clock::now();
#endif

	
	RasterizeInitial();
	Octree.Leafs.AddDefaulted(Occluded[0].Num() * 8 * 0.25f);
	for (int32 I = 0; I < NumLayers; I++) RasterizeLayer(I);
	for (int32 I = NumLayers - 2; I >= 0; I--) BuildEdges(I);
	NumBytes = Octree.GetSize();
	
#if WITH_EDITOR
	const float Duration = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - StartTime).count() / 1000.0f;
	int32 NumNodes = 0;
	for (int32 I = 0; I < NumLayers; I++) NumNodes += Octree.Layers[I].Num();
	UE_LOG(LogTemp, Display, TEXT("Generation Time : %f seconds"), Duration);
	UE_LOG(LogTemp, Display, TEXT("Desired Volume Size : %fcm"), VolumeSize);
	UE_LOG(LogTemp, Display, TEXT("Actual Volume Size : %fcm"), ActualVolumeSize);
	UE_LOG(LogTemp, Display, TEXT("Voxel Size : %fcm"), VoxelSize);
	UE_LOG(LogTemp, Display, TEXT("Voxel Exponent: %i"), VoxelExponent);
	UE_LOG(LogTemp, Display, TEXT("Total Layers : %i"), NumLayers);
	UE_LOG(LogTemp, Display, TEXT("Total Nodes : %i"), NumNodes);
	UE_LOG(LogTemp, Display, TEXT("Total Leafs : %i"), Octree.Leafs.Num());
	UE_LOG(LogTemp, Display, TEXT("Total Bytes : %i"), NumBytes);
	DebugDrawOctree();
#endif
	
	return true;
}

void ANav3DVolume::UpdateModifierVolumes()
{
	ModifierVolumes.Empty();
	TArray<AActor*> ModifierVolumeActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANav3DModifierVolume::StaticClass(), ModifierVolumeActors);
	for (auto& Actor: ModifierVolumeActors)
	{
		ANav3DModifierVolume* ModiferVolume = Cast<ANav3DModifierVolume>(Actor);

#if WITH_EDITOR		
		// Initialise each modifier volume in editor, to perform persistent line debug drawing
		ModiferVolume->Initialise();
#endif
		
		if (ModiferVolume->GetEnabled()) {
			if (ModiferVolume->GetBoundingBox().Intersect(GetBoundingBox()))
			{
				ModifierVolumes.Add(ModiferVolume);
			}
		}
	}
}

FBox ANav3DVolume::GetBoundingBox() const
{
	const FBoxSphereBounds Bounds = GetBrushComponent()->CalcBounds(ActorToWorld());
	return Bounds.GetBox();
}

uint8 ANav3DVolume::GetLocationLOD(const FVector WorldLocation)
{
	if (ModifierVolumes.Num() == 0) return DefaultLeafLOD;
	TArray<int32> Priorities;
	for (auto& ModifierVolume: ModifierVolumes) {
		if (ModifierVolume->GetComponentsBoundingBox(true).IsInside(WorldLocation))
		{
			Priorities.Add(ModifierVolume->GetPriority());
		}
	}
	if (Priorities.Num() == 0) return DefaultLeafLOD;
	if (Priorities.Num() == 1) return ModifierVolumes[0]->GetLOD();
	int32 MaxIndex, MaxValue;
	UKismetMathLibrary::MaxOfIntArray(Priorities, MaxIndex, MaxValue);
	return ModifierVolumes[MaxIndex]->GetLOD();
}

bool ANav3DVolume::GetNodeLocation(const uint8 LayerIndex, const uint_fast64_t MortonCode, FVector& Location) const
{
	const float Scale = VoxelHalfSizes[LayerIndex] * 2;
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);
	Location = VolumeOrigin - VolumeExtent + FVector(X * Scale, Y * Scale, Z * Scale) + FVector(Scale * 0.5f);
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
	return Edge.IsValid() && static_cast<int32>(Edge.NodeIndex) < Octree.Layers[Edge.LayerIndex].Num();
}

void ANav3DVolume::GetAdjacentLeafs(const FNav3DOctreeEdge& Edge, TArray<FNav3DOctreeEdge>& AdjacentEdges) const
{
	const uint_fast64_t LeafIndex = Edge.SubNodeIndex;
	if (EdgeNodeIsValid(Edge)) {
		const FNav3DOctreeNode& Node = GetNode(Edge);
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

void ANav3DVolume::GetAdjacentEdges(const FNav3DOctreeEdge& Edge, TArray<FNav3DOctreeEdge>& AdjacentEdges) const
{
	const FNav3DOctreeNode& Node = GetNode(Edge);
	for (int32 I = 0; I < 6; I++) {
		const FNav3DOctreeEdge& AdjacentEdge = Node.AdjacentEdges[I];
		if (!AdjacentEdge.IsValid()) continue;
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
	Ar << VolumeExtent;
	NumBytes = Octree.GetSize();
}

float ANav3DVolume::GetVoxelScale(const uint8 LayerIndex) const {
	return VolumeExtent.X / FMath::Pow(2.0f, VoxelExponent) * FMath::Pow(2.0f, LayerIndex + 1);
}

int32 ANav3DVolume::GetLayerNodeCount(const uint8 LayerIndex) const {
	return FMath::Pow(8, VoxelExponent - LayerIndex);
}

int32 ANav3DVolume::GetSegmentNodeCount(const uint8 LayerIndex) const {
	return FMath::Pow(2, VoxelExponent - LayerIndex);
}

void ANav3DVolume::BeginPlay() {
}

void ANav3DVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (DirtyOcclusionComponents.Num() > 0) {
		
#if WITH_EDITOR
		const auto UpdateStartTime = high_resolution_clock::now();
#endif

		FlushDebugDraw();
		UpdateOctree();

#if WITH_EDITOR
		const float UpdateDuration = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - UpdateStartTime).count() / 1000.0f;
		if (bDebugLogging) UE_LOG(LogTemp, Warning, TEXT("Octree updated in %f seconds"), UpdateDuration);
		DebugDrawOctree();
#endif

		DirtyOcclusionComponents.Reset();
	}
}

void ANav3DVolume::PostRegisterAllComponents() {
	Super::PostRegisterAllComponents();
}

void ANav3DVolume::PostUnregisterAllComponents() {
	Super::PostUnregisterAllComponents();
}

void ANav3DVolume::BuildEdges(const uint8 LayerIndex) {
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

bool ANav3DVolume::IsOccluded(const FVector& Location, const float Size) const
{
	FCollisionQueryParams CollisionQueryParams;
	CollisionQueryParams.bFindInitialOverlaps = true;
	CollisionQueryParams.bTraceComplex = false;
	CollisionQueryParams.TraceTag = "Nav3DRasterize";
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

	for (int32 I = 0; I < NumLayers; I++) {
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

				const uint8 LOD = GetLocationLOD(NodeLocation);
				
				if (IsOccluded(NodeLocation, VoxelHalfSizes[0])) {
					RasterizeLeaf(NodeLocation, LeafIndex, LOD);
					NewNode.FirstChild.SetLayerIndex(0);
					NewNode.FirstChild.SetNodeIndex(LeafIndex);
					NewNode.FirstChild.SetSubNodeIndex(0);
					LeafIndex++;
				} else {
					Octree.Leafs.AddDefaulted(1);
					LeafIndex++;
					NewNode.FirstChild.Invalidate();
				}
				Octree.Leafs[LeafIndex - 1].SetLOD(LOD);
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

void ANav3DVolume::RasterizeLeaf(const FVector NodeLocation, const int32 LeafIndex, const uint8 LOD)
{
	FVector Location;
	float VoxelScale;
	switch (LOD)
	{
		// LOD 0
		default:
        Location = NodeLocation - VoxelHalfSizes[0];
		VoxelScale = VoxelHalfSizes[0] * 0.5f;
		ParallelFor(64, [&](const int32 I) {
            uint_fast32_t X, Y, Z;
            morton3D_64_decode(I, X, Y, Z);
            const FVector VoxelLocation = Location + FVector(X * VoxelScale, Y * VoxelScale, Z * VoxelScale) + VoxelScale * 0.5f;
            if (LeafIndex >= Octree.Leafs.Num() - 1) Octree.Leafs.AddDefaulted(1);
            if (IsOccluded(VoxelLocation, VoxelScale * 0.5f)) Octree.Leafs[LeafIndex].SetSubNode(I);
        });
		break;
		
		// LOD 1
		case 1:
        Location = NodeLocation - VoxelHalfSizes[0] * 2.f;
		VoxelScale = VoxelHalfSizes[0];
		ParallelFor(8, [&](const int32 I) {
            uint_fast32_t X, Y, Z;
            const int32 Index = I * 8;
            morton3D_64_decode(Index, X, Y, Z);
            const FVector VoxelLocation = Location + FVector(X * VoxelScale, Y * VoxelScale, Z * VoxelScale) + FVector(VoxelScale * 0.5f);
            if (LeafIndex >= Octree.Leafs.Num() - 1) Octree.Leafs.AddDefaulted(1);
            if (IsOccluded(VoxelLocation, VoxelScale)) Octree.Leafs[LeafIndex].SetSubNode(Index);
        });
		break;

		// LOD 2
		case 2:
        if (LeafIndex >= Octree.Leafs.Num() - 1) Octree.Leafs.AddDefaulted(1);
		Octree.Leafs[LeafIndex].SetSubNode(0);
		break;		
	}
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

bool ANav3DVolume::GetEdge(const FVector& Location, FNav3DOctreeEdge& Edge)
{
	if (!IsWithinBounds(Location)) {
		return false;
	}
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
				const float LeafSize = VoxelHalfSize * 0.5f;

				FVector NodeLocation;
				GetNodeLocation(LayerIndex, Node.MortonCode, NodeLocation);
				const FVector NodeOrigin = NodeLocation - FVector(VoxelHalfSize);

				bool bFound = false;
				int32 LeafIndex;
				for (LeafIndex = 0; LeafIndex < 64; ++LeafIndex)
				{
					uint_fast32_t x, y, z;
					morton3D_64_decode(LeafIndex, x, y, z);
					const FVector LeafLocation = NodeOrigin + FVector(x * LeafSize, y * LeafSize, z * LeafSize) + FVector(LeafSize * 0.5f);
					if (Location.X >= LeafLocation.X - LeafSize * 0.5f && Location.X <= LeafLocation.X + LeafSize * 0.5f &&
						Location.Y >= LeafLocation.Y - LeafSize * 0.5f && Location.Y <= LeafLocation.Y + LeafSize * 0.5f &&
						Location.Z >= LeafLocation.Z - LeafSize * 0.5f && Location.Z <= LeafLocation.Z + LeafSize * 0.5f) {
						bFound = true;
						break;
					}
				}

				if (!bFound)
				{
					UE_LOG(LogTemp, Warning, TEXT("Get Edge: could not find node"));
					return false;
				}

				Edge.SetLayerIndex(LayerIndex);
				Edge.SetNodeIndex(NodeIndex);

				if (Leaf.GetSubNode(LeafIndex))
				{
#if WITH_EDITOR
					UE_LOG(LogTemp, Warning, TEXT("Get Edge: Node at location %s is occluded."), *Location.ToString());
#endif
					return false;
				}

				Edge.SetSubNodeIndex(LeafIndex);
				return true;
			} 
			LayerIndex = Node.FirstChild.LayerIndex;
		} else if (LayerIndex == 0) {
			break;
		}	
	}

#if WITH_EDITOR
	UE_LOG(LogTemp, Warning, TEXT("Get Edge: could not find a node for the provided edge"));
#endif

	return false;
}

TArray<FNav3DOctreeEdge> ANav3DVolume::CalculateVolatileEdges(const AActor* Actor) const
{
	TArray<FNav3DOctreeEdge> QueryEdges, VolatileEdges;
	if (!IsValid(Actor)) return VolatileEdges;
	const FBox ActorBounds = Actor->GetComponentsBoundingBox(false).ExpandBy(Clearance);
	QueryEdges.Emplace(NumLayers, 0, 0);
	while (QueryEdges.Num() > 0) {
		FNav3DOctreeEdge QueryEdge = QueryEdges.Pop();
		const FNav3DOctreeNode QueryNode = GetNode(QueryEdge);
		if (QueryNode.IsDefault()) continue;
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
	TSet<FNav3DOctreeEdge> UpdateEdges;
	for (const auto OcclusionComponent : DirtyOcclusionComponents)
	{
		if (!IsValid(OcclusionComponent)) continue;

		// Get the last volatile edges found by this component
		UpdateEdges.Append(OcclusionComponent->GetVolatileEdges());

		// Update the volatile edges
		if (OcclusionComponent->UpdateVolatileEdges()) {
			
			// Append the updated volatile edges to the update set
			UpdateEdges.Append(OcclusionComponent->GetVolatileEdges());
		}
	}

	TArray<TPair<uint8, uint_fast64_t>> LayerMortonCodes;
	LayerMortonCodes.Reserve(UpdateEdges.Num());
	for (const auto& Edge : UpdateEdges.Array()) {
		const FNav3DOctreeNode Node = GetNode(Edge);
		LayerMortonCodes.Emplace(Edge.LayerIndex, Node.MortonCode);
	}

	for (const auto& LayerMortonCode : LayerMortonCodes) {
		int32 NodeIndex;
		if (GetNodeIndex(LayerMortonCode.Key, LayerMortonCode.Value, NodeIndex)) {
			UpdateNode(FNav3DOctreeEdge(LayerMortonCode.Key, NodeIndex, 0));
		}
	}

	// Find any orphaned nodes
	LayerMortonCodes.Reset();
	for (int32 I = NumLayers - 2; I >= 0; I--) {
		for (int32 J = 0; J < Octree.Layers[I].Num(); ++J) {
			FNav3DOctreeNode& Node = Octree.Layers[I][J];
			int32 NodeIndex;
			if (!GetNodeIndex(I + 1, Node.MortonCode >> 3, NodeIndex)) {
				LayerMortonCodes.Emplace(static_cast<uint8>(I), Node.MortonCode);
				J += 7;
			}
		}
	}

	// Remove the orphaned nodes found
	for (auto& LayerMortonCode : LayerMortonCodes) {
		int32 NodeIndex;
		if (GetNodeIndex(LayerMortonCode.Key, LayerMortonCode.Value, NodeIndex)) {
			const int32 First = NodeIndex - (NodeIndex % 8);
			Octree.Layers[LayerMortonCode.Key].RemoveAt(First, 8);				
			if (LayerMortonCode.Key == 0) Octree.Leafs.RemoveAt(First, 8);
		}
	}

	// Repair the node parents
	for (int32 I = NumLayers - 2; I >= 0; I--) {
		for (int32 J = 0; J < Octree.Layers[I].Num(); J++) {
			FNav3DOctreeNode& Node = Octree.Layers[I][J];
			int32 NodeIndex;
			if (!GetNodeIndex(I + 1, Node.MortonCode >> 3, NodeIndex)) continue;
			Node.Parent = FNav3DOctreeEdge(I + 1, NodeIndex, 0);
			if (J % 8 == 0) Octree.Layers[I + 1][NodeIndex].FirstChild = FNav3DOctreeEdge(I, J, 0);
		}
	}

	// Repair the edges
	for (int32 I = 0; I < Octree.Layers[0].Num(); I++) {
		FNav3DOctreeEdge& Child = Octree.Layers[0][I].FirstChild;
		Child.SetLayerIndex(0);
		Child.SetNodeIndex(I);
		Child.SetSubNodeIndex(0);
	}

	// Rebuild the edges
	for (int32 I = NumLayers - 2; I >= 0; I--) BuildEdges(I);

	// Update each occlusion component's volatile edges
	for (auto OcclusionComponent : DirtyOcclusionComponents) {
		if (!IsValid(OcclusionComponent)) continue;
		OcclusionComponent->UpdateVolatileEdges();
	}
}

void ANav3DVolume::AddDirtyOcclusionComponent(UNav3DOcclusionComponent* OcclusionComponent)
{
	DirtyOcclusionComponents.AddUnique(OcclusionComponent);
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

bool ANav3DVolume::GetQuantizedLocation(const FNav3DOctreeEdge Edge, const uint8 LOD, FVector& Location)
{
	// Set the scalar for the node location offset. Used to correctly place voxels during debug drawing 
	float Scalar;
	switch (LOD)
	{
		default: Scalar = 0.375f; break; // LOD 0
		case 1: Scalar = 0.25f; break; // LOD 1
		case 2: Scalar = 0.f; break; // LOD 2
	}
	const FNav3DOctreeNode& Node = Octree.Layers[Edge.LayerIndex][Edge.NodeIndex];
	GetNodeLocation(Edge.LayerIndex, Node.MortonCode, Location);
	if (Edge.LayerIndex == 0 && Node.FirstChild.IsValid()) {
		const float Size = VoxelHalfSizes[0] * 2;
		uint_fast32_t X, Y, Z;
		morton3D_64_decode(Edge.SubNodeIndex, X, Y, Z);
		Location += FVector(X * Size / 4, Y * Size / 4, Z * Size / 4) - FVector(Size * Scalar);
		const FNav3DOctreeLeaf& Leaf = Octree.Leafs[Node.FirstChild.NodeIndex];
		return !Leaf.GetSubNode(Edge.SubNodeIndex);
	}
	return true;
}

void ANav3DVolume::UpdateNode(const FNav3DOctreeEdge Edge)
{
	const FNav3DOctreeNode& Node = GetNode(Edge);
	if (Node.IsDefault()) return;

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
				FNav3DOctreeNode CurrentNode = GetNode(CurrentEdge);
				GetNodeLocation(CurrentEdge.LayerIndex, CurrentNode.MortonCode, Location);
				if (IsOccluded(Location, VoxelHalfSizes[CurrentEdge.LayerIndex])) {
					if (CurrentEdge.LayerIndex == 0) {
						UpdateLeaf(Location - FVector(VoxelHalfSizes[0]), CurrentEdge.NodeIndex);
					} else {
						if (CurrentNode.HasChildren()) {
							for (int32 I = 0; I < 8; I++) {
								FNav3DOctreeEdge ChildEdge = CurrentNode.FirstChild;
								ChildEdge.NodeIndex += I;
								QueryEdges.Emplace(ChildEdge);
							}
						} else  {
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
	else if (Node.Parent.IsValid())
	{
		// Remove this node and its neighbours if clear
		TArray<TPair<uint8, uint_fast64_t>> ChildLayerMortonCodes;

		int32 NodeIndex;
		if (GetNodeIndex(Edge.LayerIndex + 1, Node.MortonCode >> 3, NodeIndex))
		{
			FNav3DOctreeEdge ParentEdge = FNav3DOctreeEdge(Edge.LayerIndex + 1, NodeIndex, 0);
			FNav3DOctreeNode ParentNode = GetNode(ParentEdge);
			GetNodeLocation(ParentEdge.LayerIndex, ParentNode.MortonCode, Location);
			while (!IsOccluded(Location, VoxelHalfSizes[ParentEdge.LayerIndex])) {
				const FNav3DOctreeEdge ChildEdge = ParentNode.FirstChild;
				if (!ChildEdge.IsValid()) break;

				const FNav3DOctreeNode ChildNode = GetNode(ChildEdge);
				if (ChildNode.IsDefault()) break;

				ChildLayerMortonCodes.Emplace(static_cast<uint8>(ChildEdge.LayerIndex), ChildNode.MortonCode);
				ParentNode.FirstChild.Invalidate();
				if (!GetNodeIndex(ParentEdge.LayerIndex + 1, ParentNode.MortonCode >> 3, NodeIndex)) break;

				ParentEdge = FNav3DOctreeEdge(ParentEdge.LayerIndex + 1, NodeIndex, 0);
				if (!ParentEdge.IsValid()) break;
				
				ParentNode = GetNode(ParentEdge);
				GetNodeLocation(ParentEdge.LayerIndex, ParentNode.MortonCode, Location);
			}

			for (auto& ChildLayerMortonCode : ChildLayerMortonCodes) {
				int32 ChildNodeIndex;
				if (!GetNodeIndex(ChildLayerMortonCode.Key, ChildLayerMortonCode.Value, ChildNodeIndex)) continue;

				Octree.Layers[ChildLayerMortonCode.Key].RemoveAt(ChildNodeIndex, 8);
				if (ChildLayerMortonCode.Key == 0) Octree.Leafs.RemoveAt(ChildNodeIndex, 8);
			}
		} else {
			const int32 FirstIndex = Edge.NodeIndex - (Edge.NodeIndex % 8);
			Octree.Layers[Edge.LayerIndex].RemoveAt(FirstIndex, 8);
			if (Edge.LayerIndex == 0)
			{
				Octree.Leafs.RemoveAt(FirstIndex, 8);
			}
		}
	}
}

void ANav3DVolume::UpdateLeaf(const FVector& Location, const int32 LeafIndex)
{
	FNav3DOctreeLeaf Leaf;
	const float LeafSize = VoxelHalfSizes[0] * 0.5f;
	const float LeafHalfSize = LeafSize * 0.5f;
	ParallelFor(64, [&](const int32 I) {
		uint_fast32_t X, Y, Z;
        morton3D_64_decode(I, X, Y, Z);
        const FVector LeafLocation = Location + FVector(X * LeafSize, Y * LeafSize, Z * LeafSize) + FVector(LeafHalfSize);
        if (IsOccluded(LeafLocation, LeafHalfSize)) {
            Leaf.SetSubNode(I);
        }
	});
	Octree.Leafs[LeafIndex] = Leaf;
}

void ANav3DVolume::DebugDrawVoxel(const FVector Location, const FVector Extent, const FColor Colour) const {
	if (!InDebugRange(Location)) return;
	DrawDebugBox(GetWorld(), Location, Extent, FQuat::Identity, Colour, true, -1.f, 0, LineScale);		
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

