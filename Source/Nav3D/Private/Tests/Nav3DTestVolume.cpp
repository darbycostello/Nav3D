
#include "Tests/Nav3DTestVolume.h"

#include "EngineUtils.h"
#include "Nav3D.h"
#include "Nav3DData.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DTypes.h"
#include "UObject/ConstructorHelpers.h"
#include "AI/Navigation/NavigationTypes.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Engine/World.h"

ANav3DTestVolume::ANav3DTestVolume()
{
    PrimaryActorTick.bCanEverTick = false;
    
    // Create box component to define the volume
    VolumeBox = CreateDefaultSubobject<UBoxComponent>(TEXT("VolumeBox"));
    RootComponent = VolumeBox;
    VolumeBox->SetCollisionProfileName(TEXT("NoCollision"));
    VolumeBox->SetBoxExtent(FVector(500.0f));
    VolumeBox->SetHiddenInGame(false);
    VolumeBox->bDrawOnlyIfSelected = true;
    
    // Create instanced static mesh component for obstacles
    ObstacleMeshes = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("ObstacleMeshes"));
    ObstacleMeshes->SetupAttachment(RootComponent);
    ObstacleMeshes->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    ObstacleMeshes->SetCollisionResponseToAllChannels(ECR_Block);
    ObstacleMeshes->SetGenerateOverlapEvents(true);
    ObstacleMeshes->CanCharacterStepUpOn = ECB_No;
    ObstacleMeshes->SetCanEverAffectNavigation(true);
    
    // Create spline component for spline distribution
    ObstacleSpline = CreateDefaultSubobject<USplineComponent>(TEXT("ObstacleSpline"));
    ObstacleSpline->SetupAttachment(RootComponent);
    ObstacleSpline->SetVisibility(false);
    
    // Try to find default sphere mesh
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMeshFinder(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    if (SphereMeshFinder.Succeeded())
    {
        ObstacleMesh = SphereMeshFinder.Object;
        ObstacleMeshes->SetStaticMesh(ObstacleMesh);
    }
    
    // Set default values
    OcclusionPercentage = 30.0f;
    DistributionType = ENav3DTestDistribution::Uniform;
    
    // Clustered distribution defaults
    ClusterCount = 5;
    ClusterRadius = 200.0f;

    // Perlin noise defaults
    NoiseScale = 0.1f;
    
    // Ring distribution defaults
    RingRadius = 300.0f;
    RingThickness = 50.0f;
    RingCount = 3;
    RingVerticalSpacing = 100.0f;
    
    // Disc distribution defaults
    DiscRadius = 350.0f;
    DiscOrientation = ENav3DDiscOrientation::XY;
    DiscDensity = 0.8f;
    
    // Spline distribution defaults
    SplineRadius = 100.0f;
    SplineLength = 1000.0f;
    SplinePointCount = 8;
    SplinePointSpacing = 100.0f;
    bRandomizeSpline = true;
    
    // Common defaults
    MinObstacleSize = 20.0f;
    MaxObstacleSize = 100.0f;
    MaxObstacles = 500;
    RandomSeed = 0;
    bAutoGenerate = false;
}

void ANav3DTestVolume::BeginPlay()
{
    Super::BeginPlay();
    
    if (bAutoGenerate)
    {
        GenerateObstacles();
    }
}

void ANav3DTestVolume::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    
    // Apply the selected mesh
    if (ObstacleMesh)
    {
        ObstacleMeshes->SetStaticMesh(ObstacleMesh);
    }
    
    // Show/hide spline based on selected distribution type
    ObstacleSpline->SetVisibility(DistributionType == ENav3DTestDistribution::Spline);
    
    // If auto-generate is enabled, generate obstacles when the actor is constructed in the editor
    if (bAutoGenerate)
    {
        GenerateObstacles();
    }
}

#if WITH_EDITOR
void ANav3DTestVolume::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    
    if (PropertyChangedEvent.Property)
    {
        const FName PropertyName = PropertyChangedEvent.Property->GetFName();
        
        // If mesh was changed, update the component
        if (PropertyName == GET_MEMBER_NAME_CHECKED(ANav3DTestVolume, ObstacleMesh))
        {
            if (ObstacleMesh)
            {
                ObstacleMeshes->SetStaticMesh(ObstacleMesh);
            }
        }
        // If distribution type was changed, show/hide spline component
        else if (PropertyName == GET_MEMBER_NAME_CHECKED(ANav3DTestVolume, DistributionType))
        {
            ObstacleSpline->SetVisibility(DistributionType == ENav3DTestDistribution::Spline);
            
            // Generate random spline if type is set to spline and randomize is enabled
            if (DistributionType == ENav3DTestDistribution::Spline && bRandomizeSpline)
            {
                // Create a random stream with the current seed
                const FRandomStream RandomStream(RandomSeed);
                GenerateRandomSpline(RandomStream);
            }
        }
        // If randomize spline was toggled and is now true
        else if (PropertyName == GET_MEMBER_NAME_CHECKED(ANav3DTestVolume, bRandomizeSpline) && bRandomizeSpline)
        {
            if (DistributionType == ENav3DTestDistribution::Spline)
            {
                // Create a random stream with the current seed
                const FRandomStream RandomStream(RandomSeed);
                GenerateRandomSpline(RandomStream);
            }
        }
        // If auto-generate is enabled, regenerate obstacles
        else if (PropertyName == GET_MEMBER_NAME_CHECKED(ANav3DTestVolume, bAutoGenerate))
        {
            if (bAutoGenerate)
            {
                GenerateObstacles();
            }
        }
        // If random seed is changed, regenerate obstacles if auto-generate is enabled
        else if (PropertyName == GET_MEMBER_NAME_CHECKED(ANav3DTestVolume, RandomSeed))
        {
            if (bAutoGenerate)
            {
                GenerateObstacles();
            }
        }
    }
}
#endif

void ANav3DTestVolume::GenerateObstacles() const
{
    // Clear any existing obstacles first
    ClearObstacles();
    
    // Create a random stream with the specified seed
    const FRandomStream RandomStream(RandomSeed);
    
    // Make sure we have a mesh assigned
    if (!ObstacleMesh)
    {
        UE_LOG(LogNav3D, Warning, TEXT("Cannot generate obstacles: No obstacle mesh assigned"));
        return;
    }
    
    // Generate obstacles based on distribution type
    switch (DistributionType)
    {
    case ENav3DTestDistribution::Uniform:
        GenerateUniformDistribution(RandomStream);
        break;
            
    case ENav3DTestDistribution::Clustered:
        GenerateClusteredDistribution(RandomStream);
        break;
            
    case ENav3DTestDistribution::PerlinNoise:
        GeneratePerlinNoiseDistribution(RandomStream);
        break;
            
    case ENav3DTestDistribution::Ring:
        GenerateRingDistribution(RandomStream);
        break;
            
    case ENav3DTestDistribution::Disc:
        GenerateDiscDistribution(RandomStream);
        break;
            
    case ENav3DTestDistribution::Spline:
        GenerateSplineDistribution(RandomStream);
        break;
            
    default:
        break;
    }
   
    // Log actual occlusion percentage (approximated)
    UE_LOG(LogNav3D, Log, TEXT("Nav3DTestVolume: Generated %d obstacles. Approximate occlusion: %.1f%% (target: %.1f%%)"), 
        GetObstacleCount(), CalculateActualOcclusionPercentage(), OcclusionPercentage);
}

void ANav3DTestVolume::ClearObstacles() const
{
    ObstacleMeshes->ClearInstances();
    ObstacleMeshes->MarkRenderStateDirty();
}

int32 ANav3DTestVolume::GetObstacleCount() const
{
    return ObstacleMeshes->GetInstanceCount();
}

void ANav3DTestVolume::GenerateUniformDistribution(const FRandomStream& RandomStream) const
{
    // Get volume dimensions in world space
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const FVector VolumeMin = GetActorLocation() - BoxExtent;
    const FVector VolumeMax = GetActorLocation() + BoxExtent;
    
    // Calculate total volume
    const float TotalVolume = BoxExtent.X * BoxExtent.Y * BoxExtent.Z * 8.0f;
    
    // Calculate average obstacle size
    const float AvgObstacleSize = (MinObstacleSize + MaxObstacleSize) * 0.5f;
    const float AvgObstacleVolume = (4.0f/3.0f) * PI * FMath::Pow(AvgObstacleSize, 3.0f);
    
    // Calculate approximate number of obstacles to place (limited by MaxObstacles)
    const int32 NumObstacles = FMath::Min(
        FMath::CeilToInt((OcclusionPercentage / 100.0f) * TotalVolume / AvgObstacleVolume),
        MaxObstacles
    );
    
    // Place obstacles
    int32 PlacementAttempts = 0;
    const int32 MaxAttempts = NumObstacles * 5;  // Allow multiple attempts per obstacle
    int32 PlacedObstacles = 0;
    
    while (PlacedObstacles < NumObstacles && PlacementAttempts < MaxAttempts)
    {
        // Generate random position in world space
        FVector WorldPosition = RandomStream.RandPointInBox(FBox(VolumeMin, VolumeMax));
        
        // Convert to local space for instance placement
        FVector LocalPosition = WorldToLocalPosition(WorldPosition);
        
        // Generate random size
        const float Size = RandomStream.FRandRange(MinObstacleSize, MaxObstacleSize);
        
        // Check if position is already occupied
        if (!IsPointOccluded(WorldPosition, Size))
        {
            // Add instance using local coordinates
            FTransform Transform;
            Transform.SetLocation(LocalPosition);
            Transform.SetScale3D(FVector(Size / 50.0f));  // Assuming sphere mesh has radius 50
            ObstacleMeshes->AddInstance(Transform, /*bWorldSpace=*/false);
            
            PlacedObstacles++;
        }
        
        PlacementAttempts++;
    }
    
    // Mark component dirty to update render state
    ObstacleMeshes->MarkRenderStateDirty();
}

void ANav3DTestVolume::GenerateRandomSpline() const
{
    const FRandomStream RandomStream(RandomSeed);
    GenerateRandomSpline(RandomStream);
}

void ANav3DTestVolume::GenerateRandomSpline(const FRandomStream& RandomStream) const
{
    // Clear existing spline points
    ObstacleSpline->ClearSplinePoints(false);
    
    // Get volume dimensions
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const FVector VolumeMin = GetActorLocation() - BoxExtent * 0.8f; // Slightly inset to keep points inside
    const FVector VolumeMax = GetActorLocation() + BoxExtent * 0.8f;
    
    // Generate random start point
    FVector CurrentWorldPoint = RandomStream.RandPointInBox(FBox(VolumeMin, VolumeMax));
    
    // Add initial spline point
    ObstacleSpline->AddSplinePoint(CurrentWorldPoint, ESplineCoordinateSpace::World, false);
    
    // Add additional random points with some coherence in direction
    FVector Direction = RandomStream.VRand(); // Start with a random direction
    
    // Calculate spacing between points based on desired SplineLength and point count
    const float TargetSpacing = SplineLength / (SplinePointCount - 1);
    
    for (int32 i = 1; i < SplinePointCount; i++)
    {
        // Generate a new direction with some randomness but keeping continuity
        Direction = (Direction + RandomStream.VRand() * 0.5f).GetSafeNormal();
        
        // Calculate new point in that direction using the target spacing
        FVector NewWorldPoint = CurrentWorldPoint + Direction * TargetSpacing;
        
        // Make sure the point stays within the volume
        ClampPointToVolumeBox(NewWorldPoint);
        
        // Add the point to the spline
        ObstacleSpline->AddSplinePoint(NewWorldPoint, ESplineCoordinateSpace::World, false);
        
        // Update current point
        CurrentWorldPoint = NewWorldPoint;
    }
    
    // Update the spline
    ObstacleSpline->UpdateSpline();
    
    // Log the actual spline length - might differ from target due to clamping
    const float ActualLength = ObstacleSpline->GetSplineLength();
    UE_LOG(LogNav3D, Log, TEXT("Spline Generation: Target Length=%.1f, Actual Length=%.1f, Points=%d"), 
        SplineLength, ActualLength, ObstacleSpline->GetNumberOfSplinePoints());
}

// Fix for the Clustered distribution to work with different seeds
void ANav3DTestVolume::GenerateClusteredDistribution(const FRandomStream& RandomStream) const
{
    // Get volume dimensions
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const FVector VolumeMin = GetActorLocation() - BoxExtent;
    const FVector VolumeMax = GetActorLocation() + BoxExtent;
    
    // Calculate total volume
    const float TotalVolume = BoxExtent.X * BoxExtent.Y * BoxExtent.Z * 8.0f;
    
    // Calculate average obstacle size
    const float AvgObstacleSize = (MinObstacleSize + MaxObstacleSize) * 0.5f;
    const float AvgObstacleVolume = (4.0f/3.0f) * PI * FMath::Pow(AvgObstacleSize, 3.0f);
    
    // Calculate approximate number of obstacles to place (limited by MaxObstacles)
    const int32 TotalObstacles = FMath::Min(
        FMath::CeilToInt((OcclusionPercentage / 100.0f) * TotalVolume / AvgObstacleVolume),
        MaxObstacles
    );
    
    // 1. Place a sparse set of "seed" points first (much fewer than total obstacles)
    // 2. Then cluster additional obstacles around these seed points
    
    // First, determine how many seed points to place
    const int32 NumSeedPoints = FMath::Clamp(ClusterCount, 1, 20);
    const int32 ObstaclesPerCluster = FMath::CeilToInt(static_cast<float>(TotalObstacles) / NumSeedPoints);
    
    UE_LOG(LogNav3D, Log, TEXT("Clustered Distribution: Seed=%d, SeedPoints=%d, ObstaclesPerCluster=%d, TotalObstacles=%d"), 
        RandomSeed, NumSeedPoints, ObstaclesPerCluster, TotalObstacles);
    
    // Create array to store seed points
    TArray<FVector> SeedPoints;
    SeedPoints.Reserve(NumSeedPoints);
    
    // Place seed points in a sparse uniform distribution
    for (int32 i = 0; i < NumSeedPoints; i++)
    {
        bool bValidPosition = false;
        FVector Position;
        int32 Attempts = 0;
        constexpr int32 MaxAttempts = 50;
        
        // Try to find non-overlapping positions
        while (!bValidPosition && Attempts < MaxAttempts)
        {
            Position = RandomStream.RandPointInBox(FBox(VolumeMin, VolumeMax));
            bValidPosition = true;
            
            // Make sure seed points are well separated
            for (const FVector& ExistingSeed : SeedPoints)
            {
                float MinDistance = ClusterRadius * 2.0f;  // Ensure clusters don't overlap
                if (FVector::Dist(Position, ExistingSeed) < MinDistance)
                {
                    bValidPosition = false;
                    break;
                }
            }
            
            Attempts++;
        }
        
        if (bValidPosition)
        {
            SeedPoints.Add(Position);
            
            // Place a slightly larger obstacle at each seed point
            FVector LocalPosition = WorldToLocalPosition(Position);
            float SeedSize = FMath::Lerp(MaxObstacleSize * 0.8f, MaxObstacleSize, RandomStream.FRand());
            
            // Add instance using local coordinates
            FTransform Transform;
            Transform.SetLocation(LocalPosition);
            Transform.SetScale3D(FVector(SeedSize / 50.0f));  // Assuming sphere mesh has radius 50
            ObstacleMeshes->AddInstance(Transform, /*bWorldSpace=*/false);
        }
    }
    
    // Log how many seed points we actually placed
    UE_LOG(LogNav3D, Log, TEXT("Clustered Distribution: Placed %d seed points"), SeedPoints.Num());
    
    // Now add clustered obstacles around each seed point
    int32 TotalObstaclesPlaced = SeedPoints.Num(); // Count seed points
    
    for (int32 SeedIndex = 0; SeedIndex < SeedPoints.Num(); SeedIndex++)
    {
        // Check if we've hit the MaxObstacles limit
        if (TotalObstaclesPlaced >= MaxObstacles)
        {
            break;
        }
        
        FVector SeedPoint = SeedPoints[SeedIndex];
        int32 ObstaclesToPlace = FMath::Min(ObstaclesPerCluster, MaxObstacles - TotalObstaclesPlaced);
        int32 PlacementAttempts = 0;
        const int32 MaxAttempts = ObstaclesToPlace * 10;
        int32 PlacedForThisSeed = 0;
        
        // Distribute obstacles with decreasing density from center
        while (PlacedForThisSeed < ObstaclesToPlace && PlacementAttempts < MaxAttempts)
        {
            // Generate a random direction
            FVector Direction = RandomStream.VRand().GetSafeNormal();
            
            // Generate distance with bias towards the center
            // Higher pow value = more clustering near center
            float DistanceRatio = FMath::Pow(RandomStream.FRand(), 2.5f);
            float Distance = DistanceRatio * ClusterRadius;
            
            // Calculate world position
            FVector WorldPosition = SeedPoint + Direction * Distance;
            
            // Ensure position is within volume bounds
            if (WorldPosition.X < VolumeMin.X || WorldPosition.X > VolumeMax.X ||
                WorldPosition.Y < VolumeMin.Y || WorldPosition.Y > VolumeMax.Y ||
                WorldPosition.Z < VolumeMin.Z || WorldPosition.Z > VolumeMax.Z)
            {
                PlacementAttempts++;
                continue;
            }
            
            // Size is smaller as you get farther from center (visual interest)
            float SizeFactor = 1.0f - (DistanceRatio * 0.6f); // Leave some randomness
            float Size = FMath::Lerp(MinObstacleSize, MaxObstacleSize * 0.8f, SizeFactor);
            
            // Add some randomness to size
            Size *= 0.8f + (RandomStream.FRand() * 0.4f); // 80-120% of calculated size
            
            // Check if position is already occupied
            if (!IsPointOccluded(WorldPosition, Size))
            {
                // Convert to local position for instancing
                FVector LocalPosition = WorldToLocalPosition(WorldPosition);
                
                // Add instance
                FTransform Transform;
                Transform.SetLocation(LocalPosition);
                Transform.SetScale3D(FVector(Size / 50.0f));  // Assuming sphere mesh has radius 50
                ObstacleMeshes->AddInstance(Transform, /*bWorldSpace=*/false);
                
                PlacedForThisSeed++;
                TotalObstaclesPlaced++;
            }
            
            PlacementAttempts++;
        }
        
        UE_LOG(LogNav3D, Verbose, TEXT("Cluster %d: Placed %d obstacles after %d attempts"), 
            SeedIndex, PlacedForThisSeed, PlacementAttempts);
    }
    
    UE_LOG(LogNav3D, Log, TEXT("Clustered Distribution: Created %d total obstacles (target: %d)"), 
        TotalObstaclesPlaced, TotalObstacles);
    
    // Mark component dirty to update render state
    ObstacleMeshes->MarkRenderStateDirty();
}

TArray<FVector> ANav3DTestVolume::GenerateClusterCenters(const int32 NumClusters, const FRandomStream& RandomStream) const
{
    TArray<FVector> ClusterCenters;
    ClusterCenters.Reserve(NumClusters);
    
    // Get volume dimensions
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const FVector VolumeMin = GetActorLocation() - BoxExtent;
    const FVector VolumeMax = GetActorLocation() + BoxExtent;
    
    // For very small cluster counts, use a different approach to ensure better distribution
    if (NumClusters <= 8)
    {
        // Divide the volume into sectors and place clusters more evenly
        int32 DivisionCount = FMath::CeilToInt(FMath::Pow(NumClusters, 1.0f/3.0f));
        if (DivisionCount < 1) DivisionCount = 1;
        
        // Create a grid of possible positions
        TArray<FVector> GridPositions;
        for (int32 X = 0; X < DivisionCount; X++)
        {
            for (int32 Y = 0; Y < DivisionCount; Y++)
            {
                for (int32 Z = 0; Z < DivisionCount; Z++)
                {
                    FVector Position = FVector(
                        VolumeMin.X + (X + 0.5f) * (2.0f * BoxExtent.X) / DivisionCount,
                        VolumeMin.Y + (Y + 0.5f) * (2.0f * BoxExtent.Y) / DivisionCount,
                        VolumeMin.Z + (Z + 0.5f) * (2.0f * BoxExtent.Z) / DivisionCount
                    );
                    
                    // Add some randomness to the grid positions
                    Position += FVector(
                        RandomStream.FRandRange(-BoxExtent.X / DivisionCount * 0.4f, BoxExtent.X / DivisionCount * 0.4f),
                        RandomStream.FRandRange(-BoxExtent.Y / DivisionCount * 0.4f, BoxExtent.Y / DivisionCount * 0.4f),
                        RandomStream.FRandRange(-BoxExtent.Z / DivisionCount * 0.4f, BoxExtent.Z / DivisionCount * 0.4f)
                    );
                    
                    GridPositions.Add(Position);
                }
            }
        }
        
        // Shuffle the positions
        for (int32 i = 0; i < GridPositions.Num(); i++)
        {
            const int32 SwapIndex = RandomStream.RandRange(0, GridPositions.Num() - 1);
            if (i != SwapIndex)
            {
                GridPositions.Swap(i, SwapIndex);
            }
        }
        
        // Take the first NumClusters positions
        for (int32 i = 0; i < NumClusters && i < GridPositions.Num(); i++)
        {
            ClusterCenters.Add(GridPositions[i]);
        }
    }
    else
    {
        // Original algorithm for larger cluster counts
        // Minimum distance between clusters (prevent overlap)
        const float MinClusterDistance = ClusterRadius * 0.8f;
        
        for (int32 i = 0; i < NumClusters; i++)
        {
            bool bValidPosition = false;
            FVector Position;
            int32 Attempts = 0;
            constexpr int32 MaxAttempts = 50;
            
            // Try to find non-overlapping position
            while (!bValidPosition && Attempts < MaxAttempts)
            {
                Position = RandomStream.RandPointInBox(FBox(VolumeMin, VolumeMax));
                bValidPosition = true;
                
                // Check against existing clusters
                for (const FVector& ExistingCenter : ClusterCenters)
                {
                    if (FVector::Dist(Position, ExistingCenter) < MinClusterDistance)
                    {
                        bValidPosition = false;
                        break;
                    }
                }
                
                Attempts++;
            }
            
            // If we couldn't find a non-overlapping position, just use the last one
            ClusterCenters.Add(Position);
        }
    }
    
    return ClusterCenters;
}

void ANav3DTestVolume::GeneratePerlinNoiseDistribution(const FRandomStream& RandomStream) const
{
    // Get volume dimensions
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const FVector VolumeMin = GetActorLocation() - BoxExtent;
    
    // Perlin noise settings
    const float Threshold = 1.0f - (OcclusionPercentage / 100.0f);
    
    // Grid resolution - adjust based on NoiseScale
    // Higher noise scale = less detail needed = lower resolution
    const float ResolutionMultiplier = FMath::Lerp(1.0f, 0.1f, FMath::Clamp(NoiseScale, 0.01f, 1.0f));
    
    const int32 ResolutionX = FMath::Max(FMath::CeilToInt(BoxExtent.X * 0.1f * ResolutionMultiplier), 8);
    const int32 ResolutionY = FMath::Max(FMath::CeilToInt(BoxExtent.Y * 0.1f * ResolutionMultiplier), 8);
    const int32 ResolutionZ = FMath::Max(FMath::CeilToInt(BoxExtent.Z * 0.1f * ResolutionMultiplier), 8);
    
    // Debug logging
    UE_LOG(LogNav3D, Log, TEXT("Perlin Noise Distribution: Seed=%d, NoiseScale=%.2f, Resolution=%dx%dx%d, Threshold=%.2f"),
        RandomSeed, NoiseScale, ResolutionX, ResolutionY, ResolutionZ, Threshold);
    
    // Random seed offsets for noise - derive from RandomSeed parameter
    const float SeedX = RandomStream.FRand() * 1000.0f;
    const float SeedY = RandomStream.FRand() * 1000.0f;
    const float SeedZ = RandomStream.FRand() * 1000.0f;
    
    // Track how many obstacles were created
    int32 NumObstaclesCreated = 0;
    
    // Sample points in grid and place obstacles based on noise value
    for (int32 X = 0; X < ResolutionX && NumObstaclesCreated < MaxObstacles; X++)
    {
        for (int32 Y = 0; Y < ResolutionY && NumObstaclesCreated < MaxObstacles; Y++)
        {
            for (int32 Z = 0; Z < ResolutionZ && NumObstaclesCreated < MaxObstacles; Z++)
            {
                // Calculate world position
                FVector WorldPosition = FVector(
                    VolumeMin.X + (X + 0.5f) * (2.0f * BoxExtent.X) / ResolutionX,
                    VolumeMin.Y + (Y + 0.5f) * (2.0f * BoxExtent.Y) / ResolutionY,
                    VolumeMin.Z + (Z + 0.5f) * (2.0f * BoxExtent.Z) / ResolutionZ
                );
                
                // Convert to local position for instancing
                FVector LocalPosition = WorldToLocalPosition(WorldPosition);
                
                // Calculate noise value
                // Use NoiseScale to control the frequency of the noise
                // Higher NoiseScale = larger features
                float Noise = FMath::PerlinNoise3D(FVector(
                    SeedX + X * NoiseScale,
                    SeedY + Y * NoiseScale,
                    SeedZ + Z * NoiseScale
                ));
                
                // Remap from [-1,1] to [0,1]
                Noise = (Noise + 1.0f) * 0.5f;
                
                // Place obstacle if noise value exceeds threshold
                if (Noise > Threshold)
                {
                    // Size based on noise value (higher noise = larger obstacle)
                    float SizeFactor = (Noise - Threshold) / (1.0f - Threshold);
                    const float Size = FMath::Lerp(MinObstacleSize, MaxObstacleSize, SizeFactor);
                    
                    // Check if position is already occupied
                    if (!IsPointOccluded(WorldPosition, Size))
                    {
                        // Add instance using local coordinates
                        FTransform Transform;
                        Transform.SetLocation(LocalPosition);
                        Transform.SetScale3D(FVector(Size / 50.0f));  // Assuming sphere mesh has radius 50
                        ObstacleMeshes->AddInstance(Transform, /*bWorldSpace=*/false);
                        NumObstaclesCreated++;
                    }
                }
            }
        }
    }
    
    UE_LOG(LogNav3D, Log, TEXT("Perlin Noise Distribution: Created %d obstacles"), NumObstaclesCreated);
    
    // Mark component dirty to update render state
    ObstacleMeshes->MarkRenderStateDirty();
}

void ANav3DTestVolume::GenerateRingDistribution(const FRandomStream& RandomStream) const
{
    // Get volume center
    const FVector Center = GetActorLocation();
    
    // Calculate total volume for occlusion percentage calculation
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const float TotalVolume = BoxExtent.X * BoxExtent.Y * BoxExtent.Z * 8.0f;
    
    // Calculate average obstacle size
    const float AvgObstacleSize = (MinObstacleSize + MaxObstacleSize) * 0.5f;
    const float AvgObstacleVolume = (4.0f/3.0f) * PI * FMath::Pow(AvgObstacleSize, 3.0f);
    
    // Calculate approximate number of obstacles to place for desired occlusion (limited by MaxObstacles)
    const int32 TotalObstacles = FMath::Min(
        FMath::CeilToInt((OcclusionPercentage / 100.0f) * TotalVolume / AvgObstacleVolume),
        MaxObstacles
    );
    
    // Calculate obstacles per ring (distribute evenly)
    const int32 ObstaclesPerRing = FMath::CeilToInt(static_cast<float>(TotalObstacles) / RingCount);
    
    // Track total placed obstacles
    int32 TotalPlaced = 0;
    
    // For each ring
    for (int32 RingIndex = 0; RingIndex < RingCount && TotalPlaced < MaxObstacles; RingIndex++)
    {
        // Calculate vertical offset based on ring index
        const float VerticalOffset = (RingIndex - (RingCount - 1) / 2.0f) * RingVerticalSpacing;
        FVector RingCenter = Center + FVector(0, 0, VerticalOffset);
        
        // Place obstacles around the ring
        for (int32 i = 0; i < ObstaclesPerRing && TotalPlaced < MaxObstacles; i++)
        {
            // Calculate angle around ring
            float Angle = (i * 360.0f) / ObstaclesPerRing;
            
            // Add some randomness to the radius (within thickness)
            const float RadiusVariation = RandomStream.FRandRange(-RingThickness, RingThickness);
            const float FinalRadius = RingRadius + RadiusVariation;
            
            // Calculate position on ring
            FVector Position = RingCenter + FVector(
                FinalRadius * FMath::Cos(FMath::DegreesToRadians(Angle)),
                FinalRadius * FMath::Sin(FMath::DegreesToRadians(Angle)),
                RandomStream.FRandRange(-RingThickness, RingThickness) * 0.5f // Small Z variation
            );
            
            // Check if position is within volume
            if (FMath::Abs(Position.X - Center.X) > BoxExtent.X ||
                FMath::Abs(Position.Y - Center.Y) > BoxExtent.Y ||
                FMath::Abs(Position.Z - Center.Z) > BoxExtent.Z)
            {
                continue;
            }
            
            // Random size for visual interest
            const float Size = RandomStream.FRandRange(MinObstacleSize, MaxObstacleSize);
            
            // Check if position is already occupied
            if (!IsPointOccluded(Position, Size))
            {
                // Add instance
                FTransform Transform;
                Transform.SetLocation(Position);
                Transform.SetScale3D(FVector(Size / 50.0f));  // Assuming sphere mesh has radius 50
                ObstacleMeshes->AddInstance(Transform, /*bWorldSpace=*/true);
                TotalPlaced++;
            }
        }
    }
    
    // Mark component dirty to update render state
    ObstacleMeshes->MarkRenderStateDirty();
}

void ANav3DTestVolume::GenerateDiscDistribution(const FRandomStream& RandomStream) const
{
    // Get volume center
    const FVector Center = GetActorLocation();
    
    // Calculate total volume for occlusion percentage calculation
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const float TotalVolume = BoxExtent.X * BoxExtent.Y * BoxExtent.Z * 8.0f;
    
    // Calculate average obstacle size
    const float AvgObstacleSize = (MinObstacleSize + MaxObstacleSize) * 0.5f;
    const float AvgObstacleVolume = (4.0f/3.0f) * PI * FMath::Pow(AvgObstacleSize, 3.0f);
    
    // Calculate approximate number of obstacles to place for desired occlusion (limited by MaxObstacles)
    const int32 TotalObstacles = FMath::Min(
        FMath::CeilToInt((OcclusionPercentage / 100.0f) * TotalVolume / AvgObstacleVolume),
        MaxObstacles
    );
    
    // Calculate disc area
    const float DiscArea = PI * DiscRadius * DiscRadius;
    
    // Calculate obstacles per unit area based on density
    const float ObstaclesPerUnitArea = DiscDensity * TotalObstacles / DiscArea;
    
    // Number of rings to create (more rings = more uniform distribution)
    const int32 DiscRingCount = FMath::Max(FMath::CeilToInt(DiscRadius / (AvgObstacleSize * 2.0f)), 5);
    
    // Track total placed obstacles
    int32 TotalPlaced = 0;
    
    // For each ring in the disc
    for (int32 RingIndex = 0; RingIndex < DiscRingCount && TotalPlaced < MaxObstacles; RingIndex++)
    {
        // Calculate ring radius (outer rings need more obstacles)
        const float RingRadiusRatio = static_cast<float>(RingIndex) / DiscRingCount;
        const float CurrentRingRadius = DiscRadius * RingRadiusRatio;
        
        // Calculate ring area
        const float OuterArea = PI * FMath::Pow(DiscRadius * (RingIndex + 1.0f) / DiscRingCount, 2.0f);
        const float InnerArea = PI * FMath::Pow(CurrentRingRadius, 2.0f);
        const float RingArea = OuterArea - InnerArea;
        
        // Calculate number of obstacles in this ring
        const int32 ObstaclesInRing = FMath::Max(FMath::CeilToInt(RingArea * ObstaclesPerUnitArea), 1);
        
        // Calculate spacing between obstacles around the ring
        const float ObstacleSpacing = 360.0f / ObstaclesInRing;
        
        // Place obstacles around the ring
        for (int32 i = 0; i < ObstaclesInRing && TotalPlaced < MaxObstacles; i++)
        {
            // Calculate angle around ring
            float Angle = i * ObstacleSpacing;
            
            // Calculate radius with some variation
            const float RingWidthRatio = 1.0f / DiscRingCount;
            const float RandomRadius = CurrentRingRadius + RandomStream.FRandRange(0.0f, DiscRadius * RingWidthRatio);
            
            // Calculate position based on orientation
            FVector Position = FVector();
            switch (DiscOrientation)
            {
                case ENav3DDiscOrientation::XY: // Horizontal (floor/ceiling)
                    Position = Center + FVector(
                        RandomRadius * FMath::Cos(FMath::DegreesToRadians(Angle)),
                        RandomRadius * FMath::Sin(FMath::DegreesToRadians(Angle)),
                        0.0f
                    );
                    break;
                    
                case ENav3DDiscOrientation::XZ: // Vertical (front/back wall)
                    Position = Center + FVector(
                        RandomRadius * FMath::Cos(FMath::DegreesToRadians(Angle)),
                        0.0f,
                        RandomRadius * FMath::Sin(FMath::DegreesToRadians(Angle))
                    );
                    break;
                    
                case ENav3DDiscOrientation::YZ: // Vertical (side wall)
                    Position = Center + FVector(
                        0.0f,
                        RandomRadius * FMath::Cos(FMath::DegreesToRadians(Angle)),
                        RandomRadius * FMath::Sin(FMath::DegreesToRadians(Angle))
                    );
                    break;
            }
            
            // Check if position is within volume
            if (FMath::Abs(Position.X - Center.X) > BoxExtent.X ||
                FMath::Abs(Position.Y - Center.Y) > BoxExtent.Y ||
                FMath::Abs(Position.Z - Center.Z) > BoxExtent.Z)
            {
                continue;
            }
            
            // Random size for visual interest
            const float Size = RandomStream.FRandRange(MinObstacleSize, MaxObstacleSize);
            
            // Check if position is already occupied
            if (!IsPointOccluded(Position, Size))
            {
                // Add instance
                FTransform Transform;
                Transform.SetLocation(Position);
                Transform.SetScale3D(FVector(Size / 50.0f));  // Assuming sphere mesh has radius 50
                ObstacleMeshes->AddInstance(Transform, /*bWorldSpace=*/true);
                TotalPlaced++;
            }
        }
    }
    
    // Mark component dirty to update render state
    ObstacleMeshes->MarkRenderStateDirty();
}

void ANav3DTestVolume::GenerateSplineDistribution(const FRandomStream& RandomStream) const
{
    // Make sure the spline has points
    if (ObstacleSpline->GetNumberOfSplinePoints() < 2)
    {
        UE_LOG(LogNav3D, Warning, TEXT("Spline has insufficient points. Generating random spline."));
        GenerateRandomSpline(RandomStream);
    }
    
    // Calculate total volume for occlusion percentage calculation
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const float TotalVolume = BoxExtent.X * BoxExtent.Y * BoxExtent.Z * 8.0f;
    
    // Calculate average obstacle size
    const float AvgObstacleSize = (MinObstacleSize + MaxObstacleSize) * 0.5f;
    const float AvgObstacleVolume = (4.0f/3.0f) * PI * FMath::Pow(AvgObstacleSize, 3.0f);
    
    // Calculate approximate number of obstacles to place for desired occlusion (limited by MaxObstacles)
    const int32 TotalObstacles = FMath::Min(
        FMath::CeilToInt((OcclusionPercentage / 100.0f) * TotalVolume / AvgObstacleVolume),
        MaxObstacles
    );
    
    // Calculate spline length
    const float ActualSplineLength = ObstacleSpline->GetSplineLength();
    
    // Calculate spacing between obstacles along the spline
    const float ObstacleSpacing = ActualSplineLength / TotalObstacles;
    
    // Debug log
    UE_LOG(LogNav3D, Log, TEXT("Spline Distribution: Seed=%d, Length=%.1f, Obstacles=%d, Spacing=%.1f"), 
        RandomSeed, ActualSplineLength, TotalObstacles, ObstacleSpacing);
    
    // Tracking for placed obstacles
    int32 PlacedObstacles = 0;
    
    // Place obstacles along the spline
    for (float Distance = 0.0f; Distance < ActualSplineLength && PlacedObstacles < MaxObstacles; Distance += ObstacleSpacing)
    {
        // Get position and direction at this point on the spline (in world space)
        FVector WorldPosition = ObstacleSpline->GetLocationAtDistanceAlongSpline(Distance, ESplineCoordinateSpace::World);
        FVector Direction = ObstacleSpline->GetDirectionAtDistanceAlongSpline(Distance, ESplineCoordinateSpace::World);
        
        // Add random offset perpendicular to spline
        FVector Perpendicular = RandomStream.VRand().GetSafeNormal();
        Perpendicular = (Perpendicular - (Perpendicular | Direction) * Direction).GetSafeNormal();
        const float RadialOffset = RandomStream.FRandRange(0.0f, SplineRadius);
        WorldPosition += Perpendicular * RadialOffset;
        
        // Convert to local space for instancing
        FVector LocalPosition = WorldToLocalPosition(WorldPosition);
        
        // Check if position is within volume bounds
        const FVector VolumeMin = GetActorLocation() - BoxExtent;
        const FVector VolumeMax = GetActorLocation() + BoxExtent;
        
        if (WorldPosition.X < VolumeMin.X || WorldPosition.X > VolumeMax.X ||
            WorldPosition.Y < VolumeMin.Y || WorldPosition.Y > VolumeMax.Y ||
            WorldPosition.Z < VolumeMin.Z || WorldPosition.Z > VolumeMax.Z)
        {
            continue;
        }
        
        // Random size for obstacles
        const float Size = RandomStream.FRandRange(MinObstacleSize, MaxObstacleSize);
        
        // Check if position is already occupied
        if (!IsPointOccluded(WorldPosition, Size))
        {
            // Add instance using local coordinates
            FTransform Transform;
            Transform.SetLocation(LocalPosition);
            Transform.SetScale3D(FVector(Size / 50.0f));  // Assuming sphere mesh has radius 50
            ObstacleMeshes->AddInstance(Transform, /*bWorldSpace=*/false);
            PlacedObstacles++;
        }
    }
    
    UE_LOG(LogNav3D, Log, TEXT("Spline Distribution: Placed %d obstacles"), PlacedObstacles);
    
    // Mark component dirty to update render state
    ObstacleMeshes->MarkRenderStateDirty();
}

bool ANav3DTestVolume::IsPointOccluded(const FVector& WorldPoint, const float Radius) const
{
    // Check for overlap with existing obstacles
    const int32 InstanceCount = ObstacleMeshes->GetInstanceCount();
    const FTransform ActorTransform = GetActorTransform();
    
    for (int32 i = 0; i < InstanceCount; i++)
    {
        FTransform InstanceTransform;
        ObstacleMeshes->GetInstanceTransform(i, InstanceTransform);
        
        // Convert instance local position to world space for comparison
        FVector InstanceWorldLocation = ActorTransform.TransformPosition(InstanceTransform.GetLocation());
        FVector InstanceScale = InstanceTransform.GetScale3D();
        
        // Calculate instance radius (assuming spherical mesh with radius 50)
        const float InstanceRadius = InstanceScale.GetMin() * 50.0f;
        
        // Minimum separation distance to prevent overlap
        const float MinSeparation = Radius + InstanceRadius;
        
        // Check distance
        if (FVector::Dist(WorldPoint, InstanceWorldLocation) < MinSeparation)
        {
            return true;
        }
    }
    
    return false;
}

float ANav3DTestVolume::CalculateActualOcclusionPercentage() const
{
    // Get total volume
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const float TotalVolume = BoxExtent.X * BoxExtent.Y * BoxExtent.Z * 8.0f;
    
    // Calculate occupied volume
    float OccupiedVolume = 0.0f;
    const int32 InstanceCount = ObstacleMeshes->GetInstanceCount();
    
    for (int32 i = 0; i < InstanceCount; i++)
    {
        FTransform InstanceTransform;
        ObstacleMeshes->GetInstanceTransform(i, InstanceTransform);
        
        FVector InstanceScale = InstanceTransform.GetScale3D();
        const float Radius = InstanceScale.GetMin() * 50.0f;
        
        // Sphere volume
        OccupiedVolume += (4.0f/3.0f) * PI * FMath::Pow(Radius, 3.0f);
    }
    
    // Calculate percentage
    return (OccupiedVolume / TotalVolume) * 100.0f;
}

void ANav3DTestVolume::ClampPointToVolumeBox(FVector& Point) const
{
    const FVector BoxExtent = VolumeBox->GetScaledBoxExtent();
    const FVector VolumeMin = GetActorLocation() - BoxExtent;
    const FVector VolumeMax = GetActorLocation() + BoxExtent;
    
    Point.X = FMath::Clamp(Point.X, VolumeMin.X, VolumeMax.X);
    Point.Y = FMath::Clamp(Point.Y, VolumeMin.Y, VolumeMax.Y);
    Point.Z = FMath::Clamp(Point.Z, VolumeMin.Z, VolumeMax.Z);
}

FVector ANav3DTestVolume::WorldToLocalPosition(const FVector& WorldPosition) const
{
    return GetActorTransform().InverseTransformPosition(WorldPosition);
}

void ANav3DTestVolume::TestTacticalDataGeneration() const
{
    UE_LOG(LogNav3D, Display, TEXT("=== Testing Tactical Data Generation ==="));
    
    // Find the Nav3DData in the world
    const ANav3DData* NavData = nullptr;
    if (const UWorld* World = GetWorld())
    {
        for (const TActorIterator<ANav3DData> ActorItr(World); ActorItr;)
        {
            NavData = *ActorItr;
            break;
        }
    }
    
    if (!NavData)
    {
        UE_LOG(LogNav3D, Error, TEXT("No Nav3DData found in world for tactical testing"));
        return;
    }
    
    // Check if tactical reasoning is enabled
    if (!NavData->TacticalSettings.bEnableTacticalReasoning)
    {
        UE_LOG(LogNav3D, Warning, TEXT("Tactical reasoning is disabled. Enable it in Nav3DData settings."));
        return;
    }
    
    // Get all chunk actors
    const TArray<ANav3DDataChunkActor*>& ChunkActors = NavData->GetAllChunkActors();
    UE_LOG(LogNav3D, Display, TEXT("Found %d chunk actors"), ChunkActors.Num());
    
    int32 TotalRegions = 0;
    int32 ChunksWithTacticalData = 0;
    
    for (const ANav3DDataChunkActor* ChunkActor : ChunkActors)
    {
        if (ChunkActor && ChunkActor->HasTacticalData())
        {
            ChunksWithTacticalData++;
            const int32 RegionCount = ChunkActor->GetTacticalRegionCount();
            TotalRegions += RegionCount;
            
            UE_LOG(LogNav3D, Display, TEXT("Chunk %s: %d regions, %d boundary interfaces"), 
                   *ChunkActor->GetName(), RegionCount, ChunkActor->ConnectionInterfaces.Num());
        }
    }
    
    UE_LOG(LogNav3D, Display, TEXT("Tactical Data Summary:"));
    UE_LOG(LogNav3D, Display, TEXT("  - Chunks with tactical data: %d/%d"), ChunksWithTacticalData, ChunkActors.Num());
    UE_LOG(LogNav3D, Display, TEXT("  - Total regions: %d"), TotalRegions);
    UE_LOG(LogNav3D, Display, TEXT("  - Consolidated regions: %d"), NavData->ConsolidatedTacticalData.GetRegionCount());
    
    if (TotalRegions > 0)
    {
        UE_LOG(LogNav3D, Display, TEXT("✅ Tactical data generation test PASSED"));
    }
    else
    {
        UE_LOG(LogNav3D, Error, TEXT("❌ Tactical data generation test FAILED - No regions found"));
    }
}

void ANav3DTestVolume::TestTacticalQueries() const
{
    UE_LOG(LogNav3D, Display, TEXT("=== Testing Tactical Queries ==="));
    
    // Find the Nav3DData in the world
    const ANav3DData* NavData = nullptr;
    if (const UWorld* World = GetWorld())
    {
        for (const TActorIterator<ANav3DData> ActorItr(World); ActorItr;)
        {
            NavData = *ActorItr;
            break;
        }
    }
    
    if (!NavData)
    {
        UE_LOG(LogNav3D, Error, TEXT("No Nav3DData found in world for tactical testing"));
        return;
    }
    
    if (NavData->ConsolidatedTacticalData.IsEmpty())
    {
        UE_LOG(LogNav3D, Warning, TEXT("No consolidated tactical data available for testing"));
        return;
    }
    
    // Test basic tactical queries
    const FVector TestPosition = GetActorLocation();
    const TArray<FVector> ObserverPositions = {TestPosition + FVector(100, 0, 0)};
    TArray<FPositionCandidate> Candidates;

    const bool bQuerySuccess = NavData->FindBestLocation(
        TestPosition,
        ObserverPositions,
        Candidates,
        ETacticalVisibility::TargetVisible,
        ETacticalDistance::Closest,
        ETacticalRegion::Largest,
        true, // Force new region
        false // Don't use raycasting for this test
    );
    
    if (bQuerySuccess && Candidates.Num() > 0)
    {
        UE_LOG(LogNav3D, Display, TEXT("✅ Tactical query test PASSED - Found %d candidate positions"), Candidates.Num());
        
        // Log the best candidate
        const FPositionCandidate& BestCandidate = Candidates[0];
        UE_LOG(LogNav3D, Display, TEXT("  Best position: %s (Score: %.3f, Distance: %.1f)"), 
               *BestCandidate.Position.ToString(), BestCandidate.Score, BestCandidate.DirectDistance);
    }
    else
    {
        UE_LOG(LogNav3D, Error, TEXT("❌ Tactical query test FAILED - No candidates found"));
    }
}

int32 ANav3DTestVolume::GetTacticalRegionCount() const
{
    // Find the Nav3DData in the world
    const ANav3DData* NavData = nullptr;
    if (const UWorld* World = GetWorld())
    {
        for (const TActorIterator<ANav3DData> ActorItr(World); ActorItr;)
        {
            NavData = *ActorItr;
            break;
        }
    }
    
    if (!NavData)
    {
        return 0;
    }
    
    return NavData->ConsolidatedTacticalData.GetRegionCount();
}