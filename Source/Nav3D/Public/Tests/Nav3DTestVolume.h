// ReSharper disable CppUEBlueprintCallableFunctionUnused
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "Components/SplineComponent.h"
#include "Nav3DTestVolume.generated.h"

UENUM(BlueprintType)
enum class ENav3DTestDistribution : uint8
{
    Uniform      UMETA(DisplayName = "Uniform"),
    Clustered    UMETA(DisplayName = "Clustered"),
    PerlinNoise  UMETA(DisplayName = "Perlin Noise"),
    Ring         UMETA(DisplayName = "Ring"),
    Disc         UMETA(DisplayName = "Disc"),
    Spline       UMETA(DisplayName = "Spline Path")
};

UENUM(BlueprintType)
enum class ENav3DDiscOrientation : uint8
{
    XY          UMETA(DisplayName = "XY Plane (Horizontal)"),
    XZ          UMETA(DisplayName = "XZ Plane"),
    YZ          UMETA(DisplayName = "YZ Plane (Vertical)")
};

/**
 * Actor for testing Nav3D by procedurally generating obstacles within a volume
 * 
 * The RandomSeed property can be used to generate the same pattern of obstacles
 * consistently across multiple uses. Changing the seed will result in a different
 * random distribution, but using the same seed will always produce identical results.
 */
UCLASS(Blueprintable, meta=(DisplayName="Nav3D Test Volume", PrioritizeCategories="Nav3D"))
class NAV3D_API ANav3DTestVolume : public AActor
{
    GENERATED_BODY()
    
public:    
    ANav3DTestVolume();

protected:
    virtual void BeginPlay() override;
    virtual void OnConstruction(const FTransform& Transform) override;
    
#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
    
    // Components
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Nav3D")
    UBoxComponent* VolumeBox;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Nav3D")
    UInstancedStaticMeshComponent* ObstacleMeshes;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Nav3D")
    USplineComponent* ObstacleSpline;

	// Properties
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", meta=(ClampMin="1.0", ClampMax="50.0", UIMin="1.0", UIMax="50.0"))
	float OcclusionPercentage;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D")
	ENav3DTestDistribution DistributionType;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D")
	int32 RandomSeed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D")
	UStaticMesh* ObstacleMesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D")
	float MinObstacleSize;
    
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D")
	float MaxObstacleSize;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", meta=(ClampMin="1", ClampMax="5000", UIMin="1", UIMax="5000"))
	int32 MaxObstacles;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D")
	uint8 bAutoGenerate : 1;

    // Clustered distribution settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Clustered", EditConditionHides))
    int32 ClusterCount;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Clustered", EditConditionHides))
    float ClusterRadius;

	// Perlin distribution settings 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
		  meta=(EditCondition="DistributionType==ENav3DTestDistribution::PerlinNoise", EditConditionHides, ClampMin="0.01", ClampMax="1.0", UIMin="0.01", UIMax="1.0"))
	float NoiseScale;
	
    // Ring distribution settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Ring", EditConditionHides))
    float RingRadius;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Ring", EditConditionHides))
    float RingThickness;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Ring", EditConditionHides))
    int32 RingCount;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Ring", EditConditionHides))
    float RingVerticalSpacing;
    
    // Disc distribution settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Disc", EditConditionHides))
    float DiscRadius;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Disc", EditConditionHides))
    ENav3DDiscOrientation DiscOrientation;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Disc", EditConditionHides))
    float DiscDensity;
    
    // Spline distribution settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Spline", EditConditionHides))
    float SplineRadius;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
		  meta=(EditCondition="DistributionType==ENav3DTestDistribution::Spline", EditConditionHides, ClampMin="100.0", UIMin="100.0"))
	float SplineLength;
	
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Spline", EditConditionHides))
    int32 SplinePointCount;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Spline", EditConditionHides))
    float SplinePointSpacing;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Nav3D", 
              meta=(EditCondition="DistributionType==ENav3DTestDistribution::Spline", EditConditionHides))
    uint8 bRandomizeSpline : 1;

private:
	// Helper function to convert world position to local space
	FVector WorldToLocalPosition(const FVector& WorldPosition) const;

public:
    // Public functions
    UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D")
    void GenerateObstacles() const;
    
    UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D")
    void ClearObstacles() const;
    
    UFUNCTION(BlueprintCallable, Category="Nav3D")
    int32 GetObstacleCount() const;
    
	UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D")
	void GenerateRandomSpline() const;
	
    void GenerateRandomSpline(const FRandomStream& RandomStream) const;
    
    // Tactical testing functions
    UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D Tactical")
    void TestTacticalDataGeneration() const;
    
    UFUNCTION(BlueprintCallable, CallInEditor, Category="Nav3D Tactical")
    void TestTacticalQueries() const;
    
    UFUNCTION(BlueprintCallable, Category="Nav3D Tactical")
    int32 GetTacticalRegionCount() const;
    
private:
    // Helper functions
	void GenerateUniformDistribution(const FRandomStream& RandomStream) const;
	void GenerateClusteredDistribution(const FRandomStream& RandomStream) const;
	void GeneratePerlinNoiseDistribution(const FRandomStream& RandomStream) const;
	void GenerateRingDistribution(const FRandomStream& RandomStream) const;
	void GenerateDiscDistribution(const FRandomStream& RandomStream) const;
	void GenerateSplineDistribution(const FRandomStream& RandomStream) const;
    TArray<FVector> GenerateClusterCenters(int32 NumClusters, const FRandomStream& RandomStream) const;
    bool IsPointOccluded(const FVector& Point, float Radius) const;
    float CalculateActualOcclusionPercentage() const;
    void ClampPointToVolumeBox(FVector& Point) const;
};