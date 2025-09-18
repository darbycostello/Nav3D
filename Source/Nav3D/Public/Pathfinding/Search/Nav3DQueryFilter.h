#pragma once
#include <NavFilters/NavigationQueryFilter.h>
#include <AI/Navigation/NavQueryFilter.h>
#include "Pathfinding/Core/Nav3DPathingTypes.h"
#include "Nav3DQueryFilter.generated.h"

UCLASS()
class NAV3D_API UNav3DQueryFilter final : public UNavigationQueryFilter
{
	GENERATED_BODY()

public:
	void SetQueryFilterSettings(const FNav3DQueryFilterSettings& Settings) { QueryFilterSettings = Settings; }

private:
	UPROPERTY(EditDefaultsOnly)
	FNav3DQueryFilterSettings QueryFilterSettings;
};

class NAV3D_API FNav3DQueryFilter final : public INavigationQueryFilterInterface
{
public:
	virtual void Reset() override;
	virtual void SetAreaCost(uint8 AreaType, float Cost) override;
	virtual void SetFixedAreaEnteringCost(uint8 AreaType, float Cost) override;
	virtual void SetExcludedArea(uint8 AreaType) override;
	virtual void SetAllAreaCosts(const float* CostArray, const int32 Count) override;
	virtual void GetAllAreaCosts(float* CostArray, float* FixedCostArray, const int32 Count) const override;
	virtual void SetBacktrackingEnabled(const bool bBacktracking) override;
	virtual bool IsBacktrackingEnabled() const override;
	virtual float GetHeuristicScale() const override;
	virtual bool IsEqual(const INavigationQueryFilterInterface* Other) const override;
	virtual void SetIncludeFlags(uint16 Flags) override;
	virtual uint16 GetIncludeFlags() const override;
	virtual void SetExcludeFlags(uint16 Flags) override;
	virtual uint16 GetExcludeFlags() const override;
	virtual INavigationQueryFilterInterface* CreateCopy() const override;
	FNav3DQueryFilterSettings QueryFilterSettings;
};
