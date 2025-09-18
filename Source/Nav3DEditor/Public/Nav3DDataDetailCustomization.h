#pragma once

#include "CoreMinimal.h"
#include "DetailCategoryBuilder.h"
#include "IDetailCustomization.h"

class ANav3DData;

/**
 * Custom details panel for Nav3DData actor
 */
class FNav3DDataDetailCustomization : public IDetailCustomization
{
public:
    /** Makes a new instance of this detail customization */
    static TSharedRef<IDetailCustomization> MakeInstance();

    /** IDetailCustomization interface */
    virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;

private:
    /** Cached pointer to the Nav3DData being customized */
    TWeakObjectPtr<ANav3DData> Nav3DDataPtr;

    /** Generate the voxel info display panel */
    void GenerateVoxelInfoPanel(IDetailLayoutBuilder& DetailBuilder, IDetailCategoryBuilder& CategoryBuilder) const;
    
    /** Show consolidated tactical status */
    void AddConsolidatedTacticalStatusPanel(IDetailLayoutBuilder& DetailBuilder, IDetailCategoryBuilder& CategoryBuilder) const;
}; 