#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

class IDetailLayoutBuilder;
class ANav3DModifierVolume;

class FNav3DModifierVolumeProperties : public IDetailCustomization
{
public:
    static TSharedRef<IDetailCustomization> MakeInstance();
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;

private:
    TWeakObjectPtr<ANav3DModifierVolume> Volume;
};
