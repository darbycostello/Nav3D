#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

class IDetailLayoutBuilder;
class ANav3DVolume;

class FNav3DVolumeProperties : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	FReply OnBuildOctree();
	FReply OnClearOctree();

private:
	TWeakObjectPtr<ANav3DVolume> Volume;
};
