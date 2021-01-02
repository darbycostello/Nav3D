#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

class IDetailLayoutBuilder;
class ANav3DVolume;

class FNav3DVolumeProperties final : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	FReply OnBuildOctree() const;
	FReply OnClearOctree() const;

private:
	TWeakObjectPtr<ANav3DVolume> Volume;
};
