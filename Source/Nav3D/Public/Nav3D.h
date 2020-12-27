#pragma once

#if WITH_EDITOR
DECLARE_LOG_CATEGORY_EXTERN(LogNav3D, Log, All);
#endif

class FNav3DModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};