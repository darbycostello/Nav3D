#pragma once
// https://github.com/darbycostello/Nav3D?tab=readme-ov-file
// https://www.gdcvault.com/play/1022016/Getting-off-the-NavMesh-Navigating
#if WITH_EDITOR
DECLARE_LOG_CATEGORY_EXTERN(LogNav3D, Log, All);
#endif

class FNav3DModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};