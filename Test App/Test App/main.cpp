#pragma region Includes
#include <platform/d3d11/system/platform_d3d11.h>
#include "flocking_app.h"
#pragma endregion

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR pScmdline, int iCmdshow)
{
	// This sets out the initial parameters of the window.
	gef::PlatformD3D11 platform(hInstance, 960, 544, false, true);

	flocking_app app(platform);
	app.Run();

	return 0;
}