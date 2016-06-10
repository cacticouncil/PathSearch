// Jeremiah Blanchard, 2016
#pragma once

// Used to export DLL functions
#ifdef _MSC_VER
#define DLLEXPORT __declspec(dllexport)
#define _CRT_SECURE_NO_WARNINGS
#else
#define DLLEXPORT
#endif

// Special define for Winelib
#if defined(USE_WINE_BUILD)
#define NOMINMAX
#endif
