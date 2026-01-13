#pragma once
#include <stdint.h>

#ifdef _WIN32
  #ifdef B2U_EXPORTS
    #define B2U_API extern "C" __declspec(dllexport)
  #else
    #define B2U_API extern "C" __declspec(dllimport)
  #endif
#else
  #define B2U_API extern "C"
#endif

typedef void* B2U_World;
typedef void* B2U_Body;

struct B2U_Vec2
{
    float x;
    float y;
};

B2U_API B2U_World b2uCreateWorld(float gx, float gy);
B2U_API void      b2uDestroyWorld(B2U_World world);
B2U_API void      b2uStep(B2U_World world, float dt);

B2U_API B2U_Body  b2uCreateDynamicBox(
    B2U_World world,
    float px, float py,
    float hx, float hy,
    float density
);

B2U_API void b2uGetBodyTransform(
    B2U_Body body,
    float* px,
    float* py,
    float* angle
);

B2U_API void b2uDestroyBody(B2U_Body body);
