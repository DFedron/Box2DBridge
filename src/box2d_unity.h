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

enum B2U_BodyType
{
    B2U_Static = 0,
    B2U_Kinematic = 1,
    B2U_Dynamic = 2
};

struct B2U_BodyDef
{
    int32_t  type;
    B2U_Vec2 position;
    float    angle;
    B2U_Vec2 linearVelocity;
    float    angularVelocity;
    float    gravityScale;
    float    linearDamping;
    float    angularDamping;
    uint8_t  allowSleep;
    uint8_t  fixedRotation;
    uint8_t  isBullet;
};

struct B2U_BoxBodyDef
{
    B2U_BodyDef body;
    float       hx;
    float       hy;
    float       density;
    float       friction;
    float       restitution;
    int32_t     layer;
};

struct B2U_CircleBodyDef
{
    B2U_BodyDef body;
    float       cx;
    float       cy;
    float       radius;
    float       density;
    float       friction;
    float       restitution;
    int32_t     layer;
};

B2U_API B2U_World b2uCreateWorld(float gx, float gy);
B2U_API void      b2uDestroyWorld(B2U_World world);
B2U_API void      b2uStep(B2U_World world, float dt);
B2U_API void      b2uStepEx(B2U_World world, float dt, int32_t subSteps);
B2U_API void      b2uSetWorldGravity(B2U_World world, float gx, float gy);
B2U_API void      b2uGetWorldGravity(B2U_World world, float* gx, float* gy);

B2U_API void      b2uResetLayerFilters(void);
B2U_API void      b2uSetLayerFilter(int32_t layer, uint64_t categoryBits, uint64_t maskBits, int32_t groupIndex);

B2U_API B2U_Body  b2uCreateBody(B2U_World world, const B2U_BodyDef* def);
B2U_API int32_t   b2uCreateBodies(B2U_World world, const B2U_BodyDef* defs, int32_t count, B2U_Body* outBodies);
B2U_API int32_t   b2uCreateBoxBodies(B2U_World world, const B2U_BoxBodyDef* defs, int32_t count, B2U_Body* outBodies);
B2U_API int32_t   b2uCreateCircleBodies(B2U_World world, const B2U_CircleBodyDef* defs, int32_t count, B2U_Body* outBodies);

B2U_API void      b2uCreateBoxShape(B2U_Body body, float hx, float hy, float density, float friction, float restitution, int32_t layer);
B2U_API void      b2uCreateCircleShape(B2U_Body body, float cx, float cy, float radius, float density, float friction, float restitution, int32_t layer);
B2U_API void      b2uCreatePolygonShape(B2U_Body body, const B2U_Vec2* vertices, int32_t count, float radius, float density, float friction, float restitution, int32_t layer);
B2U_API void      b2uCreateEdgeShape(B2U_Body body, float ax, float ay, float bx, float by, float density, float friction, float restitution, int32_t layer);

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

B2U_API void b2uGetBodyTransformsBatch(
    const B2U_Body* bodies,
    int32_t count,
    float* outPx,
    float* outPy,
    float* outAngle
);

B2U_API void b2uDestroyBody(B2U_Body body);
B2U_API void b2uDestroyBodies(B2U_Body* bodies, int32_t count);
