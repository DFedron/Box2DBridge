#include "box2d_unity.h"
#include "box2d/box2d.h"
#include <cmath>    // for std::atan2
#include <cstdint>

struct WorldWrap
{
    b2WorldId worldId;
};

struct BodyWrap
{
    b2BodyId bodyId;
};

static const int32_t kB2uLayerCount = 64;
static b2Filter g_layerFilters[kB2uLayerCount];
static bool g_layerFilterInit[kB2uLayerCount];

static b2BodyType b2uToBodyType(int32_t type)
{
    switch (type)
    {
    case B2U_Static: return b2_staticBody;
    case B2U_Kinematic: return b2_kinematicBody;
    case B2U_Dynamic: return b2_dynamicBody;
    default: return b2_dynamicBody;
    }
}

static b2Filter b2uGetLayerFilter(int32_t layer)
{
    if (layer < 0 || layer >= kB2uLayerCount)
    {
        return b2DefaultFilter();
    }

    if (!g_layerFilterInit[layer])
    {
        b2Filter filter = b2DefaultFilter();
        filter.categoryBits = (uint64_t)1 << layer;
        filter.maskBits = UINT64_MAX;
        filter.groupIndex = 0;
        g_layerFilters[layer] = filter;
        g_layerFilterInit[layer] = true;
    }

    return g_layerFilters[layer];
}

static void b2uApplyShapeParams(b2ShapeDef* sd, float density, float friction, float restitution, int32_t layer)
{
    sd->density = density;
    sd->filter = b2uGetLayerFilter(layer);
    sd->material = b2DefaultSurfaceMaterial();
    sd->material.friction = friction;
    sd->material.restitution = restitution;
}

static void b2uApplyBodyMassIfNeeded(b2BodyId bodyId, const b2ShapeDef* sd)
{
    if (sd->updateBodyMass)
    {
        b2Body_ApplyMassFromShapes(bodyId);
    }
}

B2U_World b2uCreateWorld(float gx, float gy)
{
    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = { gx, gy };

    WorldWrap* w = new WorldWrap();
    w->worldId = b2CreateWorld(&wd);
    return (B2U_World)w;
}

void b2uDestroyWorld(B2U_World world)
{
    if (!world) return;
    WorldWrap* w = (WorldWrap*)world;
    b2DestroyWorld(w->worldId);
    delete w;
}

void b2uStep(B2U_World world, float dt)
{
    b2uStepEx(world, dt, 8);
}

void b2uStepEx(B2U_World world, float dt, int32_t subSteps)
{
    if (!world) return;
    WorldWrap* w = (WorldWrap*)world;

    if (subSteps < 1) subSteps = 1;
    b2World_Step(w->worldId, dt, subSteps);
}

void b2uSetWorldGravity(B2U_World world, float gx, float gy)
{
    if (!world) return;
    WorldWrap* w = (WorldWrap*)world;
    b2World_SetGravity(w->worldId, { gx, gy });
}

void b2uGetWorldGravity(B2U_World world, float* gx, float* gy)
{
    if (!world) return;
    WorldWrap* w = (WorldWrap*)world;
    b2Vec2 g = b2World_GetGravity(w->worldId);
    if (gx) *gx = g.x;
    if (gy) *gy = g.y;
}

void b2uResetLayerFilters(void)
{
    for (int32_t i = 0; i < kB2uLayerCount; ++i)
    {
        g_layerFilterInit[i] = false;
    }
}

void b2uSetLayerFilter(int32_t layer, uint64_t categoryBits, uint64_t maskBits, int32_t groupIndex)
{
    if (layer < 0 || layer >= kB2uLayerCount)
    {
        return;
    }

    b2Filter filter = b2DefaultFilter();
    filter.categoryBits = categoryBits;
    filter.maskBits = maskBits;
    filter.groupIndex = groupIndex;
    g_layerFilters[layer] = filter;
    g_layerFilterInit[layer] = true;
}

B2U_Body b2uCreateBody(B2U_World world, const B2U_BodyDef* def)
{
    if (!world || !def) return nullptr;
    WorldWrap* w = (WorldWrap*)world;

    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2uToBodyType(def->type);
    bd.position = { def->position.x, def->position.y };
    bd.rotation = b2MakeRot(def->angle);
    bd.linearVelocity = { def->linearVelocity.x, def->linearVelocity.y };
    bd.angularVelocity = def->angularVelocity;
    bd.gravityScale = def->gravityScale;
    bd.linearDamping = def->linearDamping;
    bd.angularDamping = def->angularDamping;
    bd.enableSleep = def->allowSleep != 0;
    bd.isBullet = def->isBullet != 0;
    bd.motionLocks.angularZ = def->fixedRotation != 0;

    b2BodyId bodyId = b2CreateBody(w->worldId, &bd);
    BodyWrap* bw = new BodyWrap();
    bw->bodyId = bodyId;
    return (B2U_Body)bw;
}

int32_t b2uCreateBodies(B2U_World world, const B2U_BodyDef* defs, int32_t count, B2U_Body* outBodies)
{
    if (!world || !defs || !outBodies || count <= 0) return 0;

    int32_t created = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        B2U_Body body = b2uCreateBody(world, &defs[i]);
        outBodies[i] = body;
        if (body) ++created;
    }
    return created;
}

int32_t b2uCreateBoxBodies(B2U_World world, const B2U_BoxBodyDef* defs, int32_t count, B2U_Body* outBodies)
{
    if (!world || !defs || !outBodies || count <= 0) return 0;

    int32_t created = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        B2U_Body body = b2uCreateBody(world, &defs[i].body);
        outBodies[i] = body;
        if (!body) continue;
        b2uCreateBoxShape(body, defs[i].hx, defs[i].hy, defs[i].density, defs[i].friction, defs[i].restitution, defs[i].layer);
        ++created;
    }
    return created;
}

int32_t b2uCreateCircleBodies(B2U_World world, const B2U_CircleBodyDef* defs, int32_t count, B2U_Body* outBodies)
{
    if (!world || !defs || !outBodies || count <= 0) return 0;

    int32_t created = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        B2U_Body body = b2uCreateBody(world, &defs[i].body);
        outBodies[i] = body;
        if (!body) continue;
        b2uCreateCircleShape(body, defs[i].cx, defs[i].cy, defs[i].radius, defs[i].density, defs[i].friction, defs[i].restitution, defs[i].layer);
        ++created;
    }
    return created;
}

void b2uCreateBoxShape(B2U_Body body, float hx, float hy, float density, float friction, float restitution, int32_t layer)
{
    if (!body) return;
    BodyWrap* bw = (BodyWrap*)body;

    b2ShapeDef sd = b2DefaultShapeDef();
    b2uApplyShapeParams(&sd, density, friction, restitution, layer);

    b2Polygon poly = b2MakeBox(hx, hy);
    b2CreatePolygonShape(bw->bodyId, &sd, &poly);
    b2uApplyBodyMassIfNeeded(bw->bodyId, &sd);
}

void b2uCreateCircleShape(B2U_Body body, float cx, float cy, float radius, float density, float friction, float restitution, int32_t layer)
{
    if (!body) return;
    BodyWrap* bw = (BodyWrap*)body;

    b2ShapeDef sd = b2DefaultShapeDef();
    b2uApplyShapeParams(&sd, density, friction, restitution, layer);

    b2Circle circle;
    circle.center = { cx, cy };
    circle.radius = radius;
    b2CreateCircleShape(bw->bodyId, &sd, &circle);
    b2uApplyBodyMassIfNeeded(bw->bodyId, &sd);
}

void b2uCreatePolygonShape(B2U_Body body, const B2U_Vec2* vertices, int32_t count, float radius, float density, float friction, float restitution, int32_t layer)
{
    if (!body || !vertices || count < 3 || count > B2_MAX_POLYGON_VERTICES) return;
    BodyWrap* bw = (BodyWrap*)body;

    b2Vec2 points[B2_MAX_POLYGON_VERTICES];
    for (int32_t i = 0; i < count; ++i)
    {
        points[i] = { vertices[i].x, vertices[i].y };
    }

    b2Hull hull = b2ComputeHull(points, count);
    if (hull.count < 3) return;

    b2ShapeDef sd = b2DefaultShapeDef();
    b2uApplyShapeParams(&sd, density, friction, restitution, layer);

    b2Polygon poly = b2MakePolygon(&hull, radius);
    b2CreatePolygonShape(bw->bodyId, &sd, &poly);
    b2uApplyBodyMassIfNeeded(bw->bodyId, &sd);
}

void b2uCreateEdgeShape(B2U_Body body, float ax, float ay, float bx, float by, float density, float friction, float restitution, int32_t layer)
{
    if (!body) return;
    BodyWrap* bw = (BodyWrap*)body;

    b2ShapeDef sd = b2DefaultShapeDef();
    b2uApplyShapeParams(&sd, density, friction, restitution, layer);

    b2Segment seg;
    seg.point1 = { ax, ay };
    seg.point2 = { bx, by };
    b2CreateSegmentShape(bw->bodyId, &sd, &seg);
    b2uApplyBodyMassIfNeeded(bw->bodyId, &sd);
}

B2U_Body b2uCreateDynamicBox(
    B2U_World world,
    float px, float py,
    float hx, float hy,
    float density)
{
    if (!world) return nullptr;

    B2U_BodyDef bd = {};
    bd.type = B2U_Dynamic;
    bd.position = { px, py };
    bd.angle = 0.0f;
    bd.linearVelocity = { 0.0f, 0.0f };
    bd.angularVelocity = 0.0f;
    bd.gravityScale = 1.0f;
    bd.linearDamping = 0.0f;
    bd.angularDamping = 0.0f;
    bd.allowSleep = 1;
    bd.fixedRotation = 0;
    bd.isBullet = 0;

    B2U_Body body = b2uCreateBody(world, &bd);
    b2uCreateBoxShape(body, hx, hy, density, 0.6f, 0.0f, 0);
    return body;
}

void b2uGetBodyTransform(
    B2U_Body body,
    float* px,
    float* py,
    float* angle)
{
    if (!body) return;
    BodyWrap* bw = (BodyWrap*)body;

    b2Transform xf = b2Body_GetTransform(bw->bodyId);

    if (px) *px = xf.p.x;
    if (py) *py = xf.p.y;
    if (angle) *angle = std::atan2(xf.q.s, xf.q.c);
}

void b2uGetBodyTransformsBatch(
    const B2U_Body* bodies,
    int32_t count,
    float* outPx,
    float* outPy,
    float* outAngle)
{
    if (!bodies || count <= 0) return;

    for (int32_t i = 0; i < count; ++i)
    {
        const B2U_Body body = bodies[i];
        if (!body)
        {
            if (outPx) outPx[i] = 0.0f;
            if (outPy) outPy[i] = 0.0f;
            if (outAngle) outAngle[i] = 0.0f;
            continue;
        }

        BodyWrap* bw = (BodyWrap*)body;
        b2Transform xf = b2Body_GetTransform(bw->bodyId);
        if (outPx) outPx[i] = xf.p.x;
        if (outPy) outPy[i] = xf.p.y;
        if (outAngle) outAngle[i] = std::atan2(xf.q.s, xf.q.c);
    }
}

void b2uDestroyBody(B2U_Body body)
{
    if (!body) return;
    BodyWrap* bw = (BodyWrap*)body;

    b2DestroyBody(bw->bodyId);
    delete bw;
}

void b2uDestroyBodies(B2U_Body* bodies, int32_t count)
{
    if (!bodies || count <= 0) return;
    for (int32_t i = 0; i < count; ++i)
    {
        b2uDestroyBody(bodies[i]);
        bodies[i] = nullptr;
    }
}
