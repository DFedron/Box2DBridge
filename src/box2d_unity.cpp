#include "box2d_unity.h"
#include "box2d/box2d.h"
#include <cmath>    // ✅ for std::atan2

struct WorldWrap
{
    b2WorldId worldId;
};

struct BodyWrap
{
    b2BodyId bodyId;
};

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
    if (!world) return;
    WorldWrap* w = (WorldWrap*)world;

    // ✅ 你这份版本能编过 b2World_Step(worldId, dt, iterations)
    // （如果你后续想暴露更多参数，我们再扩展）
    b2World_Step(w->worldId, dt, 8);
}

B2U_Body b2uCreateDynamicBox(
    B2U_World world,
    float px, float py,
    float hx, float hy,
    float density)
{
    if (!world) return nullptr;
    WorldWrap* w = (WorldWrap*)world;

    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = { px, py };

    b2BodyId bodyId = b2CreateBody(w->worldId, &bd);

    b2ShapeDef sd = b2DefaultShapeDef();
    sd.density = density;

    b2Polygon poly = b2MakeBox(hx, hy);
    b2CreatePolygonShape(bodyId, &sd, &poly);

    BodyWrap* bw = new BodyWrap();
    bw->bodyId = bodyId;
    return (B2U_Body)bw;
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

    // ✅ 你的版本 b2Rot 没有 GetAngle()，用 atan2(s, c) 最稳
    // b2Rot 通常有字段：c = cos(theta), s = sin(theta)
    if (angle) *angle = std::atan2(xf.q.s, xf.q.c);
}

void b2uDestroyBody(B2U_Body body)
{
    if (!body) return;
    BodyWrap* bw = (BodyWrap*)body;

    // ⚠️ 这里假设你这份 3.x 有 b2DestroyBody(b2BodyId)
    // 如果你编译提示找不到该函数，把报错贴出来，我会按你版本改成正确的 API 名。
    b2DestroyBody(bw->bodyId);

    delete bw;
}
