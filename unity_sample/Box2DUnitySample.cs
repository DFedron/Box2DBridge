using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class Box2DUnitySample : MonoBehaviour
{
    private const string DllName = "box2d_unity"; // adjust to your plugin name

    [StructLayout(LayoutKind.Sequential)]
    public struct B2U_Vec2
    {
        public float x;
        public float y;
    }

    public enum B2U_BodyType : int
    {
        Static = 0,
        Kinematic = 1,
        Dynamic = 2
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct B2U_BodyDef
    {
        public int type;
        public B2U_Vec2 position;
        public float angle;
        public B2U_Vec2 linearVelocity;
        public float angularVelocity;
        public float gravityScale;
        public float linearDamping;
        public float angularDamping;
        public byte allowSleep;
        public byte fixedRotation;
        public byte isBullet;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct B2U_BoxBodyDef
    {
        public B2U_BodyDef body;
        public float hx;
        public float hy;
        public float density;
        public float friction;
        public float restitution;
        public int layer;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct B2U_CircleBodyDef
    {
        public B2U_BodyDef body;
        public float cx;
        public float cy;
        public float radius;
        public float density;
        public float friction;
        public float restitution;
        public int layer;
    }

    [DllImport(DllName)] private static extern IntPtr b2uCreateWorld(float gx, float gy);
    [DllImport(DllName)] private static extern void b2uDestroyWorld(IntPtr world);
    [DllImport(DllName)] private static extern void b2uStep(IntPtr world, float dt);
    [DllImport(DllName)] private static extern void b2uStepEx(IntPtr world, float dt, int subSteps);
    [DllImport(DllName)] private static extern void b2uSetWorldGravity(IntPtr world, float gx, float gy);
    [DllImport(DllName)] private static extern void b2uGetWorldGravity(IntPtr world, out float gx, out float gy);

    [DllImport(DllName)] private static extern void b2uResetLayerFilters();
    [DllImport(DllName)] private static extern void b2uSetLayerFilter(int layer, ulong categoryBits, ulong maskBits, int groupIndex);

    [DllImport(DllName)] private static extern IntPtr b2uCreateBody(IntPtr world, ref B2U_BodyDef def);
    [DllImport(DllName)] private static extern int b2uCreateBodies(IntPtr world, [In] B2U_BodyDef[] defs, int count, [Out] IntPtr[] outBodies);
    [DllImport(DllName)] private static extern int b2uCreateBoxBodies(IntPtr world, [In] B2U_BoxBodyDef[] defs, int count, [Out] IntPtr[] outBodies);
    [DllImport(DllName)] private static extern int b2uCreateCircleBodies(IntPtr world, [In] B2U_CircleBodyDef[] defs, int count, [Out] IntPtr[] outBodies);

    [DllImport(DllName)] private static extern void b2uCreateBoxShape(IntPtr body, float hx, float hy, float density, float friction, float restitution, int layer);
    [DllImport(DllName)] private static extern void b2uCreateCircleShape(IntPtr body, float cx, float cy, float radius, float density, float friction, float restitution, int layer);
    [DllImport(DllName)] private static extern void b2uCreatePolygonShape(IntPtr body, [In] B2U_Vec2[] vertices, int count, float radius, float density, float friction, float restitution, int layer);
    [DllImport(DllName)] private static extern void b2uCreateEdgeShape(IntPtr body, float ax, float ay, float bx, float by, float density, float friction, float restitution, int layer);

    [DllImport(DllName)] private static extern void b2uGetBodyTransform(IntPtr body, out float px, out float py, out float angle);
    [DllImport(DllName)] private static extern void b2uGetBodyTransformsBatch([In] IntPtr[] bodies, int count, [Out] float[] outPx, [Out] float[] outPy, [Out] float[] outAngle);

    [DllImport(DllName)] private static extern void b2uDestroyBody(IntPtr body);
    [DllImport(DllName)] private static extern void b2uDestroyBodies([In, Out] IntPtr[] bodies, int count);

    private IntPtr _world;
    private IntPtr[] _bodies;
    private float[] _px;
    private float[] _py;
    private float[] _angle;

    private void Start()
    {
        _world = b2uCreateWorld(0.0f, -9.81f);
        b2uSetWorldGravity(_world, 0.0f, -9.81f);

        b2uResetLayerFilters();
        b2uSetLayerFilter(0, 1UL << 0, ulong.MaxValue, 0); // default layer
        b2uSetLayerFilter(1, 1UL << 1, (1UL << 0) | (1UL << 1), 0); // example layer

        // Batch create boxes
        var defs = new B2U_BoxBodyDef[3];
        for (int i = 0; i < defs.Length; ++i)
        {
            defs[i].body = MakeBodyDef(B2U_BodyType.Dynamic, new Vector2(i * 1.5f, 5.0f));
            defs[i].hx = 0.5f;
            defs[i].hy = 0.5f;
            defs[i].density = 1.0f;
            defs[i].friction = 0.6f;
            defs[i].restitution = 0.0f;
            defs[i].layer = 0;
        }

        _bodies = new IntPtr[defs.Length];
        b2uCreateBoxBodies(_world, defs, defs.Length, _bodies);

        // Add a circle shape to the first body
        b2uCreateCircleShape(_bodies[0], 0.0f, 0.0f, 0.25f, 1.0f, 0.6f, 0.1f, 0);

        // Create a polygon shape on a separate body
        var polyBody = b2uCreateBody(_world, ref MakeBodyDef(B2U_BodyType.Dynamic, new Vector2(0.0f, 8.0f)));
        var verts = new B2U_Vec2[]
        {
            new B2U_Vec2 { x = -0.5f, y = -0.5f },
            new B2U_Vec2 { x =  0.5f, y = -0.5f },
            new B2U_Vec2 { x =  0.0f, y =  0.6f },
        };
        b2uCreatePolygonShape(polyBody, verts, verts.Length, 0.0f, 1.0f, 0.6f, 0.0f, 0);

        // Create an edge shape (static ground)
        var groundDef = MakeBodyDef(B2U_BodyType.Static, new Vector2(0.0f, 0.0f));
        var ground = b2uCreateBody(_world, ref groundDef);
        b2uCreateEdgeShape(ground, -10.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.6f, 0.0f, 0);

        _px = new float[_bodies.Length];
        _py = new float[_bodies.Length];
        _angle = new float[_bodies.Length];
    }

    private void FixedUpdate()
    {
        b2uStepEx(_world, Time.fixedDeltaTime, 8);
        b2uGetBodyTransformsBatch(_bodies, _bodies.Length, _px, _py, _angle);

        for (int i = 0; i < _bodies.Length; ++i)
        {
            // Example: update a Unity transform if you have one
            // transform.position = new Vector3(_px[i], _py[i], 0f);
            // transform.rotation = Quaternion.Euler(0f, 0f, _angle[i] * Mathf.Rad2Deg);
        }
    }

    private void OnDestroy()
    {
        if (_bodies != null && _bodies.Length > 0)
        {
            b2uDestroyBodies(_bodies, _bodies.Length);
            _bodies = null;
        }

        if (_world != IntPtr.Zero)
        {
            b2uDestroyWorld(_world);
            _world = IntPtr.Zero;
        }
    }

    private static B2U_BodyDef MakeBodyDef(B2U_BodyType type, Vector2 pos)
    {
        return new B2U_BodyDef
        {
            type = (int)type,
            position = new B2U_Vec2 { x = pos.x, y = pos.y },
            angle = 0.0f,
            linearVelocity = new B2U_Vec2 { x = 0.0f, y = 0.0f },
            angularVelocity = 0.0f,
            gravityScale = 1.0f,
            linearDamping = 0.0f,
            angularDamping = 0.0f,
            allowSleep = 1,
            fixedRotation = 0,
            isBullet = 0
        };
    }
}
