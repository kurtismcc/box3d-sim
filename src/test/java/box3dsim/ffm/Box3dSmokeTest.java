package box3dsim.ffm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import box3dsim.ffm.gen.Box3D;
import box3dsim.ffm.gen.b3BodyDef;
import box3dsim.ffm.gen.b3ShapeDef;
import box3dsim.ffm.gen.b3Sphere;
import box3dsim.ffm.gen.b3SurfaceMaterial;
import box3dsim.ffm.gen.b3Vec3;
import box3dsim.ffm.gen.b3WorldDef;
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import org.junit.jupiter.api.Test;

class Box3dSmokeTest {
    private static void setVec3(MemorySegment v, float x, float y, float z) {
        b3Vec3.x(v, x);
        b3Vec3.y(v, y);
        b3Vec3.z(v, z);
    }

    @Test
    void ballRestsOnFloor() {
        NativeLibrary.load();
        assertTrue(NativeLibrary.version().startsWith("0."), NativeLibrary.version());

        try (Arena arena = Arena.ofConfined()) {
            MemorySegment worldDef = Box3D.b3DefaultWorldDef(arena);
            setVec3(b3WorldDef.gravity(worldDef), 0f, 0f, -9.81f);
            b3WorldDef.workerCount(worldDef, 1);
            MemorySegment world = Box3D.b3CreateWorld(arena, worldDef);

            MemorySegment groundDef = Box3D.b3DefaultBodyDef(arena);
            setVec3(b3BodyDef.position(groundDef), 0f, 0f, -0.5f);
            MemorySegment ground = Box3D.b3CreateBody(arena, world, groundDef);
            MemorySegment box = Box3D.b3MakeBoxHull(arena, 10f, 10f, 0.5f);
            Box3D.b3CreateHullShape(arena, ground, Box3D.b3DefaultShapeDef(arena), box);

            MemorySegment ballDef = Box3D.b3DefaultBodyDef(arena);
            b3BodyDef.type(ballDef, Box3D.b3_dynamicBody());
            setVec3(b3BodyDef.position(ballDef), 0f, 0f, 1.0f);
            MemorySegment ball = Box3D.b3CreateBody(arena, world, ballDef);

            float radius = 0.075f;
            MemorySegment sphere = b3Sphere.allocate(arena);
            setVec3(b3Sphere.center(sphere), 0f, 0f, 0f);
            b3Sphere.radius(sphere, radius);
            MemorySegment shapeDef = Box3D.b3DefaultShapeDef(arena);
            b3ShapeDef.density(shapeDef, (float) (0.215 / (4.0 / 3.0 * Math.PI * radius * radius * radius)));
            b3SurfaceMaterial.restitution(b3ShapeDef.baseMaterial(shapeDef), 0f);
            Box3D.b3CreateSphereShape(arena, ball, shapeDef, sphere);

            for (int i = 0; i < 120; i++) {
                Box3D.b3World_Step(world, 1f / 60f, 4);
            }

            MemorySegment pos = Box3D.b3Body_GetPosition(arena, ball);
            assertEquals(0.075, b3Vec3.z(pos), 0.01);
            assertEquals(0.0, b3Vec3.x(pos), 1e-3);
            assertEquals(0.0, b3Vec3.y(pos), 1e-3);

            Box3D.b3DestroyWorld(world);
        }
    }
}
