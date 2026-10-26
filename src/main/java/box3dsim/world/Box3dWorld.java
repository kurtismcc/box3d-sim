package box3dsim.world;

import box3dsim.ffm.NativeLibrary;
import box3dsim.ffm.gen.Box3D;
import box3dsim.ffm.gen.b3BodyDef;
import box3dsim.ffm.gen.b3BodyEvents;
import box3dsim.ffm.gen.b3BodyMoveEvent;
import box3dsim.ffm.gen.b3Capsule;
import box3dsim.ffm.gen.b3Filter;
import box3dsim.ffm.gen.b3MassData;
import box3dsim.ffm.gen.b3Matrix3;
import box3dsim.ffm.gen.b3MeshDef;
import box3dsim.ffm.gen.b3MotionLocks;
import box3dsim.ffm.gen.b3Quat;
import box3dsim.ffm.gen.b3QueryFilter;
import box3dsim.ffm.gen.b3RayResult;
import box3dsim.ffm.gen.b3SensorBeginTouchEvent;
import box3dsim.ffm.gen.b3SensorEndTouchEvent;
import box3dsim.ffm.gen.b3SensorEvents;
import box3dsim.ffm.gen.b3ShapeDef;
import box3dsim.ffm.gen.b3Sphere;
import box3dsim.ffm.gen.b3SurfaceMaterial;
import box3dsim.ffm.gen.b3Transform;
import box3dsim.ffm.gen.b3Vec3;
import box3dsim.ffm.gen.b3WorldDef;
import box3dsim.math.Pose3;
import box3dsim.math.Quat;
import box3dsim.math.Vec3;
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** SimWorld backed by Box3D through FFM. */
public final class Box3dWorld implements SimWorld {
    private static final long ID_BYTES = 8;
    private static final long VEC3_BYTES = b3Vec3.layout().byteSize();

    private final Arena arena = Arena.ofShared();
    private final ScratchAllocator scratch = new ScratchAllocator(arena, 64 * 1024);
    private final MemorySegment worldId;
    private final List<MemorySegment> meshes = new ArrayList<>();
    private boolean closed;

    private List<SensorEvent> sensorBegin = List.of();
    private List<SensorEvent> sensorEnd = List.of();
    private List<BodyMove> moves = List.of();

    public Box3dWorld(WorldConfig config) {
        NativeLibrary.load();
        MemorySegment def = Box3D.b3DefaultWorldDef(scratch);
        writeVec3(b3WorldDef.gravity(def), config.gravity());
        b3WorldDef.workerCount(def, config.workerCount());
        b3WorldDef.enableSleep(def, config.enableSleep());
        b3WorldDef.enableContinuous(def, config.enableContinuous());
        b3WorldDef.restitutionThreshold(def, (float) config.restitutionThreshold());
        b3WorldDef.maximumLinearSpeed(def, (float) config.maximumLinearSpeed());
        MemorySegment id = Box3D.b3CreateWorld(scratch, def);
        worldId = arena.allocate(id.byteSize(), 4);
        worldId.copyFrom(id);
    }

    @Override
    public BodyHandle createBody(BodySpec spec) {
        scratch.reset();
        MemorySegment def = Box3D.b3DefaultBodyDef(scratch);
        b3BodyDef.type(def, switch (spec.type()) {
            case STATIC -> Box3D.b3_staticBody();
            case KINEMATIC -> Box3D.b3_kinematicBody();
            case DYNAMIC -> Box3D.b3_dynamicBody();
        });
        writeVec3(b3BodyDef.position(def), spec.pose().position());
        writeQuat(b3BodyDef.rotation(def), spec.pose().rotation());
        writeVec3(b3BodyDef.linearVelocity(def), spec.linearVelocity());
        writeVec3(b3BodyDef.angularVelocity(def), spec.angularVelocity());
        b3BodyDef.linearDamping(def, (float) spec.linearDamping());
        b3BodyDef.angularDamping(def, (float) spec.angularDamping());
        b3BodyDef.gravityScale(def, (float) spec.gravityScale());
        b3BodyDef.isBullet(def, spec.bullet());
        b3BodyDef.enableSleep(def, spec.enableSleep());
        b3BodyDef.isAwake(def, spec.awake());
        MemorySegment locks = b3BodyDef.motionLocks(def);
        MotionLocks l = spec.locks();
        b3MotionLocks.linearX(locks, l.linearX());
        b3MotionLocks.linearY(locks, l.linearY());
        b3MotionLocks.linearZ(locks, l.linearZ());
        b3MotionLocks.angularX(locks, l.angularX());
        b3MotionLocks.angularY(locks, l.angularY());
        b3MotionLocks.angularZ(locks, l.angularZ());
        if (spec.name() != null) {
            b3BodyDef.name(def, arena.allocateFrom(spec.name()));
        }
        return new BodyHandle(readId(Box3D.b3CreateBody(scratch, worldId, def)));
    }

    @Override
    public void destroyBody(BodyHandle body) {
        scratch.reset();
        Box3D.b3DestroyBody(id(body.id()));
    }

    @Override
    public ShapeHandle addShape(BodyHandle body, ShapeSpec shape, ShapeOptions options) {
        scratch.reset();
        MemorySegment bodyId = id(body.id());
        MemorySegment def = Box3D.b3DefaultShapeDef(scratch);
        MemorySegment material = b3ShapeDef.baseMaterial(def);
        b3SurfaceMaterial.friction(material, (float) options.material().friction());
        b3SurfaceMaterial.restitution(material, (float) options.material().restitution());
        b3SurfaceMaterial.rollingResistance(material, (float) options.material().rollingResistance());
        b3ShapeDef.density(def, (float) options.density());
        MemorySegment filter = b3ShapeDef.filter(def);
        b3Filter.categoryBits(filter, options.filter().categoryBits());
        b3Filter.maskBits(filter, options.filter().maskBits());
        b3ShapeDef.isSensor(def, options.isSensor());
        b3ShapeDef.enableSensorEvents(def, options.enableSensorEvents());
        b3ShapeDef.updateBodyMass(def, options.updateBodyMass());

        MemorySegment shapeId = switch (shape) {
            case ShapeSpec.Sphere s -> {
                MemorySegment sphere = b3Sphere.allocate(scratch);
                writeVec3(b3Sphere.center(sphere), s.center());
                b3Sphere.radius(sphere, (float) s.radius());
                yield Box3D.b3CreateSphereShape(scratch, bodyId, def, sphere);
            }
            case ShapeSpec.Capsule c -> {
                MemorySegment capsule = b3Capsule.allocate(scratch);
                writeVec3(b3Capsule.center1(capsule), c.p1());
                writeVec3(b3Capsule.center2(capsule), c.p2());
                b3Capsule.radius(capsule, (float) c.radius());
                yield Box3D.b3CreateCapsuleShape(scratch, bodyId, def, capsule);
            }
            case ShapeSpec.Box b -> {
                Vec3 h = b.halfExtents();
                MemorySegment hull = Box3D.b3MakeTransformedBoxHull(
                        scratch, (float) h.x(), (float) h.y(), (float) h.z(), transform(b.localPose()));
                // b3BoxHull starts with its b3HullData base.
                yield Box3D.b3CreateHullShape(scratch, bodyId, def, hull);
            }
            case ShapeSpec.Hull h -> {
                MemorySegment points = scratch.allocate(b3Vec3.layout(), h.points().size());
                for (int i = 0; i < h.points().size(); i++) {
                    writeVec3(points.asSlice(i * VEC3_BYTES, VEC3_BYTES), h.points().get(i));
                }
                MemorySegment hull = Box3D.b3CreateHull(points, h.points().size(), 64);
                if (hull.address() == 0) {
                    throw new IllegalArgumentException("Degenerate hull: " + h.points());
                }
                MemorySegment created = Box3D.b3CreateTransformedHullShape(
                        scratch, bodyId, def, hull, transform(h.localPose()), vec3(new Vec3(1, 1, 1)));
                Box3D.b3DestroyHull(hull);
                yield created;
            }
            case ShapeSpec.Mesh m -> {
                // Mesh data must outlive the shape, so it lives in the world arena.
                MemorySegment vertices = arena.allocate(b3Vec3.layout(), m.vertices().size());
                for (int i = 0; i < m.vertices().size(); i++) {
                    writeVec3(vertices.asSlice(i * VEC3_BYTES, VEC3_BYTES), m.vertices().get(i));
                }
                MemorySegment indices = arena.allocateFrom(ValueLayout.JAVA_INT, m.triangleIndices());
                MemorySegment meshDef = b3MeshDef.allocate(arena);
                b3MeshDef.vertices(meshDef, vertices);
                b3MeshDef.stride(meshDef, VEC3_BYTES);
                b3MeshDef.indices(meshDef, indices);
                b3MeshDef.materialIndices(meshDef, MemorySegment.NULL);
                b3MeshDef.vertexCount(meshDef, m.vertices().size());
                b3MeshDef.triangleCount(meshDef, m.triangleIndices().length / 3);
                MemorySegment mesh = Box3D.b3CreateMesh(meshDef, MemorySegment.NULL, 0);
                if (mesh.address() == 0) {
                    throw new IllegalArgumentException("Failed to create mesh");
                }
                meshes.add(mesh);
                yield Box3D.b3CreateMeshShape(scratch, bodyId, def, mesh, vec3(new Vec3(1, 1, 1)));
            }
        };
        return new ShapeHandle(readId(shapeId));
    }

    @Override
    public BodyHandle bodyOf(ShapeHandle shape) {
        scratch.reset();
        return new BodyHandle(readId(Box3D.b3Shape_GetBody(scratch, id(shape.id()))));
    }

    @Override
    public void step(double dt, int subSteps) {
        scratch.reset();
        Box3D.b3World_Step(worldId, (float) dt, subSteps);

        MemorySegment sensors = Box3D.b3World_GetSensorEvents(scratch, worldId);
        sensorBegin = readSensorEvents(
                b3SensorEvents.beginEvents(sensors), b3SensorEvents.beginCount(sensors), true);
        sensorEnd = readSensorEvents(b3SensorEvents.endEvents(sensors), b3SensorEvents.endCount(sensors), false);

        MemorySegment bodyEvents = Box3D.b3World_GetBodyEvents(scratch, worldId);
        int moveCount = b3BodyEvents.moveCount(bodyEvents);
        long size = b3BodyMoveEvent.layout().byteSize();
        MemorySegment all = b3BodyEvents.moveEvents(bodyEvents).reinterpret(moveCount * size);
        List<BodyMove> list = new ArrayList<>(moveCount);
        for (int i = 0; i < moveCount; i++) {
            MemorySegment e = all.asSlice(i * size, size);
            list.add(new BodyMove(
                    new BodyHandle(readId(b3BodyMoveEvent.bodyId(e))),
                    readTransform(b3BodyMoveEvent.transform(e)),
                    b3BodyMoveEvent.fellAsleep(e)));
        }
        moves = List.copyOf(list);
    }

    private List<SensorEvent> readSensorEvents(MemorySegment ptr, int count, boolean begin) {
        if (count == 0) {
            return List.of();
        }
        long size = begin ? b3SensorBeginTouchEvent.layout().byteSize() : b3SensorEndTouchEvent.layout().byteSize();
        MemorySegment all = ptr.reinterpret(count * size);
        List<SensorEvent> list = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            MemorySegment e = all.asSlice(i * size, size);
            MemorySegment sensor = begin ? b3SensorBeginTouchEvent.sensorShapeId(e) : b3SensorEndTouchEvent.sensorShapeId(e);
            MemorySegment visitor =
                    begin ? b3SensorBeginTouchEvent.visitorShapeId(e) : b3SensorEndTouchEvent.visitorShapeId(e);
            list.add(new SensorEvent(new ShapeHandle(readId(sensor)), new ShapeHandle(readId(visitor))));
        }
        return List.copyOf(list);
    }

    @Override
    public Pose3 pose(BodyHandle body) {
        scratch.reset();
        return readTransform(Box3D.b3Body_GetTransform(scratch, id(body.id())));
    }

    @Override
    public void setPose(BodyHandle body, Pose3 pose) {
        scratch.reset();
        MemorySegment b = id(body.id());
        Box3D.b3Body_SetTransform(b, vec3(pose.position()), quat(pose.rotation()));
        Box3D.b3Body_SetAwake(b, true);
    }

    @Override
    public Vec3 linearVelocity(BodyHandle body) {
        scratch.reset();
        return readVec3(Box3D.b3Body_GetLinearVelocity(scratch, id(body.id())));
    }

    @Override
    public Vec3 angularVelocity(BodyHandle body) {
        scratch.reset();
        return readVec3(Box3D.b3Body_GetAngularVelocity(scratch, id(body.id())));
    }

    @Override
    public void setVelocity(BodyHandle body, Vec3 linear, Vec3 angular) {
        scratch.reset();
        MemorySegment b = id(body.id());
        Box3D.b3Body_SetLinearVelocity(b, vec3(linear));
        Box3D.b3Body_SetAngularVelocity(b, vec3(angular));
    }

    @Override
    public Vec3 pointVelocity(BodyHandle body, Vec3 worldPoint) {
        scratch.reset();
        return readVec3(Box3D.b3Body_GetWorldPointVelocity(scratch, id(body.id()), vec3(worldPoint)));
    }

    @Override
    public void applyForce(BodyHandle body, Vec3 force, Vec3 worldPoint) {
        scratch.reset();
        Box3D.b3Body_ApplyForce(id(body.id()), vec3(force), vec3(worldPoint), true);
    }

    @Override
    public void applyForceToCenter(BodyHandle body, Vec3 force) {
        scratch.reset();
        Box3D.b3Body_ApplyForceToCenter(id(body.id()), vec3(force), true);
    }

    @Override
    public void applyTorque(BodyHandle body, Vec3 torque) {
        scratch.reset();
        Box3D.b3Body_ApplyTorque(id(body.id()), vec3(torque), true);
    }

    @Override
    public void setMassProperties(BodyHandle body, double mass, Vec3 localCenter, Vec3 inertiaDiagonal) {
        scratch.reset();
        MemorySegment data = b3MassData.allocate(scratch);
        b3MassData.mass(data, (float) mass);
        writeVec3(b3MassData.center(data), localCenter);
        MemorySegment inertia = b3MassData.inertia(data);
        writeVec3(b3Matrix3.cx(inertia), new Vec3(inertiaDiagonal.x(), 0, 0));
        writeVec3(b3Matrix3.cy(inertia), new Vec3(0, inertiaDiagonal.y(), 0));
        writeVec3(b3Matrix3.cz(inertia), new Vec3(0, 0, inertiaDiagonal.z()));
        Box3D.b3Body_SetMassData(id(body.id()), data);
    }

    @Override
    public double mass(BodyHandle body) {
        scratch.reset();
        return Box3D.b3Body_GetMass(id(body.id()));
    }

    @Override
    public boolean isAwake(BodyHandle body) {
        scratch.reset();
        return Box3D.b3Body_IsAwake(id(body.id()));
    }

    @Override
    public void setEnabled(BodyHandle body, boolean enabled) {
        scratch.reset();
        if (enabled) {
            Box3D.b3Body_Enable(id(body.id()));
        } else {
            Box3D.b3Body_Disable(id(body.id()));
        }
    }

    @Override
    public List<SensorEvent> sensorBeginEvents() {
        return sensorBegin;
    }

    @Override
    public List<SensorEvent> sensorEndEvents() {
        return sensorEnd;
    }

    @Override
    public List<BodyMove> bodyMoves() {
        return moves;
    }

    @Override
    public List<ShapeHandle> sensorOverlaps(ShapeHandle sensor) {
        scratch.reset();
        MemorySegment s = id(sensor.id());
        int capacity = Box3D.b3Shape_GetSensorCapacity(s);
        if (capacity <= 0) {
            return List.of();
        }
        MemorySegment ids = scratch.allocate(capacity * ID_BYTES, 8);
        int n = Box3D.b3Shape_GetSensorData(s, ids, capacity);
        List<ShapeHandle> out = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            out.add(new ShapeHandle(ids.get(ValueLayout.JAVA_LONG_UNALIGNED, i * ID_BYTES)));
        }
        return out;
    }

    @Override
    public Optional<RayHit> castRayClosest(Vec3 origin, Vec3 translation, CollisionFilter filter) {
        scratch.reset();
        MemorySegment qf = Box3D.b3DefaultQueryFilter(scratch);
        b3QueryFilter.categoryBits(qf, filter.categoryBits());
        b3QueryFilter.maskBits(qf, filter.maskBits());
        MemorySegment r = Box3D.b3World_CastRayClosest(scratch, worldId, vec3(origin), vec3(translation), qf);
        if (!b3RayResult.hit(r)) {
            return Optional.empty();
        }
        return Optional.of(new RayHit(
                new ShapeHandle(readId(b3RayResult.shapeId(r))),
                readVec3(b3RayResult.point(r)),
                readVec3(b3RayResult.normal(r)),
                b3RayResult.fraction(r)));
    }

    @Override
    public void close() {
        if (closed) {
            return;
        }
        closed = true;
        Box3D.b3DestroyWorld(worldId);
        for (MemorySegment mesh : meshes) {
            Box3D.b3DestroyMesh(mesh);
        }
        meshes.clear();
        arena.close();
    }

    // FFM conversions. Box3D is single precision; doubles are narrowed here only.

    private MemorySegment id(long value) {
        MemorySegment seg = scratch.allocate(ID_BYTES, 4);
        seg.set(ValueLayout.JAVA_LONG_UNALIGNED, 0, value);
        return seg;
    }

    private static long readId(MemorySegment seg) {
        return seg.get(ValueLayout.JAVA_LONG_UNALIGNED, 0);
    }

    private MemorySegment vec3(Vec3 v) {
        MemorySegment seg = b3Vec3.allocate(scratch);
        writeVec3(seg, v);
        return seg;
    }

    private MemorySegment quat(Quat q) {
        MemorySegment seg = b3Quat.allocate(scratch);
        writeQuat(seg, q);
        return seg;
    }

    private MemorySegment transform(Pose3 pose) {
        MemorySegment seg = b3Transform.allocate(scratch);
        writeVec3(b3Transform.p(seg), pose.position());
        writeQuat(b3Transform.q(seg), pose.rotation());
        return seg;
    }

    private static void writeVec3(MemorySegment seg, Vec3 v) {
        b3Vec3.x(seg, (float) v.x());
        b3Vec3.y(seg, (float) v.y());
        b3Vec3.z(seg, (float) v.z());
    }

    /** b3Quat is {b3Vec3 v; float s}: vector part first. */
    private static void writeQuat(MemorySegment seg, Quat q) {
        Quat n = q.normalized();
        writeVec3(b3Quat.v(seg), new Vec3(n.x(), n.y(), n.z()));
        b3Quat.s(seg, (float) n.w());
    }

    private static Vec3 readVec3(MemorySegment seg) {
        return new Vec3(b3Vec3.x(seg), b3Vec3.y(seg), b3Vec3.z(seg));
    }

    private static Quat readQuat(MemorySegment seg) {
        Vec3 v = readVec3(b3Quat.v(seg));
        return new Quat(b3Quat.s(seg), v.x(), v.y(), v.z());
    }

    private static Pose3 readTransform(MemorySegment seg) {
        return new Pose3(readVec3(b3Transform.p(seg)), readQuat(b3Transform.q(seg)));
    }
}
