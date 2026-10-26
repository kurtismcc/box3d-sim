package box3dsim.world;

import box3dsim.math.Pose3;
import box3dsim.math.Vec3;
import java.util.List;
import java.util.Optional;

/** Backend-neutral rigid-body world. Not thread-safe; use from one thread. */
public interface SimWorld extends AutoCloseable {
    BodyHandle createBody(BodySpec spec);

    /** Also destroys the body's shapes. */
    void destroyBody(BodyHandle body);

    ShapeHandle addShape(BodyHandle body, ShapeSpec shape, ShapeOptions options);

    BodyHandle bodyOf(ShapeHandle shape);

    /** Advances dt seconds; the backend splits it into subSteps solver substeps. */
    void step(double dt, int subSteps);

    Pose3 pose(BodyHandle body);

    /** Teleports and wakes the body. */
    void setPose(BodyHandle body, Pose3 pose);

    Vec3 linearVelocity(BodyHandle body);

    Vec3 angularVelocity(BodyHandle body);

    void setVelocity(BodyHandle body, Vec3 linear, Vec3 angular);

    Vec3 pointVelocity(BodyHandle body, Vec3 worldPoint);

    /** Applies for the next step only and wakes the body. */
    void applyForce(BodyHandle body, Vec3 force, Vec3 worldPoint);

    void applyForceToCenter(BodyHandle body, Vec3 force);

    void applyTorque(BodyHandle body, Vec3 torque);

    /** Overrides shape-derived mass. inertiaDiagonal is about the local center, in body axes. */
    void setMassProperties(BodyHandle body, double mass, Vec3 localCenter, Vec3 inertiaDiagonal);

    double mass(BodyHandle body);

    boolean isAwake(BodyHandle body);

    /** Disabled bodies leave the simulation but keep their ids. */
    void setEnabled(BodyHandle body, boolean enabled);

    /** Valid until the next step(). */
    List<SensorEvent> sensorBeginEvents();

    /** Valid until the next step(). */
    List<SensorEvent> sensorEndEvents();

    /** Bodies that moved in the last step. Valid until the next step(). */
    List<BodyMove> bodyMoves();

    /** Shapes currently overlapping a sensor shape. */
    List<ShapeHandle> sensorOverlaps(ShapeHandle sensor);

    Optional<RayHit> castRayClosest(Vec3 origin, Vec3 translation, CollisionFilter filter);

    /** Destroys the world; idempotent. */
    @Override
    void close();
}
