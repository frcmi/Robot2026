package frc.robot.lib.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class FuelSim {

  public static double DEFAULT_ELASTICITY_COEFFICIENT = 0.4; // bounce speed reduction
  public static double TRANSLATIONAL_DRAG = 0.1; // aerial drag
  public static double RADIAL_DRAG = 0.1;
  public static double TRANSLATIONAL_FRICTION = 0.6; // grounded friction
  public static double RADIUS = Units.inchesToMeters(6.1);
  public static double G = 9.81;

  public Pose3d position; // m
  public Pose3d velocity; // m/s
  public double lastTime = 0; // s
  public double elasticityCoefficient;

  public FuelSim(Pose3d position, Pose3d velocity, double elasticityCoefficient) {
    this.position = position;
    this.velocity = velocity;
    this.elasticityCoefficient = elasticityCoefficient;
    lastTime = Timer.getTimestamp();
  }

  public FuelSim(Pose3d position, Pose3d velocity) {
    this(position, velocity, DEFAULT_ELASTICITY_COEFFICIENT);
  }

  public FuelSim() {
    this(new Pose3d(), new Pose3d());
  }

  public void update() {
    // velocity update
    double deltaTime = Timer.getTimestamp() - lastTime;
    // timestep is small enough we can ignore gravity integration
    Pose3d scaledVelocity = velocity.times(deltaTime);

    Pose3d newPosition =
        new Pose3d(
            position.getTranslation().plus(scaledVelocity.getTranslation()),
            position.getRotation().plus(scaledVelocity.getRotation()));

    // check for bounce
    if (newPosition.getZ() < RADIUS) {
      // do partial step
      double deltaZ = position.getZ() - newPosition.getZ();
      double allowedDeltaZ = position.getZ() - RADIUS;
      double tPartial = allowedDeltaZ / deltaZ;

      // do second partial step
      Pose3d newVelocity =
          new Pose3d(
              velocity.getX(),
              velocity.getY(),
              -velocity.getZ() * elasticityCoefficient,
              velocity.getRotation());
      double endZ = newVelocity.times((1 - tPartial) * deltaTime).getZ() + RADIUS;
      velocity = newVelocity;
      position =
          new Pose3d(newPosition.getX(), newPosition.getY(), endZ, newPosition.getRotation());
    } else {
      position = newPosition;
    }

    // gravity and drag
    velocity =
        new Pose3d(
            new Translation3d(0, 0, -G * deltaTime).plus(velocity.getTranslation()),
            velocity.getRotation());

    velocity =
        new Pose3d(
            velocity.getTranslation().times(Math.pow(1 - TRANSLATIONAL_DRAG, deltaTime)),
            velocity.getRotation().times(Math.pow(1 - RADIAL_DRAG, deltaTime)));

    // grounded-specific forces
    if (position.getZ() <= RADIUS + 0.1) {
      velocity =
          new Pose3d(
              velocity.getTranslation().times(Math.pow(1 - TRANSLATIONAL_FRICTION, deltaTime)),
              velocity.getRotation()); // TODO: make this coefficient work
    }

    this.lastTime = Timer.getTimestamp();
  }

  public Pose3d getPose() {
    return position;
  }
}
