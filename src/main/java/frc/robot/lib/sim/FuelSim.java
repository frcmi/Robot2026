package frc.robot.lib.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
      // do partial step to get to ground
      double deltaZ = position.getZ() - newPosition.getZ();
      double allowedDeltaZ = position.getZ() - RADIUS;
      double tPartial = allowedDeltaZ / deltaZ;

      // do second partial step to bounce back up
      Pose3d newVelocity =
          new Pose3d(
              velocity.getX(),
              velocity.getY(),
              -velocity.getZ() * elasticityCoefficient,
              velocity.getRotation());
      double endZ = newVelocity.times((1 - tPartial) * deltaTime).getZ() + RADIUS;

      // reassign values
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
              velocity.getRotation()); // TODO: make this coefficient exist and work
    }

    this.lastTime = Timer.getTimestamp();
  }

  public Pose3d getPose() {
    return position;
  }

  public static FuelSim generateShotFuel(
      Pose3d turretPose,
      ChassisSpeeds robotVelocity,
      AngularVelocity flywheelSpeed,
      Angle hoodAngle) {
    Translation3d robotVel =
        new Translation3d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond, 0.0);

    // linear velocity = 0.5 * rps * circ
    double linearMetersPerSecond =
        flywheelSpeed.in(Revolutions.per(Second)) * Units.inchesToMeters(4) * Math.PI * 0.5;
    // convert to translational and vertical
    double hoodRadians = hoodAngle.in(Radians);

    double translationalVelocity = linearMetersPerSecond * Math.sin(hoodRadians);
    double verticalVelocity = linearMetersPerSecond * Math.cos(hoodRadians);

    return new FuelSim(
        new Pose3d(turretPose.getX(), turretPose.getY(), Units.inchesToMeters(8), new Rotation3d()),
        new Pose3d(
            new Translation3d(translationalVelocity, 0.0, verticalVelocity)
                .rotateBy(
                    turretPose.getRotation().plus(new Rotation3d(Rotation2d.fromDegrees(180))))
                .plus(robotVel),
            new Rotation3d()));
  }
}
