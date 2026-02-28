package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.subsystems.transfer.TransferState;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FuelSim extends VirtualSubsystem {
  private final Supplier<ShooterState> shooterState;
  private final BooleanSupplier isShooting;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotSpeeds;

  public ArrayList<Pose3d> fuel = new ArrayList<>();
  private ArrayList<Translation3d> fuelVelocities = new ArrayList<>();
  private Timer lastShot = new Timer();

  // Constants
  LoggedTunableNumber kLaunchHeight =
      new LoggedTunableNumber("FuelSim/LaunchHeightM", Inches.of(22).in(Meters));
  LoggedTunableNumber kHubHeight =
      new LoggedTunableNumber("FuelSim/HubHeightM", Inches.of(65).in(Meters));
  LoggedTunableNumber kBPS = new LoggedTunableNumber("FuelSim/BPS", 5);
  LoggedTunableNumber kGravity = new LoggedTunableNumber("FuelSim/GravityMS^2", -9.81);
  LoggedTunableNumber kExitVelocity =
      new LoggedTunableNumber("FuelSim/ExitVelocityMSPerRadS", 0.035);

  public FuelSim(
      Supplier<ShooterState> shooterState,
      BooleanSupplier isShooting,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotSpeeds) {
    this.shooterState = shooterState;
    this.robotPose = robotPose;
    this.robotSpeeds = robotSpeeds;
    this.isShooting = isShooting;
    lastShot.start();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("FuelSim/Fuel", fuel.toArray(new Pose3d[0]));

    // Update all existing fuel
    for (int i = 0; i < fuel.size(); i++) {
      Pose3d f = fuel.get(i);
      Translation3d v = fuelVelocities.get(i);

      // Update position based on velocity
      fuel.set(
          i,
          new Pose3d(
              f.getX() + v.getX() * RobotConstants.kDt,
              f.getY() + v.getY() * RobotConstants.kDt,
              f.getZ() + v.getZ() * RobotConstants.kDt,
              f.getRotation()));

      // Update velocity based on gravity
      fuelVelocities.set(
          i, new Translation3d(v.getX(), v.getY(), v.getZ() + kGravity.get() * RobotConstants.kDt));

      // Remove fuel below hub height and falling
      if (fuel.get(i).getZ() < kHubHeight.get() && fuelVelocities.get(i).getZ() < 0) {
        fuel.remove(i);
        fuelVelocities.remove(i);
        i--;
      }
    }

    // Add new fuel if shooting
    if (isShooting.getAsBoolean() && lastShot.hasElapsed(1 / kBPS.get())) {
      lastShot.restart();

      Pose2d rPose = robotPose.get();
      ChassisSpeeds rSpeeds = robotSpeeds.get();

      ShooterState sState = shooterState.get();

      Pose3d turretPoseWorld =
          new Pose3d(rPose)
              .plus(
                  new Transform3d(
                      TurretConstants.TurretOffset,
                      new Rotation3d(0.0, 0.0, sState.getTurret().in(Radians))));

      fuel.add(
          new Pose3d(
              turretPoseWorld.getX(),
              turretPoseWorld.getY(),
              kLaunchHeight.get(),
              new Rotation3d()));

      double exitVel = kExitVelocity.get() * sState.getFlywheel().in(RadiansPerSecond);
      double turretGlobal =
          sState.getTurret().in(Radians)
              + rPose.getRotation().getRadians()
              + TurretConstants.kTurretZero.in(Radians);

      double hoodAngle = sState.getHood().in(Radians);
      double horizSpeed = exitVel * Math.sin(hoodAngle);

      double vx = horizSpeed * Math.cos(turretGlobal) + rSpeeds.vxMetersPerSecond;
      double vy = horizSpeed * Math.sin(turretGlobal) + rSpeeds.vyMetersPerSecond;
      double vz = exitVel * Math.cos(hoodAngle);

      fuelVelocities.add(new Translation3d(vx, vy, vz));
    }
  }
}
