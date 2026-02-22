// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.shooter.AimingConstants;
import frc.robot.constants.shooter.FieldConstants;
import frc.robot.constants.shooter.FlywheelConstants;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.lib.alliancecolor.AllianceUpdatedObserver;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.utils.AngleUtils;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Shooter extends VirtualSubsystem implements AllianceUpdatedObserver {
  private final AngularSubsystem turret;
  private final AngularSubsystem hood;
  private final AngularSubsystem flywheel;

  @Getter private ShooterState targetState = ShooterState.kStowed;
  @Getter private ShooterState measuredState;

  private Alliance alliance = Alliance.Red;

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotVel;

  private boolean disabled = false;

  // For detecting whether aimed or not
  double targetDist = 0.0;
  private Angle rawTurretTarget = ShooterState.kStowed.getTurret();
  private Angle rawHoodTarget = ShooterState.kStowed.getHood();

  // Triggers
  public Trigger nearTrench = new Trigger(this::isNearTrench).debounce(0.05);
  public Trigger inAllianceZone = new Trigger(this::isInAllianceZone).debounce(0.2);
  public Trigger aimed = new Trigger(this::isAimed).debounce(0.05);

  /** Creates a new Shooter. */
  public Shooter(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVel) {
    this(
        new AngularSubsystem(new AngularIO() {}, TurretConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, HoodConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, FlywheelConstants.kSubsystemConfigReal),
        robotPose,
        robotVel);
  }

  public Shooter(
      AngularSubsystem turret,
      AngularSubsystem hood,
      AngularSubsystem flywheel,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelSupplier) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.robotPose = robotPoseSupplier;
    this.robotVel = robotVelSupplier;
    hood.setDefaultCommand(hood.holdAtGoal(() -> getTargetState().getHood()));
    turret.setDefaultCommand(turret.holdAtGoal(() -> getTargetState().getTurret()));
    flywheel.setDefaultCommand(flywheel.velocity(() -> getTargetState().getFlywheel()));
    measuredState = new ShooterState(turret.getAngle(), hood.getAngle(), flywheel.getVelocity());
  }

  public void onAllianceFound(Alliance alliance) {
    this.alliance = alliance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setTurret(turret.getAngle());
    measuredState.setHood(hood.getAngle());
    measuredState.setFlywheel(flywheel.getVelocity());

    Logger.recordOutput("Shooter/TargetState", targetState);
    Logger.recordOutput("Shooter/MeasuredState", measuredState);
    Logger.recordOutput("Shooter/Disabled", disabled);
    Logger.recordOutput("Shooter/NearTrench", nearTrench.getAsBoolean());
    Logger.recordOutput("Shooter/InAllianceZone", inAllianceZone.getAsBoolean());
    Logger.recordOutput("Shooter/Aimed", aimed.getAsBoolean());

    // Aim at hub
    Translation2d targetPosition;
    if (inAllianceZone.getAsBoolean()) {
      targetPosition =
        alliance == Alliance.Blue
            ? FieldConstants.kHubPositionBlue
            : FieldConstants.kHubPositionRed;
    } else {
      double targetX = alliance == Alliance.Blue ? FieldConstants.allianceZoneXBlue : FieldConstants.allianceZoneXRed;
      double targetY = robotPose.get().getY() < FieldConstants.kHubPositionBlue.getY() ? FieldConstants.allianceZoneYBottom : FieldConstants.allianceZoneYTop;
      targetPosition = new Translation2d(targetX, targetY);
    }
    Logger.recordOutput("Shooter/Target", targetPosition);

    // Calculate turret angle to target
    Pose2d currentPose = this.robotPose.get();

    // rotate turret offsets by bot heading to convert to field-centric offsets
    Translation2d turretOffset =
        new Translation2d(TurretConstants.TurretOffset.getX(), TurretConstants.TurretOffset.getY())
            .rotateBy(currentPose.getRotation());

    // Calculate aiming position iteratively
    double dx = targetPosition.getX() - (currentPose.getX() + turretOffset.getX());
    double dy = targetPosition.getY() - (currentPose.getY() + turretOffset.getY());
    targetDist = Math.hypot(dx, dy);
    ChassisSpeeds robotVelocity = robotVel.get();
    // Found that it converges over 2 iterations, but do 5 to be safe
    for (int i = 0; i < 5; i++) {
      double airtime = AimingConstants.kAirtimeTable.get(targetDist);
      dx =
          targetPosition.getX()
              - (currentPose.getX() + turretOffset.getX())
              - robotVelocity.vxMetersPerSecond * airtime;
      dy =
          targetPosition.getY()
              - (currentPose.getY() + turretOffset.getY())
              - robotVelocity.vyMetersPerSecond * airtime;
      targetDist = Math.hypot(dx, dy);
    }

    Logger.recordOutput("Shooter/DistanceToTargetM", targetDist);

    double angleToTarget = Math.atan2(dy, dx);
    Angle turretTarget =
        Radians.of(angleToTarget - currentPose.getRotation().getRadians() + Math.toRadians(180.0));

    // Wrap around to [-180, 180]
    turretTarget = AngleUtils.normalize(turretTarget);
    rawTurretTarget = turretTarget.copy();

    // Constrain to turret limits
    turretTarget =
        Radians.of(
            MathUtil.clamp(
                turretTarget.in(Radians),
                Degrees.of(AimingConstants.kTurretMinAngle.getAsDouble()).in(Radians),
                Degrees.of(AimingConstants.kTurretMaxAngle.getAsDouble()).in(Radians)));

    // Actually apply to hardware
    this.targetState.setTurret(turretTarget);
    double hoodAngle = AimingConstants.kHoodAngleTable.get(targetDist);
    rawHoodTarget = Degrees.of(hoodAngle);
    this.targetState.setHood(nearTrench.getAsBoolean() ? HoodConstants.kMinHoodAngle : Degrees.of(hoodAngle));
    double flywheelRPS = AimingConstants.kFlywheelSpeedTable.get(targetDist);
    this.targetState.setFlywheel(
        disabled ? RotationsPerSecond.of(0) : RotationsPerSecond.of(flywheelRPS));
  }

  public Command toggleDisabled() {
    return either(
        runOnce(
            () -> {
              disabled = false;
            }),
        runOnce(
            () -> {
              disabled = true;
            }),
        () -> disabled);
  }


  // Trench code
  private boolean isNearTrench() {
    Pose2d currentPose = this.robotPose.get();
     Translation2d hubPosition =
        alliance == Alliance.Blue
            ? FieldConstants.kHubPositionBlue
            : FieldConstants.kHubPositionRed;
    
    // Check X
    boolean nearX = Math.abs(currentPose.getX() - hubPosition.getX()) < (FieldConstants.trenchWidthX / 2.0);
    boolean nearY = currentPose.getY() < FieldConstants.trenchWidthY || currentPose.getY() > (FieldConstants.fieldWidthY - FieldConstants.trenchWidthY);
    return nearX && nearY;
  }

  private boolean isInAllianceZone() {
    Pose2d currentPose = this.robotPose.get();
    if (alliance == Alliance.Blue) {
      return currentPose.getX() < FieldConstants.kHubPositionBlue.getX();
    } else {
      return currentPose.getX() > FieldConstants.kHubPositionRed.getX();
    }
  }
  
  private boolean isAimed() {
    double turretErr = rawTurretTarget.minus(measuredState.getTurret()).abs(Radians);
    double errorAtTarget = targetDist * Math.sin(turretErr);
    double hoodErr = rawHoodTarget.minus(measuredState.getHood()).abs(Degrees);
    double flywheelErr = measuredState.getFlywheel().minus(targetState.getFlywheel()).abs(RotationsPerSecond);
    if (inAllianceZone.getAsBoolean()) {
      return errorAtTarget < (FieldConstants.hubWidth / 2.0) && hoodErr < 3.0 && flywheelErr < 1.6; // Degrees, rotations per second
    } else {
      // In neutral zone, more lenient since just tryna get into the alliance zone
      return errorAtTarget < (FieldConstants.bumpWidth / 2.0) && hoodErr < 5.0 && flywheelErr < 2.0;
    }
  }
}
