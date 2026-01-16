// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.Shooter.AimingConstants;
import frc.robot.constants.Shooter.FlywheelConstants;
import frc.robot.constants.Shooter.HoodConstants;
import frc.robot.constants.Shooter.TurretConstants;
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

  /** Creates a new Shooter. */
  public Shooter(Supplier<Pose2d> robotPose) {
    this(
        new AngularSubsystem(new AngularIO() {}, TurretConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, HoodConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, FlywheelConstants.kSubsystemConfigReal),
        robotPose);
  }

  public Shooter(
      AngularSubsystem turret,
      AngularSubsystem hood,
      AngularSubsystem flywheel,
      Supplier<Pose2d> robotPoseSupplier) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.robotPose = robotPoseSupplier;

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

    // Aim at hub
    Translation2d hubPosition =
        alliance == Alliance.Blue
            ? AimingConstants.kHubPositionBlue
            : AimingConstants.kHubPositionRed;

    this.targetState.setTurret(getTurretAngleToTarget(hubPosition));

    // TODO: Implement InterpLUTs
    this.targetState.setHood(Degrees.of(10.0f));
    this.targetState.setFlywheel(RotationsPerSecond.of(60)); // 3600RPM
  }

  /**
   * @return Necessary turret position to aim at the target position.
   */
  public Angle getTurretAngleToTarget(Translation2d target) {
    Pose2d currentPose = this.robotPose.get();

    // rotate turret offsets by bot heading to convert to field-centric offsets
    Translation2d turretOffset =
        new Translation2d(TurretConstants.TurretOffset.getX(), TurretConstants.TurretOffset.getY())
            .rotateBy(currentPose.getRotation());

    double dx = target.getX() - currentPose.getX() + turretOffset.getX();
    double dy = target.getY() - currentPose.getY() + turretOffset.getY();
    double angleToTarget = Math.atan2(dy, dx);

    Logger.recordOutput(
        "Shooter/TestRad", currentPose.getRotation().getRadians()); // temporary logging

    Angle turretTarget =
        Radians.of(angleToTarget - currentPose.getRotation().getRadians() + Math.toRadians(180.0));

    // Wrap around to [-180, 180]
    turretTarget = AngleUtils.normalize(turretTarget);

    Angle unconstrainedTurretTarget = turretTarget;
    // Constrain to turret limits
    turretTarget =
        Radians.of(
            MathUtil.clamp(
                turretTarget.in(Radians),
                TurretConstants.kTurretMinAngle.in(Radians),
                TurretConstants.kTurretMaxAngle.in(Radians)));

    Logger.recordOutput(
        "Shooter/Debug/UnconstrainedTargetDegrees", unconstrainedTurretTarget.in(Degrees));
    Logger.recordOutput(
        "Shooter/Debug/IsTargetConstrained",
        unconstrainedTurretTarget.in(Degrees) == turretTarget.in(Degrees));
    return turretTarget;
  }

  public Command waitUntilAtGoal() {
    return sequence(
        waitSeconds(RobotConstants.kDt),
        waitUntil(turret.atAngle()),
        waitUntil(flywheel.atAngle()));
  }
}
