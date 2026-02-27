// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.intake.PivotConstants;
import frc.robot.constants.intake.RollerConstants;
import frc.robot.constants.shooter.FieldConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends VirtualSubsystem {
  private final AngularSubsystem rollers;
  private final AngularSubsystem pivot;
  private Supplier<Pose2d> robotPose;
  private Alliance alliance = Alliance.Red;
  private Trigger nearBump = new Trigger(this::isNearBump).debounce(0.05);

  @Getter private IntakeState targetState = IntakeState.kStowed;
  @Getter private IntakeState measuredState;

  /** Creates a new Intake. */
  public Intake(Supplier<Pose2d> robotPose) {
    this(
        new AngularSubsystem(new AngularIO() {}, RollerConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, PivotConstants.kSubsystemConfigReal),
        robotPose);
  }

  public Intake(
      AngularSubsystem rollers, AngularSubsystem pivot, Supplier<Pose2d> robotPoseSupplier) {
    this.rollers = rollers;
    this.pivot = pivot;
    this.robotPose = robotPoseSupplier;

    pivot.setDefaultCommand(
        pivot.holdAtGoal(
            () -> nearBump.getAsBoolean() ? IntakeState.kStowed.getPivot() : getTargetState().getPivot()));
    rollers.setDefaultCommand(rollers.openLoop(() -> getTargetState().getRollers()));
    this.setDefaultCommand(this.set(IntakeState.kStowed));

    measuredState = new IntakeState(pivot.getAngle(), targetState.getRollers());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setPivot(pivot.getAngle());
    measuredState.setRollers(targetState.getRollers());

    Logger.recordOutput("Intake/TargetState", targetState);
    Logger.recordOutput("Intake/MeasuredState", measuredState);
  }

  public Command waitUntilAtGoal() {
    return sequence(waitSeconds(RobotConstants.kDt), waitUntil(pivot.atAngle()));
  }

  public Command set(IntakeState state) {
    return set(() -> state);
  }

  public Command set(Supplier<IntakeState> state) {
    return Commands.run(() -> this.targetState = state.get(), this);
  }

  private boolean isNearBump() {
    Pose2d currentPose = this.robotPose.get();
    Translation2d hubPosition =
        alliance == Alliance.Blue
            ? FieldConstants.kHubPositionBlue
            : FieldConstants.kHubPositionRed;

    // Check X
    boolean nearX =
        Math.abs(currentPose.getX() - hubPosition.getX()) < (FieldConstants.trenchWidthX / 2.0);
    boolean nearY =
        (currentPose.getY() > FieldConstants.trenchWidthY
                && currentPose.getY() < (FieldConstants.bumpWidthY + FieldConstants.trenchWidthY))
            || (currentPose.getY()
                    > FieldConstants.trenchWidthY
                        + FieldConstants.bumpWidthY
                        + FieldConstants.hubWidthY
                && currentPose.getY() < (FieldConstants.fieldWidthY - FieldConstants.trenchWidthY));
    return nearX && nearY;
  }
}
