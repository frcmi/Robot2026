// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.intake.PivotConstants;
import frc.robot.constants.intake.RollerConstants;
import frc.robot.constants.shooter.FieldConstants;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends VirtualSubsystem {
  private final AngularSubsystem rollers;
  private final AngularSubsystem pivot;
  private final Supplier<Pose2d> robotPose;
  private Alliance alliance = Alliance.Red;
  private final Trigger nearBump = new Trigger(this::isNearBump).debounce(0.05);
  private final BooleanSupplier shooting;
  private final BooleanSupplier isAutonomous;

  private final Timer oscillationTimer = new Timer();

  private final LoggedTunableNumber oscillationPeriod =
      new LoggedTunableNumber("Intake/OscillationPeriodS", 1.8);
  private final LoggedTunableNumber oscillationDutyCycle =
      new LoggedTunableNumber("Intake/OscillationDutyCycle", 0.7);
  private final LoggedTunableNumber oscillationInitialDelay =
      new LoggedTunableNumber("Intake/OscillationInitialDelay", 1.0);

  @Getter private IntakeState targetState = IntakeState.kStowed;
  @Getter private IntakeState measuredState;

  /** Creates a new Intake. */
  public Intake(Supplier<Pose2d> robotPose, BooleanSupplier shooting) {
    this(
        new AngularSubsystem(new AngularIO() {}, RollerConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, PivotConstants.kSubsystemConfigReal),
        robotPose,
        shooting,
        () -> false);
  }

  public Intake(
      AngularSubsystem rollers,
      AngularSubsystem pivot,
      Supplier<Pose2d> robotPoseSupplier,
      BooleanSupplier shooting,
      BooleanSupplier isAutonomous) {
    this.rollers = rollers;
    this.pivot = pivot;
    this.robotPose = robotPoseSupplier;
    this.shooting = shooting;
    this.isAutonomous =
        isAutonomous; // Unused, but may be useful at some point so just leaving it in

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> gatedTarget().getPivot()));
    rollers.setDefaultCommand(rollers.openLoop(() -> gatedTarget().getRollers()));
    this.setDefaultCommand(this.set(IntakeState.kStowed));

    measuredState = new IntakeState(pivot.getAngle(), targetState.getRollers());
  }

  private boolean prevOscillating = false;

  private IntakeState gatedTarget() {
    if (nearBump.getAsBoolean()) {
      prevOscillating = false;
      return IntakeState.kStowed;
    }
    if (shooting.getAsBoolean() && this.targetState == IntakeState.kStowed) {
      if (!prevOscillating) {
        oscillationTimer.restart();
        prevOscillating = true;
      }

      // Oscillate
      double timeNow = oscillationTimer.get();
      double per = oscillationPeriod.get();
      double initialDelay = oscillationInitialDelay.get();
      double duty = oscillationDutyCycle.get();
      boolean intakeUp =
          timeNow < initialDelay ? false : (timeNow - initialDelay) % per < per * duty;
      return new IntakeState(
          intakeUp ? IntakeState.kTransferring.getPivot() : IntakeState.kStowed.getPivot(),
          IntakeState.kTransferring.getRollers());
    } else {
      prevOscillating = false;
    }
    return targetState;
  }

  @Override
  public void periodic() {
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

  public Command zeroPivot() {
    return pivot.resetAngle(IntakeState.kDown.getPivot());
  }

  public Command openLoopPivot(Voltage volts) {
    return this.pivot.openLoop(() -> volts);
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
