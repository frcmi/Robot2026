// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.intake.KickerConstants;
import frc.robot.constants.intake.PivotConstants;
import frc.robot.constants.intake.RollerConstants;
import frc.robot.constants.intake.TransferConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends VirtualSubsystem {
  private final AngularSubsystem rollers;
  private final AngularSubsystem pivot;
  private final AngularSubsystem transfer;
  private final AngularSubsystem kicker;

  @Getter private IntakeState targetState = IntakeState.kStowed;
  @Getter private IntakeState measuredState;

  /** Creates a new Intake. */
  public Intake() {
    this(
        new AngularSubsystem(new AngularIO() {}, RollerConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, PivotConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, TransferConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, KickerConstants.kSubsystemConfigReal));
  }

  public Intake(
      AngularSubsystem rollers,
      AngularSubsystem pivot,
      AngularSubsystem transfer,
      AngularSubsystem kicker) {
    this.rollers = rollers;
    this.pivot = pivot;
    this.transfer = transfer;
    this.kicker = kicker;

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> getTargetState().getPivot()));
    rollers.setDefaultCommand(rollers.openLoop(() -> getTargetState().getRollers()));
    transfer.setDefaultCommand(transfer.openLoop(() -> getTargetState().getTransfer()));
    kicker.setDefaultCommand(kicker.openLoop(() -> getTargetState().getKicker()));

    measuredState =
        new IntakeState(
            pivot.getAngle(),
            targetState.getRollers(),
            targetState.getTransfer(),
            targetState.getKicker());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setPivot(pivot.getAngle());
    measuredState.setRollers(targetState.getRollers());
    measuredState.setTransfer(targetState.getTransfer());
    measuredState.setKicker(targetState.getKicker());

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
    return parallel(
        Commands.runOnce(() -> this.targetState = state.get()),
        pivot.angle(() -> state.get().getPivot()),
        rollers.openLoop(() -> state.get().getRollers()),
        transfer.openLoop(() -> state.get().getTransfer()),
        kicker.openLoop(() -> state.get().getKicker()));
  }
}
