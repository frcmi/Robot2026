// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transfer;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.transfer.KickerConstants;
import frc.robot.constants.transfer.TransferConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.subsystems.intake.IntakeState;

import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Transfer extends VirtualSubsystem {
  private final AngularSubsystem transfer;
  private final AngularSubsystem kicker;

  @Getter private TransferState targetState = TransferState.kIdle;
  @Getter private TransferState measuredState;

  /** Creates a new Transfer. */
  public Transfer() {
    this(
        new AngularSubsystem(new AngularIO() {}, TransferConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, KickerConstants.kSubsystemConfigReal));
  }

  public Transfer(AngularSubsystem transfer, AngularSubsystem kicker) {
    this.transfer = transfer;
    this.kicker = kicker;

    transfer.setDefaultCommand(transfer.openLoop(() -> getTargetState().getTransfer()));
    kicker.setDefaultCommand(kicker.openLoop(() -> getTargetState().getKicker()));

    this.setDefaultCommand(this.set(TransferState.kIdle));

    measuredState = new TransferState(targetState.getTransfer(), targetState.getKicker());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setTransfer(targetState.getTransfer());
    measuredState.setKicker(targetState.getKicker());

    Logger.recordOutput("Transfer/TargetState", targetState);
    Logger.recordOutput("Transfer/MeasuredState", measuredState);
  }

  public Command set(TransferState state) {
    return set(() -> state);
  }

  public Command set(Supplier<TransferState> state) {
    return parallel(
        Commands.runOnce(() -> this.targetState = state.get()),
        transfer.openLoop(() -> state.get().getTransfer()),
        kicker.openLoop(() -> state.get().getKicker()));
  }
}
