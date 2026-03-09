// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transfer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.transfer.KickerConstants;
import frc.robot.constants.transfer.TransferConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Transfer extends VirtualSubsystem {
  private final AngularSubsystem transfer;
  private final AngularSubsystem kicker;

  @Getter private TransferState targetState = TransferState.kIdle;
  @Getter private TransferState measuredState;

  private final Trigger aimed;
  private final Timer jammedTimer = new Timer();
  private final Timer unJamTimer = new Timer();
  private boolean jammed = false;

  /** Creates a new Transfer. */
  public Transfer(Trigger aimed) {
    this(
        new AngularSubsystem(new AngularIO() {}, TransferConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, KickerConstants.kSubsystemConfigReal),
        aimed);
  }

  public Transfer(AngularSubsystem transfer, AngularSubsystem kicker, Trigger aimed) {
    this.transfer = transfer;
    this.kicker = kicker;
    this.aimed = aimed;

    transfer.setDefaultCommand(transfer.openLoop(() -> targetStateAimed().getTransfer()));
    kicker.setDefaultCommand(kicker.velocity(() -> targetStateAimed().getKicker()));

    this.setDefaultCommand(this.set(TransferState.kIdle));

    measuredState = new TransferState(targetState.getTransfer(), targetState.getKicker());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setTransfer(targetStateAimed().getTransfer());
    measuredState.setKicker(targetStateAimed().getKicker());

    Logger.recordOutput("Transfer/TargetState", targetState);
    Logger.recordOutput("Transfer/MeasuredState", measuredState);
    Logger.recordOutput("Transfer/Jammed", jammed);
    Logger.recordOutput("Transfer/ShouldTransfer", targetState == TransferState.kTransferring);

    updateJamDetection();
  }

  private TransferState targetStateAimed() {
    if (jammed) {
      return TransferState.kReverse;
    }
    if (aimed.getAsBoolean()) {
      return getTargetState();
    }
    return TransferState.kIdle;
  }

  public boolean isAttemptingShooting() {
    return this.targetState == TransferState.kTransferring;
  }

  public boolean isShooting() {
    return (measuredState.getKicker().baseUnitMagnitude() > 0);
  }

  public Command set(TransferState state) {
    return set(() -> state);
  }

  public Command set(Supplier<TransferState> state) {
    return Commands.run(() -> this.targetState = state.get(), this);
  }

  private void updateJamDetection() {
    if (jammedTimer.isRunning() && jammedTimer.hasElapsed(Seconds.of(0.4))) {
      jammed = true;
      unJamTimer.start();
      jammedTimer.stop();
      jammedTimer.reset();
    }
    if (jammed && unJamTimer.isRunning() && unJamTimer.hasElapsed(0.1)) {
      jammed = false;
      unJamTimer.stop();
      unJamTimer.reset();
    }
    if (transfer.getVelocity().abs(RotationsPerSecond) < 0.5
        && targetState == TransferState.kTransferring
        && aimed.getAsBoolean()
        && !jammed) {
      jammedTimer.start();
    } else if (jammedTimer.isRunning()) {
      jammedTimer.stop();
      jammedTimer.reset();
    }
  }
}
