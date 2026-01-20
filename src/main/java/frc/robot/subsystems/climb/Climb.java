// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.climb.ClimberConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.linear.LinearIO;
import frc.robot.lib.subsystem.linear.LinearSubsystem;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Climb extends VirtualSubsystem {
  private final LinearSubsystem climber;

  @Getter private ClimbState targetState = ClimbState.kStowed;
  @Getter private ClimbState measuredState;

  /** Creates a new Climb. */
  public Climb() {
    this(new LinearSubsystem(new LinearIO() {}, ClimberConstants.kSubsystemConfigReal));
  }

  public Climb(LinearSubsystem climber) {
    this.climber = climber;

    climber.setDefaultCommand(climber.holdAtGoal(() -> getTargetState().getClimber()));

    measuredState = new ClimbState(climber.getLength());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setClimber(climber.getLength());

    Logger.recordOutput("Climb/TargetState", targetState);
    Logger.recordOutput("Climb/MeasuredState", measuredState);
  }

  public Command waitUntilAtGoal() {
    return sequence(waitSeconds(RobotConstants.kDt), waitUntil(climber.atLength()));
  }

  public Command set(ClimbState state) {
    return set(() -> state);
  }

  public Command set(Supplier<ClimbState> state) {
    return Commands.parallel(
        Commands.runOnce(() -> this.targetState = state.get()),
        climber.length(() -> state.get().getClimber()));
  }
}
