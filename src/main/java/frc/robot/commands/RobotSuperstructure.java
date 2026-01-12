package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

public class RobotSuperstructure {
  private final Intake intake;

  public RobotSuperstructure(Intake intake) {
    this.intake = intake;
  }

  public void registerAutoCommands() {
    NamedCommands.registerCommand("intakeDeploy", intakeDeploy());
    NamedCommands.registerCommand("intakeStow", intakeStowed());
  }

  public Command intakeStowed() {
    return parallel(intake.set(IntakeState.kStowed), idle()).withDeadline(intake.waitUntilAtGoal());
  }

  public Command intakeDeploy() {
    return parallel(intake.set(IntakeState.kIntaking), idle())
        .withDeadline(intake.waitUntilAtGoal());
  }
}
