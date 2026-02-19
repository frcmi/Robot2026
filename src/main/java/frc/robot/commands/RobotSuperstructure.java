package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferState;

public class RobotSuperstructure {
  private final Intake intake;
  private final Transfer transfer;
  private final Climb climb;
  private final Shooter shooter;

  public RobotSuperstructure(Intake intake, Transfer transfer, Climb climb, Shooter shooter) {
    this.intake = intake;
    this.transfer = transfer;
    this.climb = climb;
    this.shooter = shooter;
  }

  public void registerAutoCommands() {
    NamedCommands.registerCommand("ClimbRaise", climbRaise());
    NamedCommands.registerCommand("Climb", climbClimbed());
    NamedCommands.registerCommand(
        "DeployIntake",
        parallel(intake.set(IntakeState.kIntaking), transfer.set(TransferState.kTransferring)));
  }

  public Command climbRaise() {
    return parallel(climb.set(ClimbState.kRaised), idle()).withDeadline(climb.waitUntilAtGoal());
  }

  public Command climbClimbed() {
    return parallel(climb.set(ClimbState.kClimbed), idle()).withDeadline(climb.waitUntilAtGoal());
  }
}
