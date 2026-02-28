package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
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
    NamedCommands.registerCommand("Shoot", transfer.set(TransferState.kTransferring));

    new EventTrigger("Intake").whileTrue(intake.set(IntakeState.kIntaking));
    new EventTrigger("Shoot").whileTrue(transfer.set(TransferState.kTransferring));
  }

  public Command climbRaise() {
    return parallel(climb.set(ClimbState.kRaised), idle()).withDeadline(climb.waitUntilAtGoal());
  }

  public Command climbClimbed() {
    return parallel(climb.set(ClimbState.kStowed), idle()).withDeadline(climb.waitUntilAtGoal());
  }

  public Command lockHoodDown() {
    return runOnce(() -> shooter.setHoodLocked(true));
  }

  public Command unlockHood() {
    return runOnce(() -> shooter.setHoodLocked(false));
  }

  public double getDriveSpeed(boolean rotation) {
    boolean intaking = intake.getTargetState() == IntakeState.kIntaking;
    boolean transferring = transfer.getTargetState() == TransferState.kTransferring;
    double speed = rotation ? DriveConstants.MAX_SPEED_W : DriveConstants.MAX_SPEED;
    if (transferring) {
      return speed * (rotation ? DriveConstants.TRANSFER_MULT_W : DriveConstants.TRANSFER_MULT);
    }
    if (intaking) {
      return speed * (rotation ? DriveConstants.INTAKE_MULT_W : DriveConstants.INTAKE_MULT);
    }
    return speed;
  }
}
