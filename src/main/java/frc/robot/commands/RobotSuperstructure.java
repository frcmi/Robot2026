package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.shooter.HoodConstants;
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
        "WaitAimed",
        new WaitUntilCommand(shooter.aimed)
            .beforeStarting(() -> shooter.setHoodUnlocked(false))
            .alongWith(shooter.forceToggleState())
            .asProxy());
    NamedCommands.registerCommand(
        "Shoot",
        transfer
            .set(TransferState.kTransferring)
            .alongWith(shooter.forceToggleState())
            .beforeStarting(() -> shooter.setHoodUnlocked(false))
            .finallyDo(() -> shooter.setHoodUnlocked(true))
            .asProxy());
    NamedCommands.registerCommand("FullRobotCheck", this.fullRobotCheck());

    new EventTrigger("Intake").whileTrue(intake.set(IntakeState.kIntaking));
    new EventTrigger("Shoot")
        .whileTrue(transfer.set(TransferState.kTransferring).alongWith(shooter.forceToggleState()));
  }

  public Command climbRaise() {
    return parallel(climb.set(ClimbState.kRaised), idle()).withDeadline(climb.waitUntilAtGoal());
  }

  public Command climbClimbed() {
    return parallel(climb.set(ClimbState.kStowed), idle()).withDeadline(climb.waitUntilAtGoal());
  }

  public double getDriveMultiplier(boolean rotation, Trigger turbo) {
    boolean intaking = intake.getTargetState() == IntakeState.kIntaking;
    boolean transferring = transfer.getTargetState() == TransferState.kTransferring;

    // Default: Shooting speed
    double speed;
    if (shooter.inAllianceZone.getAsBoolean()) {
      speed = rotation ? DriveConstants.TRANSFER_SPEED_W : DriveConstants.TRANSFER_SPEED;
    } else {
      speed =
          rotation
              ? DriveConstants.TRANSFER_SPEED_W_NEUTRAL
              : DriveConstants.TRANSFER_SPEED_NEUTRAL;
    }

    // If in turbo mode, use max speed (assuming not transferring)
    if (turbo.getAsBoolean() && !transferring) {
      speed = rotation ? DriveConstants.MAX_SPEED_W : DriveConstants.MAX_SPEED;
      if (intaking) { // Intake speed multiplier only applies when full speed
        speed *= (rotation ? DriveConstants.INTAKE_MULT_W : DriveConstants.INTAKE_MULT);
      }
    }
    return speed;
  }

  public Command fullRobotCheck() {
    return sequence(
        // INIT SHOOTER
        shooter.forceToggleState().asProxy(),
        shooter.toggleDisabled().asProxy(),
        shooter.toggleOverride().asProxy(),
        shooter
            .overrideHood(HoodConstants.MANUAL_OVERRIDE)
            .withTimeout(1.0)
            .andThen(shooter.zeroHood())
            .asProxy(),

        // INTAKE TESTS
        intake.set(IntakeState.kDown).withTimeout(1).asProxy(),
        intake.zeroPivot(),
        intake.set(IntakeState.kIntaking).withTimeout(1).asProxy(),
        intake.set(IntakeState.kBump).withTimeout(1).asProxy(),

        // SHOOTER TESTS
        shooter.overrideHoodAngle(HoodConstants.kMaxHoodAngle).withTimeout(1.0).asProxy(),
        shooter.overrideHoodAngle(HoodConstants.kMinHoodAngle).withTimeout(1.0).asProxy(),
        shooter.turretAngle(Degrees.of(0.0)).withTimeout(1.0).asProxy(),
        shooter.turretAngle(Degrees.of(-90.0)).withTimeout(1.0).asProxy(),
        shooter.turretAngle(Degrees.of(90.0)).withTimeout(1.0).asProxy(),
        shooter.flywheelVelocity(RotationsPerSecond.of(30.0)).withTimeout(1.0).asProxy(),

        // TRANSFER TESTS
        transfer.set(TransferState.kTransferring).withTimeout(5).asProxy()).asProxy();
  }
}
