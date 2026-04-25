// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
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
import frc.robot.Constants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.intake.PivotConstants;
import frc.robot.constants.intake.RollerConstants;
import frc.robot.constants.shooter.FieldConstants;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.alliancecolor.AllianceUpdatedObserver;
import frc.robot.lib.command.CachedTrigger;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends VirtualSubsystem implements AllianceUpdatedObserver {
  private final AngularSubsystem rollers;
  private final AngularSubsystem pivot;
  private final Supplier<Pose2d> robotPose;
  private Alliance alliance = Alliance.Red;
  private final Trigger nearBump = new CachedTrigger(this ::isNearBump).debounce(0.05);
  private final BooleanSupplier shooting;
  private final BooleanSupplier manualOscillate;
  private final BooleanSupplier isAutonomous;

  private final Timer oscillationTimer = new Timer();

  private final LoggedTunableNumber shakeAngleDegrees =
      new LoggedTunableNumber("Intake/ShakeAngleDegrees", 25.0);
  private final LoggedTunableNumber shakeUpTimeSec =
      new LoggedTunableNumber("Intake/ShakeUpTimeSec", 0.4);
  private final LoggedTunableNumber shakeDownTimeSec =
      new LoggedTunableNumber("Intake/ShakeDownTimeSec", 0.4);
  private final LoggedTunableNumber foldVelocityDegreesPerSecond =
      new LoggedTunableNumber("Intake/FoldVelocityDegreesPerSecond", 10.0);
  private final LoggedTunableNumber foldMaxAngleDegrees =
      new LoggedTunableNumber("Intake/FoldMaxAngleDegrees", 80.0);

  @Getter private IntakeState targetState = IntakeState.kDown;
  @Getter private IntakeState measuredState;

  /** Creates a new Intake. */
  public Intake(
      Supplier<Pose2d> robotPose, BooleanSupplier shooting, BooleanSupplier manualOscillate) {
    this(
        new AngularSubsystem(new AngularIO() {}, RollerConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, PivotConstants.kSubsystemConfigReal),
        robotPose,
        shooting,
        manualOscillate,
        () -> false);
  }

  public Intake(
      AngularSubsystem rollers,
      AngularSubsystem pivot,
      Supplier<Pose2d> robotPoseSupplier,
      BooleanSupplier shooting,
      BooleanSupplier manualOscillate,
      BooleanSupplier isAutonomous) {
    this.rollers = rollers;
    this.pivot = pivot;
    this.robotPose = robotPoseSupplier;
    this.shooting = shooting;
    this.manualOscillate = manualOscillate;
    this.isAutonomous =
        isAutonomous; // Unused, but may be useful at some point so just leaving it in

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> gatedTarget().getPivot()));
    rollers.setDefaultCommand(rollers.openLoop(() -> gatedTarget().getRollers()));
    this.setDefaultCommand(this.set(IntakeState.kDown));

    measuredState = new IntakeState(pivot.getAngle(), targetState.getRollers());
  }

  private boolean prevOscillating = false;

  private IntakeState gatedTarget() {
    if (nearBump.getAsBoolean()) {
      prevOscillating = false;
      return IntakeState.kBump;
    }
    if ((shooting.getAsBoolean() || manualOscillate.getAsBoolean())
        && this.targetState == IntakeState.kDown) {
      if (!prevOscillating) {
        oscillationTimer.restart();
        prevOscillating = true;
      }

      double t = oscillationTimer.get();
      double t1 = shakeUpTimeSec.get();
      double t2 = t1 + shakeDownTimeSec.get();
      double pivotAngle;
      if (t < t1) {
        pivotAngle = shakeAngleDegrees.get();
      } else if (t < t2) {
        pivotAngle = 0.0;
      } else {
        pivotAngle =
            Math.min(foldVelocityDegreesPerSecond.get() * (t - t2), foldMaxAngleDegrees.get());
      }
      return new IntakeState(Degrees.of(pivotAngle), IntakeState.kTransferring.getRollers());
    } else {
      prevOscillating = false;
    }
    return targetState;
  }

  @Override
  public void periodic() {
    double start = 0.0;
    if (Constants.kEnableLoopTimingLogs) {
      start = Timer.getFPGATimestamp();
    }

    measuredState.setPivot(pivot.getAngle());
    measuredState.setRollers(targetState.getRollers());

    Logger.recordOutput("Intake/TargetState", targetState);
    Logger.recordOutput("Intake/MeasuredState", measuredState);

    if (Constants.kEnableLoopTimingLogs) {
      double end = Timer.getFPGATimestamp();
      Logger.recordOutput("Timing/IntakeMS", (end - start) * 1000);
    }
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

  public Command openLoopPivot(Voltage volts) {
    return this.pivot.openLoop(() -> volts);
  }

  private boolean isNearBump() {
    Pose2d currentPose = this.robotPose.get();
    Translation2d currentPos =
        currentPose
            .getTranslation()
            .plus(PivotConstants.IntakeOffset.rotateBy(currentPose.getRotation()));

    Translation2d hubPosition =
        alliance == Alliance.Blue
            ? FieldConstants.kHubPositionBlue
            : FieldConstants.kHubPositionRed;

    // Check X
    boolean nearX =
        Math.abs(currentPos.getX() - hubPosition.getX()) < (FieldConstants.trenchWidthX / 2.0);
    boolean nearY =
        (currentPos.getY() > FieldConstants.trenchWidthY
                && currentPos.getY() < (FieldConstants.bumpWidthY + FieldConstants.trenchWidthY))
            || (currentPos.getY()
                    > FieldConstants.trenchWidthY
                        + FieldConstants.bumpWidthY
                        + FieldConstants.hubWidthY
                && currentPos.getY() < (FieldConstants.fieldWidthY - FieldConstants.trenchWidthY));
    return nearX && nearY;
  }

  public void onAllianceFound(Alliance alliance) {
    this.alliance = alliance;
  }
}
