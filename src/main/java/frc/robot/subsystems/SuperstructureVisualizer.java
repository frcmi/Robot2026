package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.climb.ClimberConstants;
import frc.robot.constants.intake.PivotConstants;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.shooter.ShooterState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer extends VirtualSubsystem {
  private final double PADDING = 22.0; // inches

  private final LoggedMechanism2d mechanismIntake =
      new LoggedMechanism2d(
          Inches.of(PADDING + 27 + PADDING).in(Meters), // Width: 27in
          Inches.of(95.0).in(Meters));
  private final LoggedMechanismRoot2d intakePivotRoot;
  private final LoggedMechanismLigament2d intakePivot;

  private final LoggedMechanism2d mechanismShooter =
      new LoggedMechanism2d(Inches.of(PADDING + 7 + PADDING).in(Meters), Inches.of(22).in(Meters));
  private final LoggedMechanismRoot2d hoodRoot;
  private final LoggedMechanismLigament2d hood;

  // Climb mechanism on YZ plane
  private final LoggedMechanism2d mechanismClimb =
      new LoggedMechanism2d(
          Inches.of(PADDING + 27 + PADDING).in(Meters),
          Inches.of(95.0).in(Meters));
  private final LoggedMechanismRoot2d climberRoot;
  private final LoggedMechanismLigament2d climberBase;
  private final LoggedMechanismLigament2d climberExtension;

  private final Supplier<IntakeState> intakeState;
  private final Supplier<ShooterState> shooterState;
  private final Supplier<ClimbState> climbState;
  private final Supplier<Pose2d> robotPose;

  private final String logKey;

  public SuperstructureVisualizer(
      Supplier<IntakeState> intakeState,
      Supplier<ShooterState> shooterState,
      Supplier<ClimbState> climbState,
      Supplier<Pose2d> robotPose,
      String logKey,
      Color8Bit mechColor) {
    this.intakeState = intakeState;
    this.shooterState = shooterState;
    this.climbState = climbState;
    this.robotPose = robotPose;
    this.logKey = logKey;

    intakePivotRoot =
        mechanismIntake.getRoot(
            "PivotRoot",
            Inches.of(PADDING + 23 - 1.536513).in(Meters), // Relative to sideplate
            Inches.of(0.75 + 6.0)
                .in(Meters)); // 0.75 is ground clearance, 12.504 is above origin (which has ground
    // clearance)

    // Slapdown intake visualization
    intakePivot =
        intakePivotRoot.append(
            new LoggedMechanismLigament2d(
                "Pivot",
                PivotConstants.PivotLength.in(Meters),
                intakeState.get().getPivot().in(Degrees),
                4.0,
                mechColor));

    // Hood
    hoodRoot =
        mechanismShooter.getRoot(
            "HoodRoot",
            Inches.of(PADDING + 6.0).in(Meters),
            Inches.of(2.0).in(Meters)); // Turret 6" from edge of hood
    hood =
        hoodRoot.append(
            new LoggedMechanismLigament2d(
                "Hood",
                HoodConstants.HoodLength.in(Meters),
                shooterState.get().getHood().in(Degrees),
                3.0,
                mechColor));

    // Climber - YZ plane visualization (separate from intake XZ plane)
    climberRoot =
        mechanismClimb.getRoot(
            "ClimberRoot",
            Inches.of(PADDING + ClimberConstants.kClimberWidth.in(Inches) / 2.0).in(Meters),
            ClimberConstants.kClimberBaseHeight.in(Meters));

    // Fixed base of climber (vertical, not moving)
    climberBase =
        climberRoot.append(
            new LoggedMechanismLigament2d(
                "ClimberBase",
                ClimberConstants.kClimberBaseHeight.in(Meters),
                -90.0, // Pointing down into drivetrain
                8.0,
                new Color8Bit(100, 100, 100)));

    // Extending part of climber
    climberExtension =
        climberRoot.append(
            new LoggedMechanismLigament2d(
                "ClimberExtension",
                climbState.get().getClimber().in(Meters),
                90.0, // Vertical extension
                4.0,
                mechColor));
  }

  @Override
  public void periodic() {
    intakePivot.setAngle(intakeState.get().getPivot().in(Degrees));
    hood.setAngle(90 + shooterState.get().getHood().in(Degrees));
    climberExtension.setLength(climbState.get().getClimber().in(Meters));

    Logger.recordOutput(String.format("Superstructure/%sIntake", logKey), mechanismIntake);
    Logger.recordOutput(
        String.format("Superstructure/%sTurretPose", logKey),
        new Pose3d(robotPose.get())
            .plus(
                new Transform3d(
                    TurretConstants.TurretOffset,
                    new Rotation3d(0.0, 0.0, shooterState.get().getTurret().in(Radians)))));
    Logger.recordOutput(String.format("Superstructure/%sShooter", logKey), mechanismShooter);
    Logger.recordOutput(String.format("Superstructure/%sClimb", logKey), mechanismClimb);
  }
}
