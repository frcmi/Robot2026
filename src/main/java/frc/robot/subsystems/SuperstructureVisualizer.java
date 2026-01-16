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
import frc.robot.constants.Intake.PivotConstants;
import frc.robot.constants.Shooter.HoodConstants;
import frc.robot.constants.Shooter.TurretConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
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

  private final Supplier<IntakeState> intakeState;
  private final Supplier<ShooterState> shooterState;
  private final Supplier<Pose2d> robotPose;

  private final String logKey;

  public SuperstructureVisualizer(
      Supplier<IntakeState> intakeState,
      Supplier<ShooterState> shooterState,
      Supplier<Pose2d> robotPose,
      String logKey,
      Color8Bit mechColor) {
    this.intakeState = intakeState;
    this.shooterState = shooterState;
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
            "HoodRoot", Inches.of(PADDING).in(Meters), Inches.of(2.0).in(Meters));
    hood =
        hoodRoot.append(
            new LoggedMechanismLigament2d(
                "Hood",
                HoodConstants.HoodLength.in(Meters),
                shooterState.get().getHood().in(Degrees),
                3.0,
                mechColor));
  }

  @Override
  public void periodic() {
    intakePivot.setAngle(intakeState.get().getPivot().in(Degrees));
    hood.setAngle(90 - shooterState.get().getHood().in(Degrees));

    Logger.recordOutput(String.format("Superstructure/%sIntake", logKey), mechanismIntake);
    Logger.recordOutput(
        String.format("Superstructure/%sTurretPose", logKey),
        new Pose3d(robotPose.get())
            .plus(
                new Transform3d(
                    TurretConstants.TurretOffset,
                    new Rotation3d(
                        0.0,
                        0.0,
                        shooterState.get().getTurret().in(Radians) + Math.toRadians(180.0)))));
    Logger.recordOutput(String.format("Superstructure/%sShooter", logKey), mechanismShooter);
  }
}
