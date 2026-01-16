package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.Intake.PivotConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.subsystems.intake.IntakeState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer extends VirtualSubsystem {
  private final double PADDING = 22.0; // inches

  private final LoggedMechanism2d mechanismIntake = new LoggedMechanism2d(
      Inches.of(PADDING + 27 + PADDING).in(Meters), // Width: 27in
      Inches.of(95.0).in(Meters),
      new Color8Bit(34, 34, 34));
  private final LoggedMechanismRoot2d pivotRoot;
  private final LoggedMechanismLigament2d pivot;

  private final Supplier<IntakeState> intakeState;
  private final Supplier<Pose2d> robotPose;

  private final String logKey;

  public SuperstructureVisualizer(
      Supplier<IntakeState> intakeState, Supplier<Pose2d> robotPose, String logKey, Color8Bit mechColor) {
    this.intakeState = intakeState;
    this.robotPose = robotPose;
    this.logKey = logKey;

    pivotRoot = mechanismIntake.getRoot(
        "PivotRoot",
        Inches.of(PADDING + 23 - 1.536513).in(Meters), // Relative to sideplate
        Inches.of(0.75 + 12.504)
            .in(Meters)); // 0.75 is ground clearance, 12.504 is above origin (which has ground
    // clearance)

    // Slapdown intake visualization
    pivot = pivotRoot.append(
        new LoggedMechanismLigament2d(
            "Pivot",
            PivotConstants.PivotLength.in(Meters),
            intakeState.get().getPivot().in(Degrees),
            8.0,
            mechColor));
  }

  @Override
  public void periodic() {
    pivot.setAngle(intakeState.get().getPivot().in(Degrees));
    Logger.recordOutput(String.format("Superstructure/%sIntake", logKey), mechanismIntake);
    Logger.recordOutput(String.format("Superstructure/%sTurretPose", logKey),
        new Pose3d(robotPose.get()).plus(new Transform3d(
            new Translation3d(Inches.of(6.0).in(Meters), Inches.of(6.0).in(Meters), Inches.of(12.0).in(Meters)),
            new Rotation3d(0.0, 0.0, -robotPose.get().getRotation().getRadians()))));
    Logger.recordOutput(String.format("Superstructure/%sTurret", logKey), mechanismIntake);
  }
}
