// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RobotSuperstructure;
import frc.robot.constants.Intake.PivotConstants;
import frc.robot.constants.Intake.RollerConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.Shooter.FlywheelConstants;
import frc.robot.constants.Shooter.HoodConstants;
import frc.robot.constants.Shooter.TurretConstants;
import frc.robot.constants.TunerConstantsAlpha;
import frc.robot.constants.VisionConstants;
import frc.robot.lib.alliancecolor.AllianceChecker;
import frc.robot.lib.controller.Joysticks;
import frc.robot.lib.sim.CurrentDrawCalculatorSim;
import frc.robot.lib.subsystem.angular.AngularIOSim;
import frc.robot.lib.subsystem.angular.AngularIOTalonFX;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  private final Joysticks controller = new Joysticks(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d field = new Field2d();

  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Intake intake;

  private final RobotSuperstructure superstructure;

  @SuppressWarnings("FieldCanBeLocal")
  private final AllianceChecker allianceChecker = new AllianceChecker();

  @SuppressWarnings("FieldCanBeLocal")
  private final CurrentDrawCalculatorSim currentDrawCalculatorSim = new CurrentDrawCalculatorSim();

  private final Alert autoAlert = new Alert("No auto selected!", Alert.AlertType.kWarning);
  private final Alert controllerOneAlert =
      new Alert("Controller 1 is unplugged!", Alert.AlertType.kWarning);

  @SuppressWarnings("FieldCanBeLocal")
  private final SuperstructureVisualizer measuredSuperstructureState;

  @SuppressWarnings("FieldCanBeLocal")
  private final SuperstructureVisualizer targetSuperstructureState;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstantsAlpha.FrontLeft),
                new ModuleIOTalonFX(TunerConstantsAlpha.FrontRight),
                new ModuleIOTalonFX(TunerConstantsAlpha.BackLeft),
                new ModuleIOTalonFX(TunerConstantsAlpha.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        intake =
            new Intake(
                new AngularSubsystem(
                    new AngularIOTalonFX(RollerConstants.kTalonFXConfig),
                    RollerConstants.kSubsystemConfigReal),
                new AngularSubsystem(
                    new AngularIOTalonFX(PivotConstants.kTalonFXConfig),
                    PivotConstants.kSubsystemConfigReal));
        shooter =
            new Shooter(
                new AngularSubsystem(
                    new AngularIOTalonFX(TurretConstants.kTalonFXConfig),
                    TurretConstants.kSubsystemConfigReal),
                new AngularSubsystem(
                    new AngularIOTalonFX(HoodConstants.kTalonFXConfig),
                    HoodConstants.kSubsystemConfigReal),
                new AngularSubsystem(
                    new AngularIOTalonFX(FlywheelConstants.kTalonFXConfig),
                    FlywheelConstants.kSubsystemConfigReal),
                drive::getPose);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstantsAlpha.FrontLeft, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstantsAlpha.FrontRight, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstantsAlpha.BackLeft, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstantsAlpha.BackRight, currentDrawCalculatorSim));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        AngularIOSim pivotIO =
            new AngularIOSim(PivotConstants.kSimConfig, currentDrawCalculatorSim);
        pivotIO.setRealAngleFromSubsystemAngleZeroSupplier(
            PivotConstants.kRealAngleFromSubsystemAngleZeroSupplier);
        intake =
            new Intake(
                new AngularSubsystem(
                    new AngularIOSim(RollerConstants.kSimConfig, currentDrawCalculatorSim),
                    RollerConstants.kSubsystemConfigSim),
                new AngularSubsystem(pivotIO, PivotConstants.kSubsystemConfigReal));

        shooter =
            new Shooter(
                new AngularSubsystem(
                    new AngularIOSim(TurretConstants.kSimConfig, currentDrawCalculatorSim),
                    TurretConstants.kSubsystemConfigSim),
                new AngularSubsystem(
                    new AngularIOSim(HoodConstants.kSimConfig, currentDrawCalculatorSim),
                    HoodConstants.kSubsystemConfigSim),
                new AngularSubsystem(
                    new AngularIOSim(FlywheelConstants.kSimConfig, currentDrawCalculatorSim),
                    FlywheelConstants.kSubsystemConfigSim),
                drive::getPose);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(drive::addVisionMeasurement, new VisionIO() {} /* , new VisionIO() {} */);
        intake = new Intake();
        shooter = new Shooter(drive::getPose);
        break;
    }

    superstructure = new RobotSuperstructure(intake);
    superstructure.registerAutoCommands();

    measuredSuperstructureState =
        new SuperstructureVisualizer(
            intake::getMeasuredState,
            shooter::getMeasuredState,
            drive::getPose,
            "Measured",
            RobotConstants.kMeasuredStateColor);
    targetSuperstructureState =
        new SuperstructureVisualizer(
            intake::getTargetState,
            shooter::getTargetState,
            drive::getPose,
            "Target",
            RobotConstants.kTargetStateColor);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    logInit();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftStickY(),
            () -> -controller.getLeftStickX(),
            () -> -controller.getRightStickX()));

    // Switch to X pattern when X button is pressed
    controller.buttonX.onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Intake controls
    controller.buttonA.onTrue(superstructure.intakeDeploy());
    controller.buttonA.onFalse(superstructure.intakeStowed());
  }

  private void logInit() {
    SmartDashboard.putData("Field", field);

    Logger.recordOutput(
        "Poses/AprilTagField", VisionConstants.kAprilTagField.values().toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Poses/WeldedAprilTagField",
        VisionConstants.kWeldedAprilTagField.values().toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Poses/AndyMarkAprilTagField",
        VisionConstants.kAndyMarkAprilTagField.values().toArray(new Pose3d[0]));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
