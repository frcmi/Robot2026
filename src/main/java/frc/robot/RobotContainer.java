// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RobotSuperstructure;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.climb.ClimberConstants;
import frc.robot.constants.intake.PivotConstants;
import frc.robot.constants.intake.RollerConstants;
import frc.robot.constants.shooter.FlywheelConstants;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.constants.transfer.KickerConstants;
import frc.robot.constants.transfer.TransferConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.LoggedInterpolatingTableManager;
import frc.robot.lib.alliancecolor.AllianceChecker;
import frc.robot.lib.controller.Joysticks;
import frc.robot.lib.sim.CurrentDrawCalculatorSim;
import frc.robot.lib.subsystem.angular.AngularIOSim;
import frc.robot.lib.subsystem.angular.AngularIOTalonFX;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.subsystem.linear.LinearIOSim;
import frc.robot.lib.subsystem.linear.LinearIOTalonFX;
import frc.robot.lib.subsystem.linear.LinearSubsystem;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.led.CANdleSystem;
import frc.robot.subsystems.led.io.CANdleIOReal;
import frc.robot.subsystems.led.io.CANdleIOSim;
import frc.robot.subsystems.shooter.FuelSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
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
  private final Joysticks driverController = new Joysticks(0);
  private final Joysticks operatorController = new Joysticks(1);
  private final Joysticks sysIdcontroller = new Joysticks(4);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d field = new Field2d();

  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Intake intake;
  private final Transfer transfer;
  private final Climb climb;
  private final CANdleSystem led;

  private final RobotSuperstructure superstructure;

  @SuppressWarnings("FieldCanBeLocal")
  private final AllianceChecker allianceChecker = new AllianceChecker();

  @SuppressWarnings("FieldCanBeLocal")
  private final LoggedInterpolatingTableManager tableManager =
      new LoggedInterpolatingTableManager();

  @SuppressWarnings("FieldCanBeLocal")
  private final CurrentDrawCalculatorSim currentDrawCalculatorSim = new CurrentDrawCalculatorSim();

  private final Alert autoAlert = new Alert("No auto selected!", Alert.AlertType.kWarning);
  private final Alert controllerOneAlert =
      new Alert("Controller 1 is unplugged!", Alert.AlertType.kWarning);

  @SuppressWarnings("FieldCanBeLocal")
  private final SuperstructureVisualizer measuredSuperstructureState;

  @SuppressWarnings("FieldCanBeLocal")
  private final SuperstructureVisualizer targetSuperstructureState;

  @SuppressWarnings("FieldCanBeLocal")
  private final Optional<FuelSim> fuelSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.driveHardwareExists) {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
        } else {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
        }

        if (Constants.visionHardwareExists) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOLimelight(camera0Name, drive::getRotation),
                  new VisionIOLimelight(camera1Name, drive::getRotation),
                  new VisionIOLimelight(camera2Name, drive::getRotation),
                  new VisionIOLimelight(camera3Name, drive::getRotation));
        } else {
          vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        }

        if (Constants.shooterHardwareExists) {
          shooter =
              new Shooter(
                  new AngularSubsystem(
                      new AngularIOTalonFX(TurretConstants.kTalonFXConfig),
                      TurretConstants.kSubsystemConfigReal),
                  new AngularSubsystem(
                      new AngularIOTalonFX(HoodConstants.kTalonFXConfig),
                      HoodConstants.kSubsystemConfigReal),
                  /*new AngularSubsystem(
                  new AngularIOSim(HoodConstants.kSimConfig, currentDrawCalculatorSim),
                  HoodConstants.kSubsystemConfigSim),*/
                  new AngularSubsystem(
                      new AngularIOTalonFX(FlywheelConstants.kTalonFXConfig),
                      FlywheelConstants.kSubsystemConfigReal),
                  drive::getPose,
                  drive::getPoseVelocity);
        } else {
          shooter = new Shooter(drive::getPose, drive::getPoseVelocity);
        }

        if (Constants.intakeHardwareExists) {
          transfer =
              new Transfer(
                  new AngularSubsystem(
                      new AngularIOTalonFX(TransferConstants.kTalonFXConfig),
                      TransferConstants.kSubsystemConfigReal),
                  new AngularSubsystem(
                      new AngularIOTalonFX(KickerConstants.kTalonFXConfig),
                      KickerConstants.kSubsystemConfigReal),
                  shooter.aimed);
          intake =
              new Intake(
                  new AngularSubsystem(
                      new AngularIOTalonFX(RollerConstants.kTalonFXConfig),
                      RollerConstants.kSubsystemConfigReal),
                  new AngularSubsystem(
                      new AngularIOTalonFX(PivotConstants.kTalonFXConfig),
                      PivotConstants.kSubsystemConfigReal),
                  drive::getPose,
                  transfer::isAttemptingShooting);
        } else {
          transfer = new Transfer(shooter.aimed);
          intake = new Intake(drive::getPose, transfer::isAttemptingShooting);
        }

        if (Constants.climbHardwareExists) {
          climb =
              new Climb(
                  new LinearSubsystem(
                      new LinearIOTalonFX(ClimberConstants.kTalonFXConfig),
                      ClimberConstants.kSubsystemConfigReal));
        } else {
          climb = new Climb();
        }
        if (Constants.ledHardwareExists) {
          led = new CANdleSystem(new CANdleIOReal(), shooter.aimed);
        } else {
          led = new CANdleSystem();
        }
        fuelSim = Optional.empty();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstants.FrontRight, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstants.BackLeft, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstants.BackRight, currentDrawCalculatorSim));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

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
                drive::getPose,
                drive::getPoseVelocity);

        transfer =
            new Transfer(
                new AngularSubsystem(
                    new AngularIOSim(TransferConstants.kSimConfig, currentDrawCalculatorSim),
                    TransferConstants.kSubsystemConfigSim),
                new AngularSubsystem(
                    new AngularIOSim(KickerConstants.kSimConfig, currentDrawCalculatorSim),
                    KickerConstants.kSubsystemConfigSim),
                shooter.aimed);
        intake =
            new Intake(
                new AngularSubsystem(
                    new AngularIOSim(RollerConstants.kSimConfig, currentDrawCalculatorSim),
                    RollerConstants.kSubsystemConfigSim),
                new AngularSubsystem(
                    new AngularIOSim(PivotConstants.kSimConfig, currentDrawCalculatorSim),
                    PivotConstants.kSubsystemConfigSim),
                drive::getPose,
                transfer::isAttemptingShooting);

        climb =
            new Climb(
                new LinearSubsystem(
                    new LinearIOSim(ClimberConstants.kSimConfig, currentDrawCalculatorSim),
                    ClimberConstants.kSubsystemConfigSim));
        led = new CANdleSystem(new CANdleIOSim(), shooter.aimed);
        fuelSim =
            Optional.of(
                new FuelSim(
                    shooter::getMeasuredState,
                    transfer::getMeasuredState,
                    transfer::isShooting,
                    drive::getPose,
                    drive::getPoseVelocity));
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
            new Vision(drive::addVisionMeasurement, new VisionIO() {}); // , new VisionIO() {});
        shooter = new Shooter(drive::getPose, drive::getPoseVelocity);
        transfer = new Transfer(shooter.aimed);
        intake = new Intake(drive::getPose, transfer::isAttemptingShooting);
        climb = new Climb();
        led = new CANdleSystem();
        fuelSim =
            Optional.of(
                new FuelSim(
                    shooter::getMeasuredState,
                    transfer::getMeasuredState,
                    () -> (transfer.getMeasuredState().getKicker().baseUnitMagnitude() > 0),
                    drive::getPose,
                    drive::getPoseVelocity));
        break;
    }

    superstructure = new RobotSuperstructure(intake, transfer, climb, shooter);
    superstructure.registerAutoCommands();

    measuredSuperstructureState =
        new SuperstructureVisualizer(
            intake::getMeasuredState,
            shooter::getMeasuredState,
            climb::getMeasuredState,
            drive::getPose,
            "Measured",
            RobotConstants.kMeasuredStateColor);
    targetSuperstructureState =
        new SuperstructureVisualizer(
            intake::getTargetState,
            shooter::getTargetState,
            climb::getTargetState,
            drive::getPose,
            "Target",
            RobotConstants.kTargetStateColor);

    allianceChecker.registerObservers(shooter);

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
    boolean sim = Constants.currentMode == Constants.simMode;

    /* DRIVE COMMANDS
    - Left joystick: drive
    - Right joystick: turn
    - Hold X (real only): stop and move modules to X pattern to resist push (won't be able to drive while doing this)
    - Hold right bumper: Dynamically align heading & x position with trench, you just control forward/backward speed
     */
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftStickY() * superstructure.getDriveSpeed(false),
            () -> -driverController.getLeftStickX() * superstructure.getDriveSpeed(false),
            () -> -driverController.getRightStickX() * superstructure.getDriveSpeed(true)));
    if (!sim) {
      driverController.buttonX.whileTrue(Commands.runOnce(drive::stopWithX, drive));
    }
    // driverController
    //     .rightBumper
    //     .whileTrue(
    //         Commands.parallel(
    //             DriveCommands.joystickDriveThroughTrench(
    //                 drive,
    //                 () -> driverController.getLeftStickY() * superstructure.getDriveSpeed(false),
    //                 drive::getPose),
    //             superstructure.lockHoodDown()))
    //     .whileFalse(superstructure.unlockHood());

    // Switch to X pattern when X button is pressed
    // controller.buttonY.whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // // controller.buttonA.whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // controller.buttonB.whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // driverController.buttonA.onTrue(
    //     Commands.runOnce(() -> drive.setPose(new Pose2d(13.0, 0.88, new Rotation2d(0)))));

    // Intake controls
    operatorController.rightTrigger.whileTrue(
        // Commands.parallel(
        //     DriveCommands.joystickDrive(
        //         drive,
        //         () ->
        //             driverController.getLeftStickY()
        //                 * DriveConstants.MAX_SPEED_MULTIPLIER
        //                 * DriveConstants.TRANSFERRING_DRIVETRAIN_SPEED_MULTIPLIER,
        //         () ->
        //             -driverController.getLeftStickX()
        //                 * DriveConstants.MAX_SPEED_MULTIPLIER
        //                 * DriveConstants.TRANSFERRING_DRIVETRAIN_SPEED_MULTIPLIER,
        //         () ->
        //             -driverController.getRightStickX()
        //                 * DriveConstants.MAX_ROTATION_MULTIPLIER
        //                 * DriveConstants.TRANSFERRING_DRIVETRAIN_ROTATION_MULTIPLIER),
        transfer.set(TransferState.kTransferring)); // );
    operatorController
        .rightBumper
        .onTrue(transfer.set(TransferState.kReverse))
        .onFalse(transfer.set(TransferState.kIdle));

    operatorController.leftTrigger.whileTrue(
        // Commands.parallel(
        //     DriveCommands.joystickDrive(
        //         drive,
        //         () ->
        //             driverController.getLeftStickY()
        //                 * DriveConstants.MAX_SPEED_MULTIPLIER
        //                 * DriveConstants.INTAKING_DRIVETRAIN_SPEED_MULTIPLIER,
        //         () ->
        //             -driverController.getLeftStickX()
        //                 * DriveConstants.MAX_SPEED_MULTIPLIER
        //                 * DriveConstants.INTAKING_DRIVETRAIN_SPEED_MULTIPLIER,
        //         () ->
        //             -driverController.getRightStickX()
        //                 * DriveConstants.MAX_ROTATION_MULTIPLIER
        //                 * DriveConstants.INTAKING_DRIVETRAIN_ROTATION_MULTIPLIER),
        intake.set(IntakeState.kIntaking)); // );

    operatorController.dPadDown.onTrue(intake.zeroPivot());
    operatorController.leftBumper.whileTrue(intake.set(IntakeState.kReversing));
    operatorController.buttonY.whileTrue(intake.set(IntakeState.kInit));
    operatorController.buttonX.whileTrue(intake.set(IntakeState.kStowed));

    AtomicReference<IntakeState> osscilationStatus =
        new AtomicReference<IntakeState>(IntakeState.kIntaking);
    operatorController
        .buttonB
        .whileTrue(
            Commands.run(() -> osscilationStatus.set(IntakeState.kIntaking), intake)
                .andThen(
                    Commands.repeatingSequence(
                        intake.set(osscilationStatus::get),
                        Commands.waitSeconds(0.3),
                        Commands.run(
                            () -> {
                              IntakeState state = osscilationStatus.get();

                              if (state == IntakeState.kIntaking) {
                                osscilationStatus.set(IntakeState.kStowed);
                              } else {
                                osscilationStatus.set(IntakeState.kIntaking);
                              }
                            }))))
        .onFalse(intake.set(IntakeState.kIntaking));

    // operatorController.buttonX.whileTrue(climb.set(ClimbState.));
    operatorController.dPadUp.onTrue(shooter.toggleDisabled());

    // zero hood (in-match)
    // operatorController.dPadDown.whileTrue(shooter.overrideHood(-2.0)).onFalse(shooter.zeroHood());

    // update pathplanner (TEMP, DELETE LATER)
    // driverController.buttonB.onTrue(Commands.runOnce(() ->
    // drive.updatePathplannerPIDConstants()));

    // trench controls
    driverController.rightBumper.whileTrue(
        Commands.parallel(
            DriveCommands.joystickDriveThroughTrench(
                drive,
                () -> driverController.getLeftStickY() * superstructure.getDriveSpeed(false),
                drive::getPose))); // ,
    // superstructure.lockHoodDown()))
    // .whileFalse(superstructure.unlockHood());

    /* INTAKE CONTROLS
    - Driver left trigger: Intake
    - Driver left bumper: Reverse intake
     */
    driverController.leftTrigger.whileTrue(intake.set(IntakeState.kIntaking));
    driverController.leftBumper.whileTrue(intake.set(IntakeState.kReversing));

    driverController.buttonY.onTrue(intake.set(IntakeState.kInit));
    driverController.buttonX.onTrue(intake.set(IntakeState.kStowed));
    driverController.buttonA.whileTrue(intake.set(IntakeState.kOscillating));
    driverController.buttonA.onFalse(intake.set(IntakeState.kDown));
    driverController.buttonB.onTrue(shooter.toggleDisabled());

    /* SHOOTER CONTROLS
    - Operator right trigger (driver in sim): Shoot, NOTE: this rumbles the controller when not aimed
    - Operator right bumper: Reverse transfer (why is this useful?)
     */
    (sim ? driverController : operatorController)
        .rightTrigger.whileTrue(transfer.set(TransferState.kTransferring));
    operatorController
        .rightTrigger
        .and(shooter.aimed.negate())
        .onTrue(Commands.runOnce(() -> operatorController.rumble(RumbleType.kBothRumble, 1.0)))
        .onFalse(Commands.runOnce(() -> operatorController.rumble(RumbleType.kBothRumble, 0.0)));
    operatorController.rightBumper.whileTrue(transfer.set(TransferState.kReverse));

    /* CLIMBER CONTROLS
    - Operator Y (sim driver): Raise climb
    - Operator X (sim driver): Lower climb
     */
    (sim ? driverController : operatorController).buttonY.onTrue(superstructure.climbRaise());
    (sim ? driverController : operatorController).buttonX.onTrue(superstructure.climbClimbed());

    /* DEBUG/FAILSAFE CONTROLS:
     - Operator B (HOLD): Stow intake
     - Operator DPad right: Disable shooter
     - Operator DPad Left: Moves hood down when held, release to zero hood
     - Operator DPad Down: Moves climb down when held, release to zero climb
    */
    // TODO: lock turret control
    // operatorController.buttonB.whileTrue(intake.set(IntakeState.kInit));
    // operatorController.dPadRight.onTrue(shooter.toggleDisabled());
    operatorController
        .dPadLeft
        .whileTrue(shooter.overrideHood(HoodConstants.MANUAL_OVERRIDE))
        .onFalse(shooter.zeroHood());
    operatorController
        .dPadDown
        .whileTrue(climb.overrideClimb(ClimberConstants.MANUAL_OVERRIDE))
        .onFalse(climb.resetClimb());
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

    Logger.recordOutput("Drive/TrenchDrive/TrenchY", 0.0);
    Logger.recordOutput("Drive/TrenchDrive/YError", 0.0);
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
