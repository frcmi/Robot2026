// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.either;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.shooter.AimingConstants;
import frc.robot.constants.shooter.FieldConstants;
import frc.robot.constants.shooter.FlywheelConstants;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.lib.alliancecolor.AllianceUpdatedObserver;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.utils.AngleUtils;
import frc.robot.subsystems.vision.VisionIO;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Shooter extends VirtualSubsystem implements AllianceUpdatedObserver {
  // Constants for aiming tolerances
  private static final double NEUTRAL_ZONE_TOLERANCE_MULTIPLIER = 1.8;
  private static final double NEUTRAL_ZONE_ERROR_MULTIPLIER = 2.0;
  private static final double CONVERGENCE_THRESHOLD = 0.01; // meters
  private static final int MAX_ITERATIONS = 5;

  // Cached values for performance
  private final double turretZeroRad = TurretConstants.kTurretZero.in(Radians);
  private final double maxHoodErr =
      HoodConstants.kSubsystemConfigReal.getPositionTolerance().in(Degrees);

  private final AngularSubsystem turret;
  private final AngularSubsystem hood;
  private final AngularSubsystem flywheel;

  @Getter private ShooterState targetState = ShooterState.kStowed;
  @Getter private ShooterState measuredState;

  private Alliance alliance = Alliance.Red;

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotVel;

  private boolean disabled = false;
  private boolean isTurretOverride = false;
  private boolean lockHood = false;

  // for crossing shooting in init, if true hood will not lower when near trench
  @Getter @Setter private boolean hoodUnlocked = true;
  @Getter @Setter private AngularVelocity flywheelOffset = RotationsPerSecond.of(-1.0);

  // For detecting whether aimed or not
  double targetDist = 0.0;
  private Angle rawTurretTarget = ShooterState.kStowed.getTurret();
  private Angle rawHoodTarget = ShooterState.kStowed.getHood();

  // Triggers
  public Trigger nearTrench = new Trigger(this::isNearTrench).debounce(0.05);
  public Trigger inAllianceZone = new Trigger(this::isInAllianceZone).debounce(0.2);
  public Trigger aimed = new Trigger(this::isAimed).debounce(0.5, DebounceType.kFalling);
  public Trigger turretOverride = new Trigger(() -> isTurretOverride);

  // Vision IO
  private final VisionIO turretCamera;

  /**
   * Creates a new Shooter with default subsystems.
   *
   * @param robotPose Supplier for current robot pose
   * @param robotVel Supplier for current robot velocities
   */
  public Shooter(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVel) {
    this(
        new AngularSubsystem(new AngularIO() {}, TurretConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, HoodConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, FlywheelConstants.kSubsystemConfigReal),
        robotPose,
        robotVel,
        new VisionIO() {});
  }

  /**
   * Creates a new Shooter with specified subsystems.
   *
   * @param turret Turret angular subsystem
   * @param hood Hood angular subsystem
   * @param flywheel Flywheel angular subsystem
   * @param robotPoseSupplier Supplier for current robot pose
   * @param robotVelSupplier Supplier for current robot velocities
   * @param turretCamera Vision IO for turret camera
   */
  public Shooter(
      AngularSubsystem turret,
      AngularSubsystem hood,
      AngularSubsystem flywheel,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelSupplier,
      VisionIO turretCamera) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.robotPose = robotPoseSupplier;
    this.robotVel = robotVelSupplier;
    hood.setDefaultCommand(hood.holdAtGoal(() -> getTargetState().getHood()));
    turret.setDefaultCommand(
        turret.holdAtGoal(() -> getTargetState().getTurret(), this::turretFeedforward));
    flywheel.setDefaultCommand(
        either(
            flywheel.openLoop(Volts.of(0)).until(() -> !disabled),
            flywheel
                .velocity(() -> getTargetState().getFlywheel().plus(flywheelOffset))
                .until(() -> disabled),
            () -> disabled));
    measuredState = new ShooterState(turret.getAngle(), hood.getAngle(), flywheel.getVelocity());
    this.turretCamera = turretCamera;
  }

  /**
   * Called when alliance color is updated.
   *
   * @param alliance The new alliance color
   */
  public void onAllianceFound(Alliance alliance) {
    this.alliance = alliance;
  }

  /**
   * Calculates turret angle to target using iterative prediction. Accounts for robot motion and
   * projectile flight time.
   *
   * @return Normalized turret angle within physical limits
   */
  private void updateTargeting() {
    Translation2d targetPosition = calculateTargetPosition();
    Logger.recordOutput("Shooter/Target", targetPosition);

    // Calculate turret angle to target
    Pose2d currentPose = this.robotPose.get();
    if (currentPose == null) {
      Logger.recordOutput("Shooter/Error", "Null robot pose");
      return;
    }

    // rotate turret offsets by bot heading to convert to field-centric offsets
    Translation2d turretOffset =
        new Translation2d(TurretConstants.TurretOffset.getX(), TurretConstants.TurretOffset.getY())
            .rotateBy(currentPose.getRotation());

    // Calculate aiming position iteratively with convergence check
    double dx = targetPosition.getX() - (currentPose.getX() + turretOffset.getX());
    double dy = targetPosition.getY() - (currentPose.getY() + turretOffset.getY());
    targetDist = Math.hypot(dx, dy);
    ChassisSpeeds robotVelocity = robotVel.get();

    int iterations = 0;
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double prevTargetDist = targetDist;
      double airtime =
          (inAllianceZone.getAsBoolean()
                  ? AimingConstants.kAirtimeTable
                  : AimingConstants.kAirtimeTableNeutral)
              .get(targetDist);
      dx =
          targetPosition.getX()
              - (currentPose.getX() + turretOffset.getX())
              - robotVelocity.vxMetersPerSecond * airtime;
      dy =
          targetPosition.getY()
              - (currentPose.getY() + turretOffset.getY())
              - robotVelocity.vyMetersPerSecond * airtime;
      targetDist = Math.hypot(dx, dy);
      iterations = i + 1;

      // Check for convergence
      if (Math.abs(targetDist - prevTargetDist) < CONVERGENCE_THRESHOLD) {
        break;
      }
    }
    Logger.recordOutput("Shooter/Aiming/ConvergenceIterations", iterations);

    Logger.recordOutput("Shooter/DistanceToTargetM", targetDist);

    double angleToTarget = Math.atan2(dy, dx);
    Angle turretTarget =
        Radians.of(angleToTarget - currentPose.getRotation().getRadians() - turretZeroRad);

    // Wrap around to [-180, 180]
    turretTarget = AngleUtils.normalize(turretTarget);
    rawTurretTarget = turretTarget.copy();

    // Constrain to turret limits
    turretTarget =
        Radians.of(
            MathUtil.clamp(
                turretTarget.in(Radians),
                Degrees.of(AimingConstants.kTurretMinAngle.getAsDouble()).in(Radians),
                Degrees.of(AimingConstants.kTurretMaxAngle.getAsDouble()).in(Radians)));

    // Actually apply to hardware
    if (!isTurretOverride) {
      this.targetState.setTurret(turretTarget);
    } else {
      this.targetState.setTurret(measuredState.getTurret());
    }

    double hoodAngle =
        (inAllianceZone.getAsBoolean()
                ? AimingConstants.kHoodAngleTable
                : AimingConstants.kHoodAngleTableNeutral)
            .get(targetDist);
    rawHoodTarget = Degrees.of(hoodAngle);

    // if we're near the trench or forcing the hood to be locked, hood goes to min angle
    this.targetState.setHood(
        ((nearTrench.getAsBoolean() && hoodUnlocked) || lockHood)
            ? HoodConstants.kMinHoodAngle
            : Degrees.of(hoodAngle));

    double flywheelRPS =
        (inAllianceZone.getAsBoolean()
                ? AimingConstants.kFlywheelSpeedTable
                : AimingConstants.kFlywheelSpeedTableNeutral)
            .get(targetDist);
    this.targetState.setFlywheel(
        disabled ? RotationsPerSecond.of(0) : RotationsPerSecond.of(flywheelRPS));
  }

  /**
   * Calculates the target position based on alliance and zone
   *
   * @return Target position for shooting
   */
  private Translation2d calculateTargetPosition() {
    boolean allianceZone = inAllianceZone.getAsBoolean();
    if (allianceZone) {
      return alliance == Alliance.Blue
          ? FieldConstants.kHubPositionBlue
          : FieldConstants.kHubPositionRed;
    } else {
      double targetX =
          alliance == Alliance.Blue
              ? FieldConstants.allianceZoneXBlue
              : FieldConstants.allianceZoneXRed;
      double targetY =
          robotPose.get().getY() < FieldConstants.kHubPositionBlue.getY()
              ? FieldConstants.allianceZoneYBottom
              : FieldConstants.allianceZoneYTop;
      return new Translation2d(targetX, targetY);
    }
  }

  /** Updates the turret camera position based on current turret angle */
  private void updateTurretCamera() {
    double turretCamYaw = TurretConstants.kTurretZero.plus(turret.getAngle()).in(Radians);
    Translation3d robotToCamera =
        TurretConstants.TurretOffset.plus(
            TurretConstants.TurretCameraOffset.rotateAround(
                new Translation3d(0, 0, 1), new Rotation3d(0, 0, turretCamYaw)));
    turretCamera.setRobotOffset(
        new Transform3d(
            robotToCamera,
            new Rotation3d(
                TurretConstants.TurretCameraRotation.getX(),
                TurretConstants.TurretCameraRotation.getY(),
                TurretConstants.TurretCameraRotation.getZ() + turretCamYaw)));
    Logger.recordOutput("Turret/CameraOffset", robotToCamera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setTurret(turret.getAngle());
    measuredState.setHood(hood.getAngle());
    measuredState.setFlywheel(flywheel.getVelocity().minus(flywheelOffset));

    Logger.recordOutput("Shooter/TargetState", targetState);
    Logger.recordOutput("Shooter/MeasuredState", measuredState);
    Logger.recordOutput("Shooter/Disabled", disabled);
    Logger.recordOutput("Shooter/TurretOverride", isTurretOverride);
    Logger.recordOutput("Shooter/NearTrench", nearTrench.getAsBoolean());
    Logger.recordOutput("Shooter/InAllianceZone", inAllianceZone.getAsBoolean());
    Logger.recordOutput("Shooter/Aimed", aimed.getAsBoolean());
    Logger.recordOutput("Shooter/FlywheelOffsetRPS", flywheelOffset.in(RotationsPerSecond));

    // Update targeting calculations
    updateTargeting();

    // Update turret camera position
    updateTurretCamera();
  }

  /**
   * Overrides hood with specified voltage.
   *
   * @param volts Voltage to apply to hood motor
   * @return Command that applies the voltage
   */
  public Command overrideHood(Voltage volts) {
    if (volts == null) {
      Logger.recordOutput("Shooter/Error", "Null voltage in overrideHood");
      return Commands.none();
    }
    return this.hood.openLoop(() -> volts);
  }

  /**
   * Overrides hood to hold at specified angle.
   *
   * @param angle Target angle for hood
   * @return Command that holds the hood at the angle
   */
  public Command overrideHoodAngle(Angle angle) {
    if (angle == null) {
      Logger.recordOutput("Shooter/Error", "Null angle in overrideHoodAngle");
      return Commands.none();
    }
    return this.hood.holdAtGoal(() -> angle);
  }

  /**
   * Resets hood angle to zero.
   *
   * @return Command that resets the hood angle
   */
  public Command zeroHood() {
    return this.hood.resetAngle();
  }

  public Command forceToggleState() {
    return runOnce(
        () -> {
          disabled = false;
          isTurretOverride = false;
        });
  }

  /**
   * Toggles the disabled state of the shooter.
   *
   * @return Command that toggles between enabled/disabled
   */
  public Command toggleDisabled() {
    return either(
        runOnce(
            () -> {
              disabled = false;
            }),
        runOnce(
            () -> {
              disabled = true;
            }),
        () -> disabled);
  }

  /**
   * Sets the hood lock state.
   *
   * @param lock True to lock hood, false to unlock
   * @return Command that sets the hood lock state
   */
  public Command setHoodLock(boolean lock) {
    return runOnce(
        () -> {
          lockHood = lock;
        });
  }

  private boolean getNearTrenchFromHub(Translation2d hubPosition, Pose2d currentPose) {
    // Check X
    boolean nearX =
        Math.abs(currentPose.getX() - hubPosition.getX()) < (FieldConstants.trenchWidthX / 2.0);
    boolean nearY =
        currentPose.getY() < FieldConstants.trenchWidthY
            || currentPose.getY() > (FieldConstants.fieldWidthY - FieldConstants.trenchWidthY);
    return nearX && nearY;
  }

  // Trench code
  private boolean isNearTrench() {
    ChassisSpeeds velDelta = robotVel.get().times(HoodConstants.HOOD_LOWER_TIME.in(Seconds));
    Pose2d currentPose = this.robotPose.get();
    Pose2d futurePose =
        currentPose.plus(
            new Transform2d(
                velDelta.vxMetersPerSecond, velDelta.vyMetersPerSecond, new Rotation2d()));
    return getNearTrenchFromHub(FieldConstants.kHubPositionBlue, currentPose)
        || getNearTrenchFromHub(FieldConstants.kHubPositionRed, currentPose)
        || getNearTrenchFromHub(FieldConstants.kHubPositionBlue, futurePose)
        || getNearTrenchFromHub(FieldConstants.kHubPositionRed, futurePose);
  }

  private boolean isInAllianceZone() {
    Pose2d currentPose = this.robotPose.get();
    if (alliance == Alliance.Blue) {
      return currentPose.getX() < FieldConstants.kHubPositionBlue.getX();
    } else {
      return currentPose.getX() > FieldConstants.kHubPositionRed.getX();
    }
  }

  /**
   * Checks if the shooter is aimed at the target within tolerance
   *
   * @return True if shooter is aimed and ready to fire
   */
  private boolean isAimed() {
    Logger.recordOutput("Shooter/RawTurretTargetDeg", rawTurretTarget.in(Degrees));
    double turretErr = rawTurretTarget.minus(measuredState.getTurret()).abs(Radians);

    // Clamp turret error to prevent NaN from sin() with large angles
    double clampedTurretErr = MathUtil.clamp(turretErr, -Math.PI, Math.PI);
    double errorAtTarget = targetDist * Math.sin(clampedTurretErr);

    double hoodErr = rawHoodTarget.minus(measuredState.getHood()).abs(Degrees);
    double flywheelErr =
        measuredState.getFlywheel().minus(targetState.getFlywheel()).abs(RotationsPerSecond);

    // Use cached maxHoodErr instead of recalculating
    boolean shooterDistInRange = targetDist > AimingConstants.kFlywheelSpeedTable.getMinKey();

    // Enhanced logging for debugging
    Logger.recordOutput("Shooter/Aiming/TurretErrorDeg", Math.toDegrees(turretErr));
    Logger.recordOutput("Shooter/Aiming/ErrorAtTargetM", errorAtTarget);
    Logger.recordOutput("Shooter/Aiming/HoodErrorDeg", hoodErr);
    Logger.recordOutput("Shooter/Aiming/FlywheelErrorRPS", flywheelErr);

    if (inAllianceZone.getAsBoolean()) {
      return errorAtTarget < (FieldConstants.hubWidth)
          && hoodErr < maxHoodErr
          && flywheel.isAtAngle()
          && shooterDistInRange;
    } else {
      // In neutral zone, more lenient since just tryna get into the alliance zone
      return errorAtTarget < (FieldConstants.bumpWidth * NEUTRAL_ZONE_ERROR_MULTIPLIER)
          && hoodErr < (maxHoodErr * NEUTRAL_ZONE_TOLERANCE_MULTIPLIER)
          && flywheelErr
              < FlywheelConstants.kSubsystemConfigReal.getVelocityTolerance().in(RotationsPerSecond)
                  * NEUTRAL_ZONE_TOLERANCE_MULTIPLIER
          && shooterDistInRange;
    }
  }

  /**
   * Toggles turret override state. When overridden, turret maintains current position instead of
   * auto-aiming.
   *
   * @return Command that toggles turret override
   */
  public Command toggleOverride() {
    return either(
        runOnce(
            () -> {
              isTurretOverride = false;
            }),
        runOnce(
            () -> {
              isTurretOverride = true;
            }),
        () -> isTurretOverride);
  }

  /**
   * Applies open-loop voltage control to turret.
   *
   * @param volts Supplier for voltage to apply
   * @return Command that controls turret with voltage
   */
  public Command turretPower(Supplier<Voltage> volts) {
    if (volts == null) {
      Logger.recordOutput("Shooter/Error", "Null voltage supplier in turretPower");
      return Commands.none();
    }
    return this.turret.openLoop(volts);
  }

  /**
   * Sets turret to hold at specified angle.
   *
   * @param angle Target angle for turret
   * @return Command that holds turret at the angle
   */
  public Command turretAngle(Angle angle) {
    if (angle == null) {
      Logger.recordOutput("Shooter/Error", "Null angle in turretAngle");
      return Commands.none();
    }
    return this.turret.holdAtGoal(() -> angle);
  }

  /**
   * Sets flywheel to maintain specified velocity.
   *
   * @param vel Target angular velocity for flywheel
   * @return Command that controls flywheel velocity
   */
  public Command flywheelVelocity(AngularVelocity vel) {
    if (vel == null) {
      Logger.recordOutput("Shooter/Error", "Null velocity in flywheelVelocity");
      return Commands.none();
    }
    return this.flywheel.velocity(() -> vel);
  }

  /**
   * Calculates feedforward voltage for turret to compensate for robot rotation.
   *
   * @return Feedforward voltage to apply
   */
  private Voltage turretFeedforward() {
    ChassisSpeeds robotVelocity = robotVel.get();
    if (robotVelocity == null) {
      Logger.recordOutput("Shooter/Error", "Null robot velocity in turretFeedforward");
      return Volts.of(0.0);
    }

    double robotOmega = robotVelocity.omegaRadiansPerSecond;
    double ffV = -TurretConstants.kV * robotOmega;
    Logger.recordOutput("Shooter/TurretFF_V", ffV);
    return Volts.of(ffV);
  }
}
