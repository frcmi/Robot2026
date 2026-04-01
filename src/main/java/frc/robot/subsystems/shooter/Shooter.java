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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.shooter.AimingConstants;
import frc.robot.constants.shooter.FieldConstants;
import frc.robot.constants.shooter.FlywheelConstants;
import frc.robot.constants.shooter.HoodConstants;
import frc.robot.constants.shooter.TurretConstants;
import frc.robot.lib.alliancecolor.AllianceUpdatedObserver;
import frc.robot.lib.command.CachedTrigger;
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
  @Getter @Setter private AngularVelocity flywheelOffset = RotationsPerSecond.of(0.0);

  // For detecting whether aimed or not
  double targetDist = 0.0;
  private Angle rawTurretTarget = ShooterState.kStowed.getTurret();
  private Angle rawHoodTarget = ShooterState.kStowed.getHood();

  // Triggers
  public Trigger nearTrench = new CachedTrigger(this::isNearTrench).debounce(0.05);
  public Trigger inAllianceZone = new CachedTrigger(this::isInAllianceZone).debounce(0.2);
  public Trigger aimed = new CachedTrigger(this::isAimed).debounce(0.2, DebounceType.kFalling);
  public Trigger turretOverride = new CachedTrigger(() -> isTurretOverride);

  // Vision IO
  private final VisionIO turretCamera;

  /** Creates a new Shooter. */
  public Shooter(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVel) {
    this(
        new AngularSubsystem(new AngularIO() {}, TurretConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, HoodConstants.kSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, FlywheelConstants.kSubsystemConfigReal),
        robotPose,
        robotVel,
        new VisionIO() {});
  }

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

  public void onAllianceFound(Alliance alliance) {
    this.alliance = alliance;
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
    Logger.recordOutput("Shooter/RawTurretTargetDeg", rawTurretTarget.in(Degrees));
    Logger.recordOutput("Shooter/NearTrench", nearTrench.getAsBoolean());
    Logger.recordOutput("Shooter/InAllianceZone", inAllianceZone.getAsBoolean());
    Logger.recordOutput("Shooter/Aimed", aimed.getAsBoolean());
    Logger.recordOutput("Shooter/FlywheelOffsetRPS", flywheelOffset.in(RotationsPerSecond));

    // Aim at hub
    Translation2d targetPosition;
    boolean allianceZone = inAllianceZone.getAsBoolean();
    if (allianceZone) {
      targetPosition =
          alliance == Alliance.Blue
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
      targetPosition = new Translation2d(targetX, targetY);
    }
    Logger.recordOutput("Shooter/Target", targetPosition);

    // Calculate turret angle to target
    Pose2d currentPose = this.robotPose.get();

    // rotate turret offsets by bot heading to convert to field-centric offsets
    Translation2d turretOffset =
        new Translation2d(TurretConstants.TurretOffset.getX(), TurretConstants.TurretOffset.getY())
            .rotateBy(currentPose.getRotation());

    // Calculate aiming position iteratively
    double dx = targetPosition.getX() - (currentPose.getX() + turretOffset.getX());
    double dy = targetPosition.getY() - (currentPose.getY() + turretOffset.getY());
    targetDist = Math.hypot(dx, dy);
    ChassisSpeeds robotVelocity = robotVel.get();
    // Found that it converges over 2 iterations, but do 5 to be safe
    for (int i = 0; i < 5; i++) {
      double airtime =
          (allianceZone ? AimingConstants.kAirtimeTable : AimingConstants.kAirtimeTableNeutral)
              .get(targetDist) * AimingConstants.kAirtimeMultiplier.get();
      dx =
          targetPosition.getX()
              - (currentPose.getX() + turretOffset.getX())
              - robotVelocity.vxMetersPerSecond * airtime;
      dy =
          targetPosition.getY()
              - (currentPose.getY() + turretOffset.getY())
              - robotVelocity.vyMetersPerSecond * airtime;
      targetDist = Math.hypot(dx, dy);
    }

    Logger.recordOutput("Shooter/DistanceToTargetM", targetDist);
    SmartDashboard.putNumber("DistanceToTargetM", targetDist);

    double angleToTarget = Math.atan2(dy, dx);
    Angle turretTarget =
        Radians.of(
            angleToTarget
                - currentPose.getRotation().getRadians()
                - TurretConstants.kTurretZero.in(Radians));

    // Wrap around to [-180, 180]
    turretTarget = AngleUtils.normalize(turretTarget);
    rawTurretTarget = turretTarget.copy();

    // Constrain to turret limits, with hysteresis
    double turretTargetVal = turretTarget.in(Radians);
    double[] possibleTurretAngles = {
      turretTargetVal, turretTargetVal + 2 * Math.PI, turretTargetVal - 2 * Math.PI
    };
    double optimizedTurretTarget = possibleTurretAngles[0];
    for (int i = 0; i < possibleTurretAngles.length; i++) {
      double dist = Math.abs(possibleTurretAngles[i] - measuredState.getTurret().in(Radians));
      if (dist < Math.abs(optimizedTurretTarget - measuredState.getTurret().in(Radians))
          && possibleTurretAngles[i] >= TurretConstants.kTurretPhysicalMinAngle.in(Radians)
          && possibleTurretAngles[i] <= TurretConstants.kTurretPhysicalMaxAngle.in(Radians)) {
        optimizedTurretTarget = possibleTurretAngles[i];
      }
    }
    turretTarget =
        Radians.of(
            MathUtil.clamp(
                optimizedTurretTarget,
                Degrees.of(AimingConstants.kTurretMinAngle.getAsDouble()).in(Radians),
                Degrees.of(AimingConstants.kTurretMaxAngle.getAsDouble()).in(Radians)));

    // Actually apply to hardware
    if (!isTurretOverride) {
      this.targetState.setTurret(turretTarget);
    } else {
      this.targetState.setTurret(measuredState.getTurret());
    }

    double hoodAngle =
        (allianceZone ? AimingConstants.kHoodAngleTable : AimingConstants.kHoodAngleTableNeutral)
            .get(targetDist);
    rawHoodTarget = Degrees.of(hoodAngle);

    // if we're near the trench or forcing the hood to be locked, hood goes to min angle
    this.targetState.setHood(
        ((nearTrench.getAsBoolean() && hoodUnlocked) || lockHood)
            ? HoodConstants.kMinHoodAngle
            : Degrees.of(hoodAngle));

    double flywheelRPS =
        (allianceZone
                ? AimingConstants.kFlywheelSpeedTable
                : AimingConstants.kFlywheelSpeedTableNeutral)
            .get(targetDist);
    this.targetState.setFlywheel(
        disabled ? RotationsPerSecond.of(0) : RotationsPerSecond.of(flywheelRPS));

    // Update turret camera
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

  public Command overrideHood(Voltage volts) {
    return this.hood.openLoop(() -> volts);
  }

  public Command overrideHoodAngle(Angle angle) {
    return this.hood.holdAtGoal(() -> angle);
  }

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

  public boolean isAimed() {
    double turretErr = rawTurretTarget.minus(measuredState.getTurret()).abs(Radians);
    double errorAtTarget = targetDist * Math.sin(turretErr);
    double hoodErr = rawHoodTarget.minus(measuredState.getHood()).abs(Degrees);
    double flywheelErr =
        measuredState.getFlywheel().minus(targetState.getFlywheel()).abs(RotationsPerSecond);
    double maxHoodErr = HoodConstants.kSubsystemConfigReal.getPositionTolerance().in(Degrees);
    boolean shooterDistInRange = targetDist > AimingConstants.kFlywheelSpeedTable.getMinKey();
    if (inAllianceZone.getAsBoolean()) {
      return errorAtTarget < (FieldConstants.hubWidth)
          && hoodErr < maxHoodErr
          && (flywheel.isAtAngle() || disabled)
          && shooterDistInRange;
    } else {
      // In neutral zone, more lenient since just tryna get into the alliance zone
      return errorAtTarget < (FieldConstants.bumpWidth * 2)
          && hoodErr < maxHoodErr * 1.8
          && ((flywheelErr
                  < FlywheelConstants.kSubsystemConfigReal
                          .getVelocityTolerance()
                          .in(RotationsPerSecond)
                      * 1.8)
              || disabled)
          && shooterDistInRange;
    }
  }

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

  public Command turretPower(Supplier<Voltage> volts) {
    return this.turret.openLoop(volts);
  }

  public Command turretAngle(Angle angle) {
    return this.turret.holdAtGoal(() -> angle);
  }

  public Command flywheelVelocity(AngularVelocity vel) {
    return this.flywheel.velocity(() -> vel);
  }

  private Voltage turretFeedforward() {
    double robotOmega = robotVel.get().omegaRadiansPerSecond;
    double ffV = -TurretConstants.kV * robotOmega;
    Logger.recordOutput("Shooter/TurretFF_V", ffV);
    return Volts.of(ffV);
  }

  public Command resetTurret() {
    return this.turret.resetAngle();
  }
}
