package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOTalonFXConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.subsystems.shooter.ShooterState;
import java.util.Optional;
import java.util.function.Supplier;

public class TurretConstants {
  public static final Translation3d TurretOffset =
      new Translation3d(
          Inches.of(-5.195f).in(Meters),
          Inches.of(4.995f).in(Meters),
          Inches.of(5.0f).in(Meters)); // Forward, left, up

  // Distance from camera lens to center of turret, 0, height of camera lens above ground
  public static final Translation3d TurretCameraOffset =
      new Translation3d(Inches.of(7.005).in(Meters), 0, Inches.of(19.010477).in(Meters));
  public static final Rotation3d TurretCameraRotation = new Rotation3d(0, Math.toRadians(32), 0);

  public static final Angle kTurretPhysicalMinAngle = Degrees.of(-180);
  public static final Angle kTurretPhysicalMaxAngle =
      Degrees.of(180); // Positive = CCW from top-down perspective
  public static final Angle kTurretZero =
      Degrees.of(
          90); // CCW from intake angle, e.g. 180 means that the zero of the turret is opposite to
  // intake direction

  public static final double kTurretGearRatio = 75.0 / 21.0 * 48.0 / 11.0;
  public static final double kV = kTurretGearRatio / DCMotor.getKrakenX44(1).KvRadPerSecPerVolt;

  public static final Voltage OVERRIDE_VOLTAGE = Volts.of(1.5);

  public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
      () -> Rotation2d.kZero;

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Turret")
          .bus(kRioBus)
          .positionTolerance(Degrees.of(10.0))
          .velocityTolerance(RotationsPerSecond.of(1.0)) // Robot can be spinning while we shoot
          .kP(10.0)
          .kI(0.0)
          .kD(0.4)
          .kV(TurretConstants.kV)
          .cruiseVelocity(RotationsPerSecond.of(5)) // 420rpm
          .acceleration(RotationsPerSecondPerSecond.of(15.0))
          .build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(22)
          .sensorId(Optional.of(42))
          .bus(kRioBus)
          .resetAngle(ShooterState.kStowed.getTurret())
          .softMinAngle(kTurretPhysicalMinAngle)
          .softMaxAngle(kTurretPhysicalMaxAngle)
          .motorRotationsPerOutputRotations(1) // 1:1 ratio between the two
          .rotorRotationsPerSensorRotation(kTurretGearRatio)
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .inverted(InvertedValue.CounterClockwise_Positive)
          .supplyCurrentLimit(Amps.of(40.0))
          .statorCurrentLimit(Amps.of(80.0))
          .neutralMode(NeutralModeValue.Brake)
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kV(kSubsystemConfigReal.getKV())
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey(kSubsystemConfigReal.getLogKey())
          .bus(kSubsystemConfigReal.getBus())
          .positionTolerance(kSubsystemConfigReal.getPositionTolerance())
          .velocityTolerance(kSubsystemConfigReal.getVelocityTolerance())
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kV(kSubsystemConfigReal.getKV())
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.09);

  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getKrakenX44(1))
          .moi(kMOI)
          .resetAngle(kTalonFXConfig.getResetAngle())
          .physicalMinAngle(kTurretPhysicalMinAngle)
          .physicalMaxAngle(kTurretPhysicalMaxAngle)
          .motorRotationsPerOutputRotations(
              kTalonFXConfig.getMotorRotationsPerOutputRotations()
                  * kTalonFXConfig.getRotorRotationsPerSensorRotation())
          .neutralMode(kTalonFXConfig.getNeutralMode())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .kV(kSubsystemConfigSim.getKV())
          .cruiseVelocity(kSubsystemConfigSim.getCruiseVelocity())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .supplyCurrentLimit(kTalonFXConfig.getSupplyCurrentLimit())
          .statorCurrentLimit(kTalonFXConfig.getStatorCurrentLimit())
          .build();
}
