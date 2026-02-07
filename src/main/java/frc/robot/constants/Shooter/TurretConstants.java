package frc.robot.constants.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOTalonFXConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.subsystems.shooter.ShooterState;
import java.util.Optional;
import java.util.function.Supplier;

public class TurretConstants {
  public static final Translation3d TurretOffset =
      new Translation3d(
          Inches.of(-10.0f).in(Meters), Inches.of(10.0f).in(Meters), Inches.of(5.0f).in(Meters));

  public static final Angle kTurretMinAngle = Degrees.of(-90.0f);
  public static final Angle kTurretMaxAngle = Degrees.of(90.0f);

  public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
      () -> Rotation2d.kZero;

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Turret")
          .bus(kRioBus)
          .positionTolerance(Degrees.of(1.0))
          .velocityTolerance(RotationsPerSecond.of(1.0)) // Robot can be spinning while we shoot
          .kP(0.3)
          .kI(0.0)
          .kD(0.0)
          .cruiseVelocity(RotationsPerSecond.of(0.2)) // 420rpm
          .acceleration(DegreesPerSecondPerSecond.of(4000.0))
          .build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(22)
          .sensorId(Optional.of(0))
          .bus(kRioBus)
          .resetAngle(ShooterState.kStowed.getTurret())
          .softMinAngle(kTurretMinAngle)
          .softMaxAngle(kTurretMaxAngle)
          .motorRotationsPerOutputRotations(1.388) // Assuming 1:1 ratio for turret yaw to cancoder
          .rotorRotationsPerSensorRotation(3.14) // TODO: Figure out gearing
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .inverted(InvertedValue.CounterClockwise_Positive)
          .supplyCurrentLimit(Amps.of(40.0))
          .statorCurrentLimit(Amps.of(60.0))
          .neutralMode(NeutralModeValue.Brake)
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
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
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final MomentOfInertia kMOI =
      KilogramSquareMeters.of(0.0951078874); // TODO: Figure out

  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getKrakenX44(1))
          .moi(kMOI)
          .resetAngle(kTalonFXConfig.getResetAngle())
          .physicalMinAngle(kTurretMinAngle)
          .physicalMaxAngle(kTurretMaxAngle)
          .motorRotationsPerOutputRotations(
              kTalonFXConfig.getMotorRotationsPerOutputRotations()
                  * kTalonFXConfig.getRotorRotationsPerSensorRotation())
          .neutralMode(kTalonFXConfig.getNeutralMode())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .cruiseVelocity(kSubsystemConfigSim.getCruiseVelocity())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .build();
}
