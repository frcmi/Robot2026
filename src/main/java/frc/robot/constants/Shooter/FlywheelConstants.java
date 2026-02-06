package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOTalonFXConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import java.util.function.Supplier;

public class FlywheelConstants {
  public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
      () -> Rotation2d.kZero;

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Flywheel")
          .velocityTolerance(RotationsPerSecond.of(0.333)) // 20rpm
          .acceleration(RotationsPerSecondPerSecond.of(200.0)) // 12000rpm/s, 0.5s spinup
          .kP(0.08)
          .kI(0.0)
          .kD(0.0)
          .kV(12.0 / (6000 * (2 * Math.PI / 60.0))) // 12V per 6000rpm
          .bus(kRioBus)
          .build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(14)
          .followerId(15)
          .opposeMaster(true)
          .motorRotationsPerOutputRotations(1)
          .bus(kRioBus)
          .inverted(InvertedValue.CounterClockwise_Positive)
          .supplyCurrentLimit(Amps.of(30.0))
          .statorCurrentLimit(Amps.of(60.0))
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kV(kSubsystemConfigReal.getKV())
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey(kSubsystemConfigReal.getLogKey())
          .bus(kSubsystemConfigReal.getBus())
          .velocityTolerance(kSubsystemConfigReal.getVelocityTolerance())
          .kP(0.05)
          .kI(0.0)
          .kD(0.0)
          .kV(kSubsystemConfigReal.getKV())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();
  public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0048975432);
  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getKrakenX60(2))
          .moi(kMOI)
          .motorRotationsPerOutputRotations(kTalonFXConfig.getMotorRotationsPerOutputRotations())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .kV(kSubsystemConfigSim.getKV())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .build();
}
