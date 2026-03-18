package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.RobotConstants.kCanivoreBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
          .bus(kCanivoreBus)
          .velocityTolerance(RotationsPerSecond.of(6))
          .acceleration(RotationsPerSecondPerSecond.of(200.0)) // 12000rpm/s, 0.5s spinup
          .kP(0.04)
          .kI(0.0)
          .kD(0.0)
          .kV(12.0 / (6000 * (2 * Math.PI / 60.0))) // 12V per 6000rpm
          .build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(40)
          .followerIds(java.util.List.of(49, 50))
          .bus(kCanivoreBus)
          .opposeMaster(true)
          .motorRotationsPerOutputRotations(1)
          .inverted(InvertedValue.CounterClockwise_Positive)
          .supplyCurrentLimit(Amps.of(40.0))
          .supplyCurrentLowerTime(Seconds.of(2.0))
          .supplyCurrentLower(Amps.of(15.0))
          .statorCurrentLimit(Amps.of(120.0))
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kV(kSubsystemConfigReal.getKV())
          .neutralMode(NeutralModeValue.Coast)
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey(kSubsystemConfigReal.getLogKey())
          .bus(kSubsystemConfigReal.getBus())
          .velocityTolerance(kSubsystemConfigReal.getVelocityTolerance())
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kV(kSubsystemConfigReal.getKV())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();
  public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0048975432);
  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getKrakenX60(3))
          .numMotors(3)
          .moi(kMOI)
          .motorRotationsPerOutputRotations(kTalonFXConfig.getMotorRotationsPerOutputRotations())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .kV(kSubsystemConfigSim.getKV())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .supplyCurrentLimit(kTalonFXConfig.getSupplyCurrentLimit())
          .statorCurrentLimit(kTalonFXConfig.getStatorCurrentLimit())
          .build();
}
