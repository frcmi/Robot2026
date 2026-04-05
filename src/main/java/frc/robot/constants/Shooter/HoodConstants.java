package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOTalonFXConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import java.util.function.Supplier;

public class HoodConstants {
  public static final Distance HoodLength = Inches.of(9.0);

  public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
      () -> Rotation2d.kZero;

  public static final Angle kMinHoodAngle = Degrees.of(25.2);
  public static final Angle kMaxHoodAngle = Degrees.of(43.2);
  public static final Voltage MANUAL_OVERRIDE = Volts.of(-0.5);

  public static final Time HOOD_LOWER_TIME = Seconds.of(0.2);

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Hood")
          .bus(kRioBus)
          .positionTolerance(Degrees.of(3.0))
          .velocityTolerance(DegreesPerSecond.of(4.58))
          .kP(60.0)
          .kI(0.0)
          .kD(0.1)
          .cruiseVelocity(DegreesPerSecond.of(600.0))
          .acceleration(DegreesPerSecondPerSecond.of(4000.0))
          .build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(20)
          .bus(kSubsystemConfigReal.getBus())
          .resetAngle(kMinHoodAngle)
          .softMinAngle(kMinHoodAngle)
          .softMaxAngle(kMaxHoodAngle)
          .motorRotationsPerOutputRotations(187.0 / 12.0 * 36.0 / 11.0)
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .inverted(InvertedValue.CounterClockwise_Positive)
          .supplyCurrentLimit(Amps.of(30.0))
          .statorCurrentLimit(Amps.of(15.0))
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

  public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0506594076);

  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getKrakenX44(1))
          .moi(kMOI)
          .resetAngle(kTalonFXConfig.getResetAngle())
          .physicalMinAngle(kMinHoodAngle)
          .physicalMaxAngle(kMaxHoodAngle)
          .motorRotationsPerOutputRotations(kTalonFXConfig.getMotorRotationsPerOutputRotations())
          .neutralMode(kTalonFXConfig.getNeutralMode())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .cruiseVelocity(kSubsystemConfigSim.getCruiseVelocity())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .supplyCurrentLimit(kTalonFXConfig.getSupplyCurrentLimit())
          .statorCurrentLimit(kTalonFXConfig.getStatorCurrentLimit())
          .build();
}
