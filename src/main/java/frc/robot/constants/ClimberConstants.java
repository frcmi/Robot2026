package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.subsystem.linear.LinearIOSimConfig;
import frc.robot.lib.subsystem.linear.LinearIOTalonFXConfig;
import frc.robot.lib.subsystem.linear.LinearSubsystemConfig;

public class ClimberConstants {
  public static final Distance kStowedDistance = Inches.of(23);
  public static final Distance kExtendedDistance = Inches.of(40);
  public static final Angle kClimberAngle = Degrees.of(90);

  public static final LinearSubsystemConfig kSubsystemConfigReal =
      LinearSubsystemConfig.builder()
          .logKey("Climber")
          .bus(kRioBus)
          .positionTolerance(Meters.of(0.1))
          .velocityTolerance(MetersPerSecond.of(4.58))
          .kP(5.0)
          .kI(0.0)
          .kD(0.0)
          .cruiseVelocity(MetersPerSecond.of(0.5))
          .acceleration(MetersPerSecondPerSecond.of(3.0))
          .build();

  public static final LinearIOTalonFXConfig kTalonFXConfig1 =
      LinearIOTalonFXConfig.builder()
          .masterId(9)
          .bus(kRioBus)
          .resetLength(kStowedDistance)
          .softMinLength(kExtendedDistance)
          .softMaxLength(kStowedDistance)
          .motorRotationsPerOutputRotations(4) // The reductions on the intake
          .outputDistancePerOutputRotation(Inches.of(0.787 * Math.PI))
          //   .inverted(InvertedValue.Clockwise_Positive)
          .supplyCurrentLimit(Amps.of(30.0))
          .statorCurrentLimit(Amps.of(90.0))
          .neutralMode(NeutralModeValue.Brake)
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final LinearIOTalonFXConfig kTalonFXConfig2 =
      LinearIOTalonFXConfig.builder()
          .masterId(9)
          .bus(kRioBus)
          .resetLength(kStowedDistance)
          .softMinLength(kExtendedDistance)
          .softMaxLength(kStowedDistance)
          .motorRotationsPerOutputRotations(4) // The reductions on the intake
          .outputDistancePerOutputRotation(Inches.of(0.787 * Math.PI))
          .inverted(InvertedValue.Clockwise_Positive)
          .supplyCurrentLimit(Amps.of(30.0))
          .statorCurrentLimit(Amps.of(90.0))
          .neutralMode(NeutralModeValue.Brake)
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final LinearSubsystemConfig kSubsystemConfigSim =
      LinearSubsystemConfig.builder()
          .logKey(kSubsystemConfigReal.getLogKey())
          .bus(kSubsystemConfigReal.getBus())
          .positionTolerance(kSubsystemConfigReal.getPositionTolerance())
          .velocityTolerance(kSubsystemConfigReal.getVelocityTolerance())
          .kP(5.0)
          .kI(0.0)
          .kD(0.0)
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final LinearIOSimConfig kSimConfig =
      LinearIOSimConfig.builder()
          .motor(DCMotor.getKrakenX60(1))
          .carriageMass(Kilograms.of(1.7))
          .resetLength(kTalonFXConfig1.getResetLength())
          .physicalMinLength(kStowedDistance)
          .physicalMaxLength(kExtendedDistance)
          .motorRotationsPerOutputRotations(kTalonFXConfig1.getMotorRotationsPerOutputRotations())
          .neutralMode(kTalonFXConfig2.getNeutralMode())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .cruiseVelocity(kSubsystemConfigSim.getCruiseVelocity())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .build();
}
