// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.climb;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.lib.subsystem.linear.LinearIOSimConfig;
import frc.robot.lib.subsystem.linear.LinearIOTalonFXConfig;
import frc.robot.lib.subsystem.linear.LinearSubsystemConfig;
import frc.robot.subsystems.climb.ClimbState;

public class ClimberConstants {
  // Physical constants for climber-in-a-box
  public static final Distance kSpoolDiameter = Inches.of(1.5); // Diameter of the winch spool
  public static final Distance kDistancePerRotation =
      Inches.of(Math.PI * kSpoolDiameter.in(Inches)); // Circumference

  // Visualization constants
  public static final Distance kClimberBaseHeight =
      Inches.of(10.0); // Height of climber base from ground
  public static final Distance kClimberWidth = Inches.of(3.0); // Width of climber visualization

  public static final LinearSubsystemConfig kSubsystemConfigReal =
      LinearSubsystemConfig.builder()
          .logKey("Climber")
          .bus(kRioBus)
          .positionTolerance(Inches.of(0.5))
          .velocityTolerance(InchesPerSecond.of(2.0))
          .kP(10.0)
          .kI(0.0)
          .kD(0.5)
          .kG(0.05) // Small gravity compensation for hanging climber
          .cruiseVelocity(InchesPerSecond.of(200.0))
          .acceleration(InchesPerSecondPerSecond.of(1000.0))
          .build();

  public static final LinearIOTalonFXConfig kTalonFXConfig =
      LinearIOTalonFXConfig.builder()
          .masterId(20) // Example CAN ID - adjust as needed
          .bus(kRioBus)
          .resetLength(ClimbState.kStowed.getClimber())
          .softMinLength(ClimbState.kStowed.getClimber())
          .softMaxLength(ClimbState.kRaised.getClimber())
          .motorRotationsPerOutputRotations(4.0) // 4:1 gear ratio for climber in a box
          .outputDistancePerOutputRotation(kDistancePerRotation)
          .inverted(InvertedValue.CounterClockwise_Positive)
          .supplyCurrentLimit(Amps.of(40.0))
          .statorCurrentLimit(Amps.of(80.0)) // Higher current for climbing
          .neutralMode(NeutralModeValue.Brake)
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kG(kSubsystemConfigReal.getKG())
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final LinearSubsystemConfig kSubsystemConfigSim =
      LinearSubsystemConfig.builder()
          .logKey(kSubsystemConfigReal.getLogKey())
          .bus(kSubsystemConfigReal.getBus())
          .positionTolerance(kSubsystemConfigReal.getPositionTolerance())
          .velocityTolerance(kSubsystemConfigReal.getVelocityTolerance())
          .kP(10.0)
          .kI(0.0)
          .kD(0.5)
          .kG(0.05)
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final Mass kCarriageMass = Pounds.of(10.0); // Estimated mass for climber mechanism

  public static final LinearIOSimConfig kSimConfig =
      LinearIOSimConfig.builder()
          .motor(DCMotor.getKrakenX60(1))
          .carriageMass(kCarriageMass)
          .resetLength(kTalonFXConfig.getResetLength())
          .physicalMinLength(ClimbState.kStowed.getClimber())
          .physicalMaxLength(ClimbState.kRaised.getClimber())
          .motorRotationsPerOutputRotations(kTalonFXConfig.getMotorRotationsPerOutputRotations())
          .outputDistancePerOutputRotation(kTalonFXConfig.getOutputDistancePerOutputRotation())
          .neutralMode(kTalonFXConfig.getNeutralMode())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .kG(kSubsystemConfigSim.getKG())
          .cruiseVelocity(kSubsystemConfigSim.getCruiseVelocity())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .build();
}
