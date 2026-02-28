// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOTalonFXConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.subsystems.intake.IntakeState;
import java.util.function.Supplier;

public class PivotConstants {
  public static final Distance PivotLength = Inches.of(14.9);
  public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
      () -> Rotation2d.fromRotations(0.165);

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Pivot")
          .bus(kRioBus)
          .positionTolerance(Degrees.of(2.0))
          .velocityTolerance(DegreesPerSecond.of(4.58))
          .kP(5)
          .kI(0.0)
          .kD(0.1)
          .kS(0.1)
          .kV(0.3)
          .kA(0.05)
          .cruiseVelocity(RotationsPerSecond.of(1.2))
          .acceleration(RotationsPerSecondPerSecond.of(2.4))
          .build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(32)
          .followerId(33)
          .opposeMaster(true)
          .bus(kRioBus)
          .resetAngle(IntakeState.kInit.getPivot())
          .softMinAngle(IntakeState.kIntaking.getPivot())
          .softMaxAngle(IntakeState.kInit.getPivot())
          .motorRotationsPerOutputRotations(5.0 * 5.0) // The reductions on the intake
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .inverted(InvertedValue.CounterClockwise_Positive)
          .supplyCurrentLimit(Amps.of(30.0))
          .statorCurrentLimit(Amps.of(90.0))
          .neutralMode(NeutralModeValue.Brake)
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kS(kSubsystemConfigReal.getKS())
          .kV(kSubsystemConfigReal.getKV())
          .kA(kSubsystemConfigReal.getKA())
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigSim =
      AngularSubsystemConfig.builder()
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

  public static final MomentOfInertia kMOI =
      KilogramSquareMeters.of(
          0.00125); // KilogramSquareMeters.of(293.783602 * 0.000292639653); // Converted from lb
  // in^2 to kg m^2
  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getKrakenX60(1))
          .moi(kMOI)
          .resetAngle(kTalonFXConfig.getResetAngle())
          .physicalMinAngle(IntakeState.kIntaking.getPivot())
          .physicalMaxAngle(IntakeState.kInit.getPivot())
          .motorRotationsPerOutputRotations(kTalonFXConfig.getMotorRotationsPerOutputRotations())
          .neutralMode(kTalonFXConfig.getNeutralMode())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .cruiseVelocity(kSubsystemConfigSim.getCruiseVelocity())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .build();
}
