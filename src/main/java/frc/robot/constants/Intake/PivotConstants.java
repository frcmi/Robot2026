// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOTalonFXConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.subsystems.intake.IntakeState;
import java.util.Optional;
import java.util.function.Supplier;

public class PivotConstants {
  public static final Translation2d IntakeOffset = new Translation2d(Inches.of(16.0).in(Meters), 0);

  public static final Voltage MANUAL_VOLTAGE = Volts.of(-0.5);

  public static final Distance PivotLength = Inches.of(14.9);
  public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
      () -> Rotation2d.fromRotations(0.165);

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Pivot")
          .bus(kRioBus)
          .positionTolerance(Degrees.of(2.0))
          .velocityTolerance(DegreesPerSecond.of(4.58))
          .kP(40)
          .kI(0.0)
          .kD(1)
          .kG(0.08)
          .cruiseVelocity(RadiansPerSecond.of(5))
          .acceleration(RadiansPerSecondPerSecond.of(8))
          .build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(32)
          .followerId(33)
          .sensorId(Optional.of(43))
          .opposeMaster(true)
          .bus(kRioBus)
          .resetAngle(IntakeState.kInit.getPivot())
          .softMinAngle(IntakeState.kIntaking.getPivot())
          .softMaxAngle(IntakeState.kInit.getPivot())
          .motorRotationsPerOutputRotations(1) // Sensor is 1:1 with intake
          .rotorRotationsPerSensorRotation(5.0 * 4.0) // The reductions on the intake
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .inverted(InvertedValue.Clockwise_Positive)
          .supplyCurrentLimit(Amps.of(30.0))
          .statorCurrentLimit(Amps.of(120.0))
          .neutralMode(NeutralModeValue.Brake)
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kS(kSubsystemConfigReal.getKS())
          .kV(kSubsystemConfigReal.getKV())
          .kA(kSubsystemConfigReal.getKA())
          .kG(kSubsystemConfigReal.getKG())
          .gravityType(Optional.of(GravityTypeValue.Arm_Cosine))
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
          .kS(kSubsystemConfigReal.getKS())
          .kV(kSubsystemConfigReal.getKV())
          .kA(kSubsystemConfigReal.getKA())
          .kG(kSubsystemConfigReal.getKG())
          .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();

  public static final MomentOfInertia kMOI =
      KilogramSquareMeters.of(
          1.46); // KilogramSquareMeters.of(293.783602 * 0.000292639653); // Converted from lb
  // in^2 to kg m^2
  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getKrakenX44(2))
          .moi(kMOI)
          .resetAngle(kTalonFXConfig.getResetAngle())
          .physicalMinAngle(IntakeState.kIntaking.getPivot())
          .physicalMaxAngle(IntakeState.kInit.getPivot())
          .motorRotationsPerOutputRotations(
              kTalonFXConfig.getMotorRotationsPerOutputRotations()
                  * kTalonFXConfig.getRotorRotationsPerSensorRotation())
          .neutralMode(kTalonFXConfig.getNeutralMode())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .kV(kSubsystemConfigSim.getKV())
          .kG(kSubsystemConfigSim.getKG())
          .cruiseVelocity(kSubsystemConfigSim.getCruiseVelocity())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .supplyCurrentLimit(kTalonFXConfig.getSupplyCurrentLimit())
          .statorCurrentLimit(kTalonFXConfig.getStatorCurrentLimit())
          .numMotors(2)
          .kgArm(true)
          .realAngleFromSubsystemAngleZeroSupplier(
              Optional.of(kRealAngleFromSubsystemAngleZeroSupplier))
          .armLengthSupplier(Optional.of(() -> PivotLength))
          .build();
}
