// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.transfer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOTalonFXConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import java.util.function.Supplier;

public class KickerConstants {
  public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
      () -> Rotation2d.kZero;

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Kicker")
          .bus(kRioBus)
          .kP(0.13)
          .kI(0.0)
          .kD(0.0)
          .kV(
              12.0
                  / ((7530.0 / 3.0)
                      * (2 * Math.PI
                          / 60.0))) // 12V per 1200rpm output (7530rpm motor with 3:1 gearing)
          .velocityTolerance(RotationsPerSecond.of(1.6))
          .acceleration(RotationsPerSecondPerSecond.of(350.0))
          .build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(23)
          .bus(kRioBus)
          .inverted(InvertedValue.CounterClockwise_Positive)
          .motorRotationsPerOutputRotations(3)
          .supplyCurrentLimit(Amps.of(40.0))
          .supplyCurrentLowerTime(Seconds.of(0.5))
          .supplyCurrentLower(Amps.of(20.0))
          .statorCurrentLimit(Amps.of(60.0))
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey(kSubsystemConfigReal.getLogKey())
          .bus(kSubsystemConfigReal.getBus())
          .kP(kSubsystemConfigReal.getKP())
          .kI(kSubsystemConfigReal.getKI())
          .kD(kSubsystemConfigReal.getKD())
          .kV(kSubsystemConfigReal.getKV())
          .velocityTolerance(kSubsystemConfigReal.getVelocityTolerance())
          .acceleration(kSubsystemConfigReal.getAcceleration())
          .build();
  public static final MomentOfInertia kMOI =
      KilogramSquareMeters.of(0.000424327497472); // Converted from lb in^2 to kg m^2
  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getKrakenX44(1))
          .moi(kMOI)
          .motorRotationsPerOutputRotations(3)
          .supplyCurrentLimit(kTalonFXConfig.getSupplyCurrentLimit())
          .statorCurrentLimit(kTalonFXConfig.getStatorCurrentLimit())
          .kP(kSubsystemConfigSim.getKP())
          .kI(kSubsystemConfigSim.getKI())
          .kD(kSubsystemConfigSim.getKD())
          .kV(kSubsystemConfigSim.getKV())
          .acceleration(kSubsystemConfigSim.getAcceleration())
          .build();
}
