// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.intake;

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

public class TransferConstants {
  public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
      () -> Rotation2d.kZero;

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder().logKey("Transfer").bus(kRioBus).build();

  public static final AngularIOTalonFXConfig kTalonFXConfig =
      AngularIOTalonFXConfig.builder()
          .masterId(21) // TODO: Update with actual motor ID
          .bus(kRioBus)
          .inverted(InvertedValue.Clockwise_Positive)
          .supplyCurrentLimit(Amps.of(40))
          .statorCurrentLimit(Amps.of(60))
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey(kSubsystemConfigReal.getLogKey())
          .bus(kSubsystemConfigReal.getBus())
          .build();
  public static final MomentOfInertia kMOI =
      KilogramSquareMeters.of(0.000292639653); // Converted from lb in^2 to kg m^2
  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder().motor(DCMotor.getKrakenX60(1)).moi(kMOI).build();
}
