// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.utils.StructUtils;
import java.nio.ByteBuffer;
import java.util.Optional;
import lombok.Setter;

/** Add your docs here. */
public class IntakeState implements StructSerializable {
  @Setter private Angle pivot;
  @Setter private Voltage rollers;
  @Setter private Voltage transfer;
  @Setter private Voltage kicker;
  private final String type;

  private final Optional<LoggedTunableNumber> pivotTunable;
  private final Optional<LoggedTunableNumber> rollerVelocityTunable;
  private final Optional<LoggedTunableNumber> transferVoltageTunable;
  private final Optional<LoggedTunableNumber> kickerVoltageTunable;

  public IntakeState(Angle pivot, Voltage rollers) {
    this(pivot, rollers, Volts.of(0.0), Volts.of(0.0));
  }

  public IntakeState(Angle pivot, Voltage rollers, Voltage transfer, Voltage kicker) {
    this.pivot = pivot;
    this.rollers = rollers;
    this.transfer = transfer;
    this.kicker = kicker;

    pivotTunable = Optional.empty();
    rollerVelocityTunable = Optional.empty();
    transferVoltageTunable = Optional.empty();
    kickerVoltageTunable = Optional.empty();

    type = "kNotTunable";
  }

  public IntakeState(
      Angle pivot, Voltage rollers, Voltage transfer, Voltage kicker, String logKey) {
    this.pivot = pivot;
    this.rollers = rollers;
    this.transfer = transfer;
    this.kicker = kicker;
    this.type = logKey;

    pivotTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("IntakeStates/%s/PivotAngleDegrees", logKey), pivot.in(Degrees)));
    rollerVelocityTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("ArmStates/%s/RollersVolts", logKey), rollers.in(Volts)));
    transferVoltageTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("IntakeStates/%s/TransferVolts", logKey), transfer.in(Volts)));
    kickerVoltageTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("IntakeStates/%s/KickerVolts", logKey), kicker.in(Volts)));
  }

  public Angle getPivot() {
    return pivotTunable
        .map(loggedTunableNumber -> Degrees.of(loggedTunableNumber.get()))
        .orElse(pivot);
  }

  public Voltage getRollers() {
    return rollerVelocityTunable
        .map(loggedTunableNumber -> Volts.of(loggedTunableNumber.get()))
        .orElse(rollers);
  }

  public Voltage getTransfer() {
    return transferVoltageTunable
        .map(loggedTunableNumber -> Volts.of(loggedTunableNumber.get()))
        .orElse(transfer);
  }

  public Voltage getKicker() {
    return kickerVoltageTunable
        .map(loggedTunableNumber -> Volts.of(loggedTunableNumber.get()))
        .orElse(kicker);
  }

  // States
  public static final IntakeState kStowed =
      new IntakeState(Degrees.of(97.5), Volts.of(0.0f), Volts.of(0.0), Volts.of(0.0), "kStowed");
  public static final IntakeState kIntaking =
      new IntakeState(
          Degrees.of(-28.7), Volts.of(4.2f), Volts.of(6.0f), Volts.of(10.8f), "kIntaking");

  @SuppressWarnings("unused")
  public static final Struct<IntakeState> struct =
      new Struct<>() {
        @Override
        public Class<IntakeState> getTypeClass() {
          return IntakeState.class;
        }

        @Override
        public String getTypeName() {
          return "IntakeState";
        }

        @Override
        public int getSize() {
          return kSizeDouble * 4 + 256;
        }

        @Override
        public String getSchema() {
          // spotless:off
                    return "double pivotAngleDegrees;double rollerVoltageVolts;double transferVoltageVolts;double kickerVoltageVolts;char Type[256]";
                    // spotless:on
        }

        @Override
        public IntakeState unpack(ByteBuffer bb) {
          Angle pivot = Degrees.of(bb.getDouble());
          Voltage rollers = Volts.of(bb.getDouble());
          Voltage transfer = Volts.of(bb.getDouble());
          Voltage kicker = Volts.of(bb.getDouble());
          String type = StructUtils.readString(bb, 256);

          return new IntakeState(pivot, rollers, transfer, kicker, type);
        }

        @Override
        public void pack(ByteBuffer bb, IntakeState value) {
          bb.putDouble(value.getPivot().in(Degrees));
          bb.putDouble(value.getRollers().in(Volts));
          bb.putDouble(value.getTransfer().in(Volts));
          bb.putDouble(value.getKicker().in(Volts));
          StructUtils.writeString(bb, value.type, 256);
        }

        @Override
        public boolean isImmutable() {
          return true;
        }
      };
}
