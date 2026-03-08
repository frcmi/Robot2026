// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transfer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.utils.StructUtils;
import java.nio.ByteBuffer;
import java.util.Optional;
import lombok.Setter;

/** Add your docs here. */
public class TransferState implements StructSerializable {
  @Setter private Voltage transfer;
  @Setter private AngularVelocity kicker;
  private final String type;

  private final Optional<LoggedTunableNumber> transferVoltageTunable;
  private final Optional<LoggedTunableNumber> kickerVoltageTunable;

  public TransferState(Voltage transfer, AngularVelocity kicker) {
    this.transfer = transfer;
    this.kicker = kicker;

    transferVoltageTunable = Optional.empty();
    kickerVoltageTunable = Optional.empty();

    type = "kNotTunable";
  }

  public TransferState(Voltage transfer, AngularVelocity kicker, String logKey) {
    this.transfer = transfer;
    this.kicker = kicker;
    this.type = logKey;

    transferVoltageTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("TransferStates/%s/TransferVolts", logKey), transfer.in(Volts)));
    kickerVoltageTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("TransferStates/%s/KickerVelRPS", logKey),
                kicker.in(RotationsPerSecond)));
  }

  public Voltage getTransfer() {
    return transferVoltageTunable
        .map(loggedTunableNumber -> Volts.of(loggedTunableNumber.get()))
        .orElse(transfer);
  }

  public AngularVelocity getKicker() {
    return kickerVoltageTunable
        .map(loggedTunableNumber -> RotationsPerSecond.of(loggedTunableNumber.get()))
        .orElse(kicker);
  }

  // States
  public static final TransferState kIdle =
      new TransferState(Volts.of(-1.0), RotationsPerSecond.of(0.0), "kIdle");
  public static final TransferState kTransferring =
      new TransferState(Volts.of(12.0), RotationsPerSecond.of(15.0), "kTransferring");
  public static final TransferState kReverse =
      new TransferState(Volts.of(-12.0), RotationsPerSecond.of(-15.0), "kReverse");

  @SuppressWarnings("unused")
  public static final Struct<TransferState> struct =
      new Struct<>() {
        @Override
        public Class<TransferState> getTypeClass() {
          return TransferState.class;
        }

        @Override
        public String getTypeName() {
          return "TransferState";
        }

        @Override
        public int getSize() {
          return kSizeDouble * 2 + 256;
        }

        @Override
        public String getSchema() {
          // spotless:off
                    return "double transferVoltageVolts;double kickerVelocityRPS;char Type[256]";
                    // spotless:on
        }

        @Override
        public TransferState unpack(ByteBuffer bb) {
          Voltage transfer = Volts.of(bb.getDouble());
          AngularVelocity kicker = RotationsPerSecond.of(bb.getDouble());
          String type = StructUtils.readString(bb, 256);

          return new TransferState(transfer, kicker, type);
        }

        @Override
        public void pack(ByteBuffer bb, TransferState value) {
          bb.putDouble(value.getTransfer().in(Volts));
          bb.putDouble(value.getKicker().in(RotationsPerSecond));
          StructUtils.writeString(bb, value.type, 256);
        }

        @Override
        public boolean isImmutable() {
          return true;
        }
      };
}
