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
  @Setter private Angle minPivot;
  @Setter private Angle maxPivot;
  @Setter private Voltage rollers;
  private final String type;

  private final Optional<LoggedTunableNumber> pivotTunable;
  private final Optional<LoggedTunableNumber> rollerVoltageTunable;

  public IntakeState(Angle pivot, Voltage rollers) {
    this.pivot = pivot;
    this.rollers = rollers;

    pivotTunable = Optional.empty();
    rollerVoltageTunable = Optional.empty();

    type = "kNotTunable";
  }

  public IntakeState(Angle minPivot, Angle maxPivot) {
    this.minPivot = minPivot;
    this.maxPivot = maxPivot;
    this.rollers = Volts.of(0);

    pivotTunable = Optional.empty();
    rollerVoltageTunable = Optional.empty();

    type = "kNotTunable";
  }

  public IntakeState(Angle pivot, Voltage rollers, String logKey) {
    this.pivot = pivot;
    this.rollers = rollers;
    this.type = logKey;

    pivotTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("IntakeStates/%s/PivotAngleDegrees", logKey), pivot.in(Degrees)));
    rollerVoltageTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("IntakeStates/%s/RollersVolts", logKey), rollers.in(Volts)));
  }

  public IntakeState(Angle minPivot, Angle maxPivot, String logKey) {
    this.minPivot = minPivot;
    this.maxPivot = maxPivot;
    this.pivot = minPivot.plus(maxPivot).div(2);
    this.rollers = Volts.of(0);
    this.type = logKey;

    pivotTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("IntakeStates/%s/PivotAngleDegrees", logKey), pivot.in(Degrees)));
    rollerVoltageTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("IntakeStates/%s/RollersVolts", logKey), rollers.in(Volts)));
  }

  public Angle getPivot() {
    return pivotTunable
        .map(loggedTunableNumber -> Degrees.of(loggedTunableNumber.get()))
        .orElse(pivot);
  }

  public Voltage getRollers() {
    return rollerVoltageTunable
        .map(loggedTunableNumber -> Volts.of(loggedTunableNumber.get()))
        .orElse(rollers);
  }

  public Angle getMin() {
    return minPivot;
  }

  public Angle getMax() {
    return maxPivot;
  }

  // States
  public static final IntakeState kInit =
      new IntakeState(Rotations.of(0.202), Volts.of(0.0f), "kInit");
  public static final IntakeState kStowed =
      new IntakeState(Degrees.of(20.0), Volts.of(0.0f), "kStowed");
  public static final IntakeState kTransferring =
      new IntakeState(Degrees.of(23.0), Volts.of(4.0f), "kTransfer");
  public static final IntakeState kIntaking =
      new IntakeState(Degrees.of(0.0), Volts.of(5.0f), "kIntaking");
  public static final IntakeState kReversing =
      new IntakeState(Degrees.of(0.0), Volts.of(-8.0f), "kReversing");
  public static final IntakeState kDown =
      new IntakeState(Degrees.of(0.0), Volts.of(0.0f), "kDown");

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
          return kSizeDouble * 2 + 256;
        }

        @Override
        public String getSchema() {
          // spotless:off
                    return "double pivotAngleDegrees;double rollerVoltageVolts;char Type[256]";
                    // spotless:on
        }

        @Override
        public IntakeState unpack(ByteBuffer bb) {
          Angle pivot = Degrees.of(bb.getDouble());
          Voltage rollers = Volts.of(bb.getDouble());
          String type = StructUtils.readString(bb, 256);

          return new IntakeState(pivot, rollers, type);
        }

        @Override
        public void pack(ByteBuffer bb, IntakeState value) {
          bb.putDouble(value.getPivot().in(Degrees));
          bb.putDouble(value.getRollers().in(Volts));
          StructUtils.writeString(bb, value.type, 256);
        }

        @Override
        public boolean isImmutable() {
          return true;
        }
      };
}
