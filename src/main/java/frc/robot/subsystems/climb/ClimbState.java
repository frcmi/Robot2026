// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.utils.StructUtils;
import java.nio.ByteBuffer;
import java.util.Optional;
import lombok.Setter;

/** Climb state class representing the position of the climber. */
public class ClimbState implements StructSerializable {
  @Setter private Distance climber;
  private final String type;

  private final Optional<LoggedTunableNumber> climberTunable;

  public ClimbState(Distance climber) {
    this.climber = climber;

    climberTunable = Optional.empty();

    type = "kNotTunable";
  }

  public ClimbState(Distance climber, String logKey) {
    this.climber = climber;
    this.type = logKey;

    climberTunable =
        Optional.of(
            new LoggedTunableNumber(
                String.format("ClimbStates/%s/ClimberPositionInches", logKey), climber.in(Inches)));
  }

  public Distance getClimber() {
    return climberTunable
        .map(loggedTunableNumber -> Inches.of(loggedTunableNumber.get()))
        .orElse(climber);
  }

  // States
  public static final ClimbState kStowed = new ClimbState(Inches.of(0.0), "kStowed");
  public static final ClimbState kRaised = new ClimbState(Inches.of(10.5), "kRaised");
  public static final ClimbState kClimbed = new ClimbState(Inches.of(0.0), "kClimbed");

  @SuppressWarnings("unused")
  public static final Struct<ClimbState> struct =
      new Struct<>() {
        @Override
        public Class<ClimbState> getTypeClass() {
          return ClimbState.class;
        }

        @Override
        public String getTypeName() {
          return "ClimbState";
        }

        @Override
        public int getSize() {
          return kSizeDouble + 256;
        }

        @Override
        public String getSchema() {
          // spotless:off
          return "double climberPositionInches;char Type[256]";
          // spotless:on
        }

        @Override
        public ClimbState unpack(ByteBuffer bb) {
          Distance climber = Inches.of(bb.getDouble());
          String type = StructUtils.readString(bb, 256);

          return new ClimbState(climber);
        }

        @Override
        public void pack(ByteBuffer bb, ClimbState value) {
          bb.putDouble(value.getClimber().in(Inches));
          StructUtils.writeString(bb, value.type, 256);
        }

        @Override
        public boolean isImmutable() {
          return true;
        }
      };
}
