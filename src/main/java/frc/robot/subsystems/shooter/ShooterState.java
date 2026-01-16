// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import lombok.Getter;
import lombok.Setter;

/** Add your docs here. */
public class ShooterState implements StructSerializable {
  @Setter @Getter private Angle turret; // yaw, 0 = forward, positive CCW (from above)

  @Setter @Getter private Angle hood; // 0 = ball shoots vertical, 90 = ball shoots horizontal

  @Setter @Getter private AngularVelocity flywheel;

  public ShooterState(Angle turret, Angle hood, AngularVelocity flywheel) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
  }

  public static final ShooterState kStowed =
      new ShooterState(Degrees.of(0.0), Degrees.of(0.0), DegreesPerSecond.of(0.0));

  public static final Struct<ShooterState> struct =
      new Struct<>() {
        @Override
        public Class<ShooterState> getTypeClass() {
          return ShooterState.class;
        }

        @Override
        public String getTypeName() {
          return "ShooterState";
        }

        @Override
        public int getSize() {
          return kSizeDouble * 2 + 256;
        }

        @Override
        public String getSchema() {
          // spotless:off
      return "double turretAngleDegrees;double hoodAngleDegrees;double flywheelAngularVelocityDegreesPerSecond";
      // spotless:on
        }

        @Override
        public ShooterState unpack(ByteBuffer bb) {
          Angle turret = Degrees.of(bb.getDouble());
          Angle hood = Degrees.of(bb.getDouble());
          AngularVelocity flywheel = DegreesPerSecond.of(bb.getDouble());

          return new ShooterState(turret, hood, flywheel);
        }

        @Override
        public void pack(ByteBuffer bb, ShooterState value) {
          bb.putDouble(value.getTurret().in(Degrees));
          bb.putDouble(value.getHood().in(Degrees));
          bb.putDouble(value.getFlywheel().in(DegreesPerSecond));
        }

        @Override
        public boolean isImmutable() {
          return true;
        }
      };
}
