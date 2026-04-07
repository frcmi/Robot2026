package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.subsystems.led.io.CANdleIO;
import frc.robot.subsystems.led.io.CANdleIOInputsAutoLogged;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class LED extends VirtualSubsystem {

  private static final int SlotStartIdx = 8;
  private static final int SlotEndIdx = CANdleConstants.k_candleNumLEDs + SlotStartIdx - 1;

  private final CANdleIO io;
  private final CANdleIOInputsAutoLogged inputs;

  public LED() {
    this(new CANdleIO() {}, new Trigger(() -> false), new Trigger(() -> false));
  }

  public LED(CANdleIO io, Trigger aimed, BooleanSupplier attemptingShooting) {
    this.io = io;
    this.inputs = new CANdleIOInputsAutoLogged();

    // LED animations
    setDefaultCommand(setRainbow().ignoringDisable(true));
    Trigger browned =
        new Trigger(RobotController::isBrownedOut)
            .debounce(0.5, DebounceType.kFalling)
            .whileTrue(setColor(new RGBWColor(0, 0, 255), () -> false));
    aimed
        .and(browned.negate())
        .whileTrue(setColor(new RGBWColor(0, 255, 0), attemptingShooting))
        .whileFalse(setColor(new RGBWColor(255, 0, 0), attemptingShooting));
  }

  @Override
  public void periodic() {
    double start = 0.0;
    if (Constants.kEnableLoopTimingLogs) {
      start = Timer.getFPGATimestamp();
    }
    this.io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
    if (Constants.kEnableLoopTimingLogs) {
      double end = Timer.getFPGATimestamp();

      Logger.recordOutput("Timing/CANDleMS", (end - start) * 1000);
    }
  }

  private Command setColor(RGBWColor color, BooleanSupplier solid) {
    return run(
        () -> {
          if (!solid.getAsBoolean()) {
            io.setControl(
                new StrobeAnimation(SlotStartIdx, SlotEndIdx).withSlot(0).withColor(color));
          } else {
            io.setControl(new SolidColor(SlotStartIdx, SlotEndIdx).withColor(color));
          }
        });
  }

  private Command setRainbow() {
    return run(
        () -> {
          io.setControl(new RainbowAnimation(SlotStartIdx, SlotEndIdx).withSlot(0));
        });
  }
}
