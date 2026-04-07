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
  private ControlRequest lastControlRequest = null;

  // Pre-generated control requests to avoid allocations during periodic
  private final ControlRequest rainbowRequest;
  private final ControlRequest strobeRedRequest;
  private final ControlRequest solidRedRequest;
  private final ControlRequest strobeGreenRequest;
  private final ControlRequest solidGreenRequest;
  private final ControlRequest strobeBlueRequest;
  private final ControlRequest solidBlueRequest;

  public LED() {
    this(new CANdleIO() {}, new Trigger(() -> false), new Trigger(() -> false));
  }

  public LED(CANdleIO io, Trigger aimed, BooleanSupplier attemptingShooting) {
    this.io = io;
    this.inputs = new CANdleIOInputsAutoLogged();

    // Pre-generate all ControlRequest objects so we don't allocate or build
    // animation strings while the scheduler is running.
    rainbowRequest = new RainbowAnimation(SlotStartIdx, SlotEndIdx).withSlot(0);
    strobeRedRequest =
        new StrobeAnimation(SlotStartIdx, SlotEndIdx)
            .withSlot(0)
            .withColor(new RGBWColor(255, 0, 0));
    solidRedRequest = new SolidColor(SlotStartIdx, SlotEndIdx).withColor(new RGBWColor(255, 0, 0));
    strobeGreenRequest =
        new StrobeAnimation(SlotStartIdx, SlotEndIdx)
            .withSlot(0)
            .withColor(new RGBWColor(0, 255, 0));
    solidGreenRequest =
        new SolidColor(SlotStartIdx, SlotEndIdx).withColor(new RGBWColor(0, 255, 0));
    strobeBlueRequest =
        new StrobeAnimation(SlotStartIdx, SlotEndIdx)
            .withSlot(0)
            .withColor(new RGBWColor(0, 0, 255));
    solidBlueRequest = new SolidColor(SlotStartIdx, SlotEndIdx).withColor(new RGBWColor(0, 0, 255));

    // LED animations
    setDefaultCommand(setRainbow().ignoringDisable(true));
    Trigger browned =
        new Trigger(RobotController::isBrownedOut)
            .debounce(0.5, DebounceType.kFalling)
            .whileTrue(setBlue(() -> false));
    aimed
        .and(browned.negate())
        .whileTrue(setGreen(attemptingShooting))
        .whileFalse(setRed(attemptingShooting));
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

  private void setControlIfChanged(ControlRequest request) {
    if (request == null) {
      return;
    }
    if (lastControlRequest != request) {
      io.setControl(request);
      lastControlRequest = request;
    }
  }

  private Command setRed(BooleanSupplier solid) {
    return run(
        () -> {
          if (!solid.getAsBoolean()) {
            setControlIfChanged(strobeRedRequest);
          } else {
            setControlIfChanged(solidRedRequest);
          }
        });
  }

  private Command setGreen(BooleanSupplier solid) {
    return run(
        () -> {
          if (!solid.getAsBoolean()) {
            setControlIfChanged(strobeGreenRequest);
          } else {
            setControlIfChanged(solidGreenRequest);
          }
        });
  }

  private Command setBlue(BooleanSupplier solid) {
    return run(
        () -> {
          if (!solid.getAsBoolean()) {
            setControlIfChanged(strobeBlueRequest);
          } else {
            setControlIfChanged(solidBlueRequest);
          }
        });
  }

  private Command setRainbow() {
    return run(() -> setControlIfChanged(rainbowRequest));
  }
}
