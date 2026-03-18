package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.subsystems.led.io.CANdleIO;
import frc.robot.subsystems.led.io.CANdleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class LED extends VirtualSubsystem {

  private static final int SlotStartIdx = 8;
  private static final int SlotEndIdx = CANdleConstants.k_candleNumLEDs + SlotStartIdx - 1;

  private final CANdleIO io;
  private final CANdleIOInputsAutoLogged inputs;

  public LED() {
    this(new CANdleIO() {}, new Trigger(() -> false));
  }

  public LED(CANdleIO io, Trigger aimed) {
    this.io = io;
    this.inputs = new CANdleIOInputsAutoLogged();

    // LED animations
    setDefaultCommand(setRainbow().ignoringDisable(true));
    aimed.whileTrue(setToGreen()).whileFalse(setToRed());
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
  }

  private Command setToRed() {
    return run(
        () -> {
          io.setControl(
              new ColorFlowAnimation(SlotStartIdx, SlotEndIdx)
                  .withSlot(0)
                  .withColor(new RGBWColor(255, 0, 0, 0)));
        });
  }

  private Command setToGreen() {
    return run(
        () -> {
          io.setControl(
              new ColorFlowAnimation(SlotStartIdx, SlotEndIdx)
                  .withSlot(0)
                  .withColor(new RGBWColor(0, 255, 0, 0)));
        });
  }

  private Command setAnimFlow() {
    return run(
        () -> {
          io.setControl(
              new ColorFlowAnimation(SlotStartIdx, SlotEndIdx)
                  .withSlot(0)
                  .withColor(new RGBWColor(0, 217, 0, 0)));
        });
  }

  private Command setRainbow() {
    return run(
        () -> {
          io.setControl(new RainbowAnimation(SlotStartIdx, SlotEndIdx).withSlot(0));
        });
  }
}
