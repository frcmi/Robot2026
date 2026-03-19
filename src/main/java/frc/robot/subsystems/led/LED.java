package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    setDefaultCommand(setOff().ignoringDisable(true));
    aimed.whileTrue(setToAllianceColor()).whileFalse(setToRed());
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
  }

  private RGBWColor getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      return new RGBWColor(0, 0, 255, 0); // Blue alliance
    }
    return new RGBWColor(255, 0, 0, 0); // Red alliance (also fallback)
  }

  private Command setToAllianceColor() {
    return run(
        () ->
            io.setControl(
                new ColorFlowAnimation(SlotStartIdx, SlotEndIdx)
                    .withSlot(0)
                    .withColor(getAllianceColor())));
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

  private Command setOff() {
    return run(
        () -> {
          io.setControl(
              new ColorFlowAnimation(SlotStartIdx, SlotEndIdx)
                  .withSlot(0)
                  .withColor(new RGBWColor(0, 0, 0, 0)));
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

  private Command setLarson() {
    return run(
        () -> {
          io.setControl(new LarsonAnimation(SlotStartIdx, SlotEndIdx).withSlot(0));
        });
  }
}
