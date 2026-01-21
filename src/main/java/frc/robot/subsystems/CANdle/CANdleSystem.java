package frc.robot.subsystems.CANdle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CANdle.io.CANdleIO;

public class CANdleSystem extends SubsystemBase {

  private static final int SlotStartIdx = 8;
  private static final int SlotEndIdx = 37;

  private final CANdleIO io;

  public CANdleSystem() {
    this(new CANdleIO() {});
  }

  public CANdleSystem(CANdleIO io) {
    this.io = io;
  }

  private Command setAnimFlow() {
    return run(() -> {
          io.setControl(
              new ColorFlowAnimation(SlotStartIdx, SlotEndIdx)
                  .withSlot(0)
                  .withColor(new RGBWColor(0, 217, 0, 0)));
        });
  }
}
