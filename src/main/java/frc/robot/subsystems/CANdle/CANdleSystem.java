package frc.robot.subsystems.CANdle;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CANdle.io.CANdleIO;
import frc.robot.subsystems.CANdle.io.CANdleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CANdleSystem extends SubsystemBase {

  private static final int SlotStartIdx = 8;
  private static final int SlotEndIdx = 37;

  private final CANdleIO io;
  private final CANdleIOInputsAutoLogged inputs;

  public CANdleSystem() {
    this(new CANdleIO() {});
  }

  public CANdleSystem(CANdleIO io) {
    this.io = io;
    this.inputs = new CANdleIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);

    Logger.processInputs("CANdle", inputs);
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
}
