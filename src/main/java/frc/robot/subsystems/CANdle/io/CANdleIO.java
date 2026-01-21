package frc.robot.subsystems.CANdle.io;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface CANdleIO {

  @AutoLog
  static class CANdleIOInputs {
    public String animationName;
  }

  default void setControl(ControlRequest request) {}
  ;

  default void updateInputs(CANdleIOInputs inputs) {}
  ;
}
