package frc.robot.subsystems.CANdle.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix6.controls.ControlRequest;

public interface CANdleIO {

    @AutoLog
    static class CANdleIOInputs {
        public ControlRequest animation;
    }

    default void setControl(ControlRequest request) {};

    default void updateInputs(CANdleIOInputs inputs) {};

}