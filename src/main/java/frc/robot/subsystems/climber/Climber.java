package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public final TalonFX climber1 = new TalonFX(0);
  public final TalonFX climber2 = new TalonFX(0);

  private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  private Slot0Configs slot0Configs = talonFXConfigs.Slot0;
  private MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private boolean up = false;

  private double upAngle = 10;

  public Climber() {
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 5;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kG = 0;

    motionMagicConfigs.MotionMagicCruiseVelocity = 80;
    motionMagicConfigs.MotionMagicAcceleration = 160;
    motionMagicConfigs.MotionMagicJerk = 1600;

    climber1.getConfigurator().apply(talonFXConfigs);
    climber2.getConfigurator().apply(talonFXConfigs);

    climber1.setPosition(0);
    climber2.setPosition(0);

    setDefaultCommand(setAngle());
  }

  private Command setAngle() {
    return run(
        () -> {
          climber1.setControl(m_request.withPosition(up ? upAngle : 0));
          climber2.setControl(m_request.withPosition(up ? upAngle : 0));
        });
  }

  public Command up() {
    return runOnce(
        () -> {
          up = true;
        });
  }

  public Command down() {
    return runOnce(
        () -> {
          up = false;
        });
  }
}
