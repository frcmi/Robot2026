package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.linear.LinearIO;
import frc.robot.lib.subsystem.linear.LinearSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Climber extends VirtualSubsystem {
  public final LinearSubsystem climber1;
  public final LinearSubsystem climber2;

  @Getter private Distance targetState = ClimberConstants.kStowedDistance;
  @Getter private Distance measuredState;

  public Climber() {
    this(
        new LinearSubsystem(new LinearIO() {}, ClimberConstants.kSubsystemConfigReal),
        new LinearSubsystem(new LinearIO() {}, ClimberConstants.kSubsystemConfigReal));
  }

  public Climber(LinearSubsystem climber1, LinearSubsystem climber2) {
    this.climber1 = climber1;
    this.climber2 = climber2;

    climber1.setDefaultCommand(climber1.holdAtGoal(() -> targetState));
    climber2.setDefaultCommand(climber2.holdAtGoal(() -> targetState));

    measuredState = climber1.getLength();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState = climber1.getLength();

    Logger.recordOutput("Climber/TargetState", targetState);
    Logger.recordOutput("Climber/MeasuredState", measuredState);
  }

  public Command up() {
    return runOnce(
        () -> {
          targetState = ClimberConstants.kExtendedDistance;
        });
  }

  public Command down() {
    return runOnce(
        () -> {
          targetState = ClimberConstants.kStowedDistance;
        });
  }
}
