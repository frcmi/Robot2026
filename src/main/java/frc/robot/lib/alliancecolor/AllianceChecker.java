package frc.robot.lib.alliancecolor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.subsystem.VirtualSubsystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class AllianceChecker extends VirtualSubsystem {

  // 1 is blue, 0 is red
  private LoggedTunableNumber simulationAlliance =
      new LoggedTunableNumber("Alliance/SimulationAlliance", 0.0);

  private final List<AllianceUpdatedObserver> observers = new ArrayList<>();
  private Optional<Alliance> alliance = DriverStation.getAlliance();

  public void registerObserver(AllianceUpdatedObserver observer) {
    observers.add(observer);
  }

  public void registerObservers(AllianceUpdatedObserver... addObservers) {
    Collections.addAll(observers, addObservers);
  }

  public void periodic() {
    if (Robot.isReal()) {
      alliance = DriverStation.getAlliance();
    } else {
      if (simulationAlliance.get() == 1.0) {
        alliance = Optional.of(Alliance.Blue);
      } else {
        alliance = Optional.of(Alliance.Red);
      }
    }
    alliance.ifPresent(color -> observers.forEach(observer -> observer.onAllianceFound(color)));
  }
}
