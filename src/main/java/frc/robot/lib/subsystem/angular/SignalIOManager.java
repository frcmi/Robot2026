package frc.robot.lib.subsystem.angular;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class SignalIOManager {
  private static final Map<String, List<BaseStatusSignal>> signalsByBus = new HashMap<>();

  public static void update() {
    double startTime = Timer.getFPGATimestamp();

    for (var entry : signalsByBus.entrySet()) {
      List<BaseStatusSignal> signalsForBus = entry.getValue();
      if (signalsForBus.isEmpty()) {
        continue;
      }
      BaseStatusSignal.refreshAll(signalsForBus);
    }

    double endTime = Timer.getFPGATimestamp();
    Logger.recordOutput("Timing/SignalIOManagerMS", (endTime - startTime) * 1e3);
  }

  public static void addSignals(String busName, List<BaseStatusSignal> newSignals) {
    signalsByBus.computeIfAbsent(busName, ignored -> new ArrayList<>()).addAll(newSignals);
  }

  public static void addSignals(String busName, BaseStatusSignal... signals) {
    addSignals(busName, Arrays.asList(signals));
  }
}
