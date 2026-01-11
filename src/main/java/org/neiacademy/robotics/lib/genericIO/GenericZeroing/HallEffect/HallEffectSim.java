package org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect;

import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;

/**
 * Simple hall effect simulation source driven from a tunable dashboard value. The tunable
 * represents the logical "triggered" state (1 = triggered, 0 = not triggered).
 */
public class HallEffectSim {
  private final LoggedTunableNumber simulatedTriggered;

  public HallEffectSim(String name) {
    simulatedTriggered =
        new LoggedTunableNumber("HallEffectSensors/" + name + "/SimTriggered", 0.0);
  }

  /** Returns the simulated logical triggered state. */
  public boolean getTriggered() {
    return simulatedTriggered.get() > 0.5;
  }
}
