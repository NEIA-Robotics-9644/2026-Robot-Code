package org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect;

import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

public class GenericHallEffectSensorIOImpl implements GenericHallEffectSensorIO {

  /** Defines how the hall effect sensor signals a trigger */
  public enum TriggerType {
    /** Sensor is triggered when the DIO input is HIGH (true) */
    ACTIVE_HIGH,

    /** Sensor is triggered when the DIO input is LOW (false) */
    ACTIVE_LOW
  }

  private final DigitalInput input;
  private final TriggerType triggerType;
  private final String name;
  private final boolean isSim;
  private final HallEffectSim sim;

  /**
   * @param constants Configuration for the hall effect sensor (name, port, trigger type)
   * @param isSim Whether this IO should run in simulation mode
   */
  public GenericHallEffectSensorIOImpl(GenericHallEffectSensorConstants constants, boolean isSim) {
    this.name = constants.kName;
    this.triggerType = constants.ktriggerType;
    this.isSim = isSim;

    if (isSim) {
      this.input = null;
      this.sim = new HallEffectSim(constants.kName);
    } else {
      this.input = new DigitalInput(constants.kDIOport);
      this.sim = null;
    }
  }

  @Override
  public void updateInputs(HallEffectSensorIOInputs inputs) {
    boolean rawValue;
    boolean triggered;

    if (isSim) {
      triggered = sim.getTriggered();
      rawValue = triggerType == TriggerType.ACTIVE_HIGH ? triggered : !triggered;
    } else {
      rawValue = input.get(); // true = HIGH, false = LOW
      triggered =
          switch (triggerType) {
            case ACTIVE_HIGH -> rawValue;
            case ACTIVE_LOW -> !rawValue;
          };
    }

    inputs.triggered = triggered;

    Logger.recordOutput("HallEffectSensors/IO/" + name + "/Raw", rawValue);
    Logger.recordOutput("HallEffectSensors/IO/" + name + "/Triggered", triggered);
  }
}
