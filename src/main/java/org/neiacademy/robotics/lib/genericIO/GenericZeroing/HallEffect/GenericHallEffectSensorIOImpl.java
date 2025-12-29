package org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect;

import edu.wpi.first.wpilibj.DigitalInput;

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

  /**
   * @param dioPort DIO port the hall effect sensor is plugged into
   * @param triggerType Whether the sensor is active-high or active-low
   */
  public GenericHallEffectSensorIOImpl(int dioPort, TriggerType triggerType) {
    this.input = new DigitalInput(dioPort);
    this.triggerType = triggerType;
  }

  @Override
  public void updateInputs(HallEffectSensorIOInputs inputs) {
    boolean rawValue = input.get(); // true = HIGH, false = LOW

    inputs.triggered =
        switch (triggerType) {
          case ACTIVE_HIGH -> rawValue;
          case ACTIVE_LOW -> !rawValue;
        };
  }
}
