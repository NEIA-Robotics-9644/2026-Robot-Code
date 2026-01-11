package org.neiacademy.robotics.frc2026.subsystems.test.HallEffect;

import org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect.GenericHallEffectSensorConstants;
import org.neiacademy.robotics.lib.genericIO.GenericZeroing.HallEffect.GenericHallEffectSensorIOImpl.TriggerType;

public class TestHallEffectConstants {
  public static final GenericHallEffectSensorConstants kSubSysConstants =
      new GenericHallEffectSensorConstants();

  static {
    kSubSysConstants.kName = "testHallEffect";
    kSubSysConstants.kDIOport = 9;
    kSubSysConstants.ktriggerType = TriggerType.ACTIVE_LOW;
  }
}
