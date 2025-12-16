package org.neiacademy.robotics.frc2026.subsystems.misc.GenericLaserCANSubsystem;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface GenericLaserCANSubsystemIO {
  @AutoLog
  abstract class LaserCANIOInputs {
    public Distance distance;
  }

  default boolean getValidStatus() {
    return true;
  }

  default void updateInputs(LaserCANIOInputs inputs) {}
}
