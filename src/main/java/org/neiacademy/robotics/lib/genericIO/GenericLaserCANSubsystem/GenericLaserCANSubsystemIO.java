package org.neiacademy.robotics.lib.genericIO.GenericLaserCANSubsystem;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;

public interface GenericLaserCANSubsystemIO {
    @AutoLog
    abstract class LaserCANIOInputs {
        public Distance distance;
    }

    default boolean getValidStatus() 
    {
        return true;
    }
    
    default void updateInputs(LaserCANIOInputs inputs)
    {}
}