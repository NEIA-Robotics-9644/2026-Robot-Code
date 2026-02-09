package org.neiacademy.robotics.frc2026.subsystems.indexer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase{
    private final IndexerIO wheel;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    public Indexer(IndexerIO wheel){
        this.wheel = wheel;
    }
    public void periodic(){
        wheel.updateInputs(inputs);
        wheel.periodic();
        Logger.processInputs("Indexer ", inputs);
    }

    public void setVelocity(double velocity) {
        wheel.setVelocity(velocity);
    }


}
