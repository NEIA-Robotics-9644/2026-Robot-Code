package org.neiacademy.robotics.frc2026.subsystems.indexer;

public class IndexerIOSim implements IndexerIO{
     private double velocityRPM = 0.0;

    private double maxSpeedRPM = 5000.0;

    public double inputVelocity = 0.0;

    private boolean newInput = false;

    @Override
    public void setVelocity(double velocityPercent) {
        inputVelocity = maxSpeedRPM * velocityPercent;
        newInput = true;
    }

    @Override
    public double getVelocityPercent() {
        return velocityRPM / maxSpeedRPM;
    }

    @Override
    public void periodic() {

        if (newInput) {
        velocityRPM = inputVelocity;
        // System.out.println("New Velocity: " + velocityRPM);
        newInput = false;
        } else {
        velocityRPM = 0.0;
        // System.out.println("Reset Velocity: " + velocityRPM);
        }
    }

    @Override
    public void setBrakeMode(boolean brake) {
        // TODO Auto-generated method stub

  }
}
