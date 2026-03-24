package org.neiacademy.robotics.frc2026.subsystems.hood;

import org.neiacademy.robotics.frc2026.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class HoodIOServo implements HoodIO {

  private final Servo servo;
  private boolean inverted;
  private double length;
  private double speed;
  private double lastTime = 0;

  public HoodIOServo(int channel, boolean servoInverted, double length, double speed) {
    this.servo = new Servo(channel);
    this.inverted = servoInverted;
    this.length = length;
    this.speed = speed;
    setBounds(
        Constants.Shooter.LINEAR_ACTUATOR_MAX, 
        Constants.Shooter.LINEAR_ACTUATOR_DEADBAND_MAX, 
        Constants.Shooter.LINEAR_ACTUATOR_CENTER, 
        Constants.Shooter.LINEAR_ACTUATOR_DEADBAND_MIN, 
        Constants.Shooter.LINEAR_ACTUATOR_MIN);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.normalizedSpeed = Units.rotationsToRadians(servo.getSpeed());
    inputs.angleDegrees = servo.getAngle();
    //inputs.normalizedSetpoint = servo.getPosition();
    inputs.normalizedPosition = 
    inputs.rawPWM = servo.getPulseTimeMicroseconds();
  }

  /*@Override
  public void periodic(){
    double dt = Timer.getFPGATimestamp() - lastTime;
    if (curPos > setPos + m_speed * dt) {
      curPos -= m_speed * dt;
    } else if (curPos < setPos - m_speed * dt) {
      curPos += m_speed * dt;
    } else {
      curPos = setPos;
    }
  }*/

  @Override
  public void setSpeed(double normalizedSpeed) {
    servo.setSpeed(normalizedSpeed);
  }

  @Override
  public void setPositionNormalized(double normalizedPosition) {
    servo.set(normalizedPosition);
  }
  @Override
  public void setPositionMillimeters(double setpoint) {
    servo.set(MathUtil.clamp(setpoint, 0, length) / length);
  }

  @Override
  public void setDisabled() {
    servo.setDisabled();
  }

  @Override
  public void setBounds(int max, int deadbandMax, int center, int deadbandMin, int min) {
    servo.setBoundsMicroseconds(max, deadbandMax, center, deadbandMin, min);
  }

  @Override
  public void setAngle(double angleDegrees) {
    servo.setAngle(MathUtil.clamp(angleDegrees, 0, 180));
  }
}
