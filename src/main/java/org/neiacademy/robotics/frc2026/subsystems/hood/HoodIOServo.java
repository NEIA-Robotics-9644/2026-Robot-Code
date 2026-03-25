package org.neiacademy.robotics.frc2026.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import org.neiacademy.robotics.frc2026.Constants;

public class HoodIOServo implements HoodIO {

  private final Servo leftServo;
  private final Servo rightServo;

  private boolean leftInverted;
  private boolean rightInverted;

  private double length;

  public HoodIOServo() {
    leftServo = new Servo(Constants.Hood.LEFT_CHANNEL);
    rightServo = new Servo(Constants.Hood.RIGHT_CHANNEL);

    leftInverted = Constants.Hood.LEFT_INVERTED;
    rightInverted = Constants.Hood.RIGHT_INVERTED;

    length = Constants.Hood.LENGTH;

    setBounds(
        Constants.Hood.LINEAR_ACTUATOR_MAX,
        Constants.Hood.LINEAR_ACTUATOR_DEADBAND_MAX,
        Constants.Hood.LINEAR_ACTUATOR_CENTER,
        Constants.Hood.LINEAR_ACTUATOR_DEADBAND_MIN,
        Constants.Hood.LINEAR_ACTUATOR_MIN);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.leftNormalizedSpeed = leftServo.getSpeed();
    inputs.leftNormalizedPosition =
        leftInverted ? (1 - leftServo.getPosition()) : leftServo.getPosition();
    inputs.leftPositionMillimeters =
        leftInverted ? (140 - leftServo.getPosition() * 140) : leftServo.getPosition() * 140;

    inputs.rightNormalizedSpeed = rightServo.getSpeed();
    inputs.rightNormalizedPosition =
        rightInverted ? (1 - rightServo.getPosition()) : rightServo.getPosition();
    inputs.rightPositionMillimeters =
        rightInverted ? (140 - rightServo.getPosition() * 140) : rightServo.getPosition() * 140;
  }

  @Override
  public void setSpeed(double normalizedSpeed) {
    leftServo.setSpeed(normalizedSpeed);
    rightServo.setSpeed(normalizedSpeed);
  }

  @Override
  public void setPositionNormalized(double normalizedPosition) {
    leftServo.set(leftInverted ? (1 - normalizedPosition) : normalizedPosition);
    rightServo.set(rightInverted ? (1 - normalizedPosition) : normalizedPosition);
  }

  @Override
  public void setPositionMillimeters(double millimeters) {
    double position = MathUtil.clamp(millimeters, 0, length) / length;
    double speed = (position * 2) - 1;

    leftServo.set(leftInverted ? (1 - position) : position);
    rightServo.set(rightInverted ? (1 - position) : position);

    leftServo.setSpeed(speed);
    rightServo.setSpeed(speed);
  }

  public void resetServosPosition() {
    leftServo.setPosition(leftInverted ? 1 : 0);
    rightServo.setPosition(rightInverted ? 1 : 0);
  }

  @Override
  public void setDisabled() {
    leftServo.setDisabled();
    rightServo.setDisabled();
  }

  @Override
  public void setBounds(int max, int deadbandMax, int center, int deadbandMin, int min) {
    leftServo.setBoundsMicroseconds(max, deadbandMax, center, deadbandMin, min);
    rightServo.setBoundsMicroseconds(max, deadbandMax, center, deadbandMin, min);
  }
}
