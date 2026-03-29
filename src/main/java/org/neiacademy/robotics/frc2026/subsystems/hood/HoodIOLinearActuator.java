package org.neiacademy.robotics.frc2026.subsystems.hood;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import org.neiacademy.robotics.frc2026.Constants;

public class HoodIOLinearActuator implements HoodIO {
  private static final Distance kServoLength = Constants.Hood.LENGTH_MILLIMETERS;
  private static final LinearVelocity kMaxServoSpeed = Constants.Hood.MAX_SPEED;
  private static final double kMinPosition = Constants.Hood.MIN_POSITION;
  private static final double kMaxPosition = Constants.Hood.MAX_POSITION;
  private static final double kPositionTolerance = Constants.Hood.POSITION_TOLERANCE;
  private Servo leftServo;
  private Servo rightServo;

  @Getter private double currentPosition = 0.0; // Start at unknown position

  @Getter private double targetPosition = 0.0; // No target until explicitly commanded
  private Time lastUpdateTime = Seconds.of(0);

  public HoodIOLinearActuator() {
    leftServo = new Servo(Constants.Hood.LEFT_CHANNEL);
    rightServo = new Servo(Constants.Hood.RIGHT_CHANNEL);
    leftServo.setBoundsMicroseconds(
        Constants.Hood.LINEAR_ACTUATOR_MAX,
        Constants.Hood.LINEAR_ACTUATOR_DEADBAND_MAX,
        Constants.Hood.LINEAR_ACTUATOR_CENTER,
        Constants.Hood.LINEAR_ACTUATOR_DEADBAND_MIN,
        Constants.Hood.LINEAR_ACTUATOR_MIN);
    rightServo.setBoundsMicroseconds(
        Constants.Hood.LINEAR_ACTUATOR_MAX,
        Constants.Hood.LINEAR_ACTUATOR_DEADBAND_MAX,
        Constants.Hood.LINEAR_ACTUATOR_CENTER,
        Constants.Hood.LINEAR_ACTUATOR_DEADBAND_MIN,
        Constants.Hood.LINEAR_ACTUATOR_MIN);
  }

  @Override
  public void setPositionNormalized(double position) {
    final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
    leftServo.set(clampedPosition);
    rightServo.set(clampedPosition);
    targetPosition = clampedPosition;
  }

  @Override
  public void resetPosition() {
    currentPosition = 0;
  }

  @Override
  public void setSpeedNormalized(double speed) {
    leftServo.setSpeed(speed);
    rightServo.setSpeed(speed);
  }

  @Override
  public boolean isPositionWithinTolerance() {
    return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
  }

  @Override
  public void updateCurrentPosition() {
    final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
    final Time elapsedTime = currentTime.minus(lastUpdateTime);
    lastUpdateTime = currentTime;

    if (isPositionWithinTolerance()) {
      currentPosition = targetPosition;
      return;
    }

    final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
    final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
    currentPosition =
        targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.leftSetpointPosition = targetPosition;
    inputs.rightSetpointPosition = targetPosition;
    inputs.leftNormalizedPosition = currentPosition;
    inputs.leftNormalizedPosition = currentPosition;
  }
}
