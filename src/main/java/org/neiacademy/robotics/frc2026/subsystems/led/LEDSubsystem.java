package org.neiacademy.robotics.frc2026.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  private final int length = 120;

  private double waveSpeed = 1; // higher is slower
  private double waveLength = 25.0;

  public LEDSubsystem(int pwmPort) {
    m_led = new AddressableLED(pwmPort);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(length);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  private void setLED_GBR(int index, Color c) {
    int r = (int) (c.red * 255);
    int g = (int) (c.green * 255);
    int b = (int) (c.blue * 255);

    // YOUR STRIP USES G, R, B order:
    m_ledBuffer.setRGB(index, g, r, b);
  }

  public void solid(Color c) {
    for (int i = 0; i < length; i++) {
      setLED_GBR(i, c);
    }
    m_led.setData(m_ledBuffer);
  }

  public void strobe(Color c1, Color c2, double periodSeconds) {
    boolean useC1 = ((Timer.getFPGATimestamp() % periodSeconds) / periodSeconds) > 0.5;
    Color chosen = useC1 ? c1 : c2;

    for (int i = 0; i < length; i++) {
      setLED_GBR(i, chosen);
    }
    m_led.setData(m_ledBuffer);
  }

  public void breath(Color c1, Color c2, double periodSeconds) {
    double t = Timer.getFPGATimestamp();
    double phase = (t % periodSeconds) / periodSeconds * 2.0 * Math.PI;

    double ratio = (Math.sin(phase) + 1.0) / 2.0;

    Color mix =
        new Color(
            lerp(c1.red, c2.red, ratio),
            lerp(c1.green, c2.green, ratio),
            lerp(c1.blue, c2.blue, ratio));

    for (int i = 0; i < length; i++) {
      setLED_GBR(i, mix);
    }
    m_led.setData(m_ledBuffer);
  }

  private double lerp(double a, double b, double t) {
    return a * (1 - t) + b * t;
  }

  public void rainbow(double periodSeconds) {
    double time = Timer.getFPGATimestamp();
    double baseHue = ((time % periodSeconds) / periodSeconds) * 180.0;

    for (int i = 0; i < length; i++) {
      int hue = (int) ((baseHue + (i * 180.0 / length)) % 180.0);

      Color c = Color.fromHSV(hue, 255, 255);
      setLED_GBR(i, c);
    }

    m_led.setData(m_ledBuffer);
  }

  public void wave(Color c1, Color c2) {
    double t = Timer.getFPGATimestamp();
    double scroll = (t % waveSpeed) / waveSpeed;

    double x = scroll * 2.0 * Math.PI;
    double step = (2.0 * Math.PI) / waveLength;

    for (int i = 0; i < length; i++) {
      x += step;

      double ratio = (Math.sin(x) + 1.0) / 2.0;

      Color mix =
          new Color(
              lerp(c1.red, c2.red, ratio),
              lerp(c1.green, c2.green, ratio),
              lerp(c1.blue, c2.blue, ratio));

      setLED_GBR(i, mix);
    }
    m_led.setData(m_ledBuffer);
  }

  public void stripes(Color[] colors, int stripeSize, double periodSeconds) {
    int sequenceLength = colors.length * stripeSize;

    int offset =
        (int) (((Timer.getFPGATimestamp() % periodSeconds) / periodSeconds) * sequenceLength);

    for (int i = 0; i < length; i++) {
      int idx = (i + offset) / stripeSize % colors.length;
      setLED_GBR(i, colors[idx]);
    }

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // wave(Color.kRed, Color.kBlue);
    // breath(Color.kGreen, Color.kBlack, 2.0);
    // solid(Color.kPurple);
    // strobe(Color.kWhite, Color.kBlack, 0.1);
    // rainbow(0.5);
    // stripes(new Color[]{Color.kRed, Color.kWhite, Color.kBlue}, 5, 2.0);

    wave(Color.kMagenta, Color.kPurple);
  }
}
