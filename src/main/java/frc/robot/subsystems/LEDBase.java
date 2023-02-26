package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class LEDBase extends SubsystemBase {

  /** Creates a new LEDBase. */

  private DigitalOutput led_strip;

  public LEDBase() {
    led_strip = new DigitalOutput(LEDConstants.DIO_LED_STRIP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void stop() {
    /*
     * Stubbed for now.
     */
  }

  public void toggle_led_strip() {
    led_strip.set(
      !led_strip.get()
    );
  }

}
