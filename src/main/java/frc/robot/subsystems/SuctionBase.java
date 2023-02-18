package frc.robot.subsystems;

import frc.robot.Constants.SuctionConstants;
import frc.robot.Robot;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SuctionBase extends SubsystemBase {

  /** Creates a new SuctionBase. */

  public DoubleSolenoid solenoid;

  public SuctionBase() {
    solenoid = new DoubleSolenoid(
      HardwareCAN.PneumaticHUB, 
      PneumaticsModuleType.REVPH, 
      PneumaticChannels.FORWARD, 
      PneumaticChannels.REVERSE
    );
  }

  public void enableCompressor() {
    Robot.compressor.enableDigital();
  }

  public void openConeGrabber() {
    solenoid.set(Value.kForward);
  }
  public void closeConeGrabber() {
    solenoid.set(Value.kReverse);
  }

  public void stop() {
    /*
     * Keep the solenoid closed in our default state.
     */


     solenoid.set(Value.kReverse);
  }

  /**

   * Example command factory method.

   *

   * @return a command

   */

  public CommandBase exampleMethodCommand() {

    // Inline construction of command goes here.

    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(

        () -> {

          /* one-time action goes here */

        });

  }


  /**

   * An example method querying a boolean state of the subsystem (for example, a digital sensor).

   *

   * @return value of some boolean subsystem state, such as a digital sensor.

   */

  public boolean exampleCondition() {

    // Query some boolean state, such as a digital sensor.

    return false;

  }


  @Override

  public void periodic() {

    // This method will be called once per scheduler run

  }


  @Override

  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation

  }
}