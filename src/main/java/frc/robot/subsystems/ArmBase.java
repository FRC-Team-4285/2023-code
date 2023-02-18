package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmBase extends SubsystemBase {

  /** Creates a new ArmBase. */

  private CANSparkMax arm_motor;
  private RelativeEncoder arm_motor_encoder;

  public ArmBase() {

    arm_motor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    arm_motor_encoder = arm_motor.getEncoder();

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

  public void engage_arm(boolean direction) {
    /*
     * Engage arm motor.
     */

    double power = ArmConstants.ARM_MOTOR_POWER;
    double pos = arm_motor_encoder.getPosition();

    System.out.println("arm motor pos: " + pos);

    if (direction) {
        arm_motor.set(power);
    }
    else {
        arm_motor.set(-power);
    }
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    arm_motor.set(0.0);
  }
}