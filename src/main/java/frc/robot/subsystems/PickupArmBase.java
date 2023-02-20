package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PickupArmBase extends SubsystemBase {

  /** Creates a new ArmBase. */

  private CANSparkMax armMotor;
  private RelativeEncoder armMotorEncoder;


  public PickupArmBase() {
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorEncoder = armMotor.getEncoder();
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
    double pos = armMotorEncoder.getPosition();

    System.out.println("arm motor pos: " + pos);

    if (direction) {
        armMotor.set(power);
    }
    else {
        armMotor.set(-power);
    }
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    armMotor.set(0.0);
  }
}
