package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticChannels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PickupArmBase extends SubsystemBase {

  /** Creates a new ArmBase. */

  private CANSparkMax armMotor;
  private RelativeEncoder armMotorEncoder;
  private DoubleSolenoid armLocker;
  private Solenoid cubeGrabber;
  private Solenoid coneGrabber;

  //pickup arm has an encoder connected to SPARK MAX
  //said encoder is capable of operating in duty-cycle mode
  //How do you specify the encoder mode to the SPARK MAX???

  public PickupArmBase() {
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorEncoder = armMotor.getEncoder();

    armLocker = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      PneumaticChannels.ARM_LOCKER_OFF,
      PneumaticChannels.ARM_LOCKER_ON
    );

    cubeGrabber = new Solenoid(
      PneumaticsModuleType.REVPH,
      PneumaticChannels.CUBE_GRAB
    );
    
    coneGrabber = new Solenoid(
      PneumaticsModuleType.REVPH,
      PneumaticChannels.CONE_GRAB
    );
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

  public void grab_cube(){
    cubeGrabber.set(true);
  }

  public void release_cube(){
    cubeGrabber.set(false);
  }

  public void grab_cone(){
    grab_cube();
    coneGrabber.set(true); //cone grabber is engaged AFTER cube grabber
  }

  public void release_cone(){
    coneGrabber.set(false);//cone grabber is released BEFORE cube grabber
    release_cube();
  }

  public void lock_arm(){
    armLocker.set(Value.kForward);
  }

  public void unlock_arm(){
    armLocker.set(Value.kReverse);
  }

  public void stop() {
    /*
     * Turn off all motors.
     */

    armMotor.set(0.0);
  }
}
