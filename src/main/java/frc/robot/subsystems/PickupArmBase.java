package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PickupArmBase extends SubsystemBase {

  /** Creates a new ArmBase. */

  private CANSparkMax armMotor;
  private DutyCycleEncoder armMotorEncoder;
  private DoubleSolenoid armLocker;
  private Solenoid cubeGrabber;
  private Solenoid coneGrabber;

  //pickup arm has an encoder connected to SPARK MAX
  //said encoder is capable of operating in duty-cycle mode
  //How do you specify the encoder mode to the SPARK MAX???

  public PickupArmBase() {
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    //armMotorEncoder = armMotor.getAlternateEncoder(null, 8192);
    armMotorEncoder = new DutyCycleEncoder(1);
    armMotorEncoder.setDistancePerRotation(360.0);
    armLocker = new DoubleSolenoid(
      HardwareCAN.PNEUMATIC_HUB,
      PneumaticsModuleType.REVPH,
      PneumaticChannels.ARM_LOCKER_OFF,
      PneumaticChannels.ARM_LOCKER_ON
    );

    cubeGrabber = new Solenoid(
      HardwareCAN.PNEUMATIC_HUB,
      PneumaticsModuleType.REVPH,
      PneumaticChannels.CUBE_GRAB
    );

    coneGrabber = new Solenoid(
      HardwareCAN.PNEUMATIC_HUB,
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
    // need to set 50.0 as offset for this encoder
    double pos = 50.0 - armMotorEncoder.getDistance(); //encoder is backwards, fixed by - sign

    System.out.println("arm motor pos: " + pos);
    boolean isSafe = (pos > 0 && pos < 100);
    if (!isSafe) {
      //DOES NOT WORK
      //COMMAND CONTINUES EVEN IF ARM EXCEEDS SAFE ANGLES
      //DOES WORK WHEN THE COMMAND IS STOPPED AND ANOTHER COMMAND IS ATTEMPTED
      if(pos < 0) armMotor.set(0.05);
      else if(pos>100) armMotor.set(-0.05);
      else;
      return;
    }

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

  public void grab_cone() {
    grab_cube();
    coneGrabber.set(false); //cone grabber is engaged AFTER cube grabber
  }

  public void release_cone(){
    coneGrabber.set(true);//cone grabber is released BEFORE cube grabber
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
