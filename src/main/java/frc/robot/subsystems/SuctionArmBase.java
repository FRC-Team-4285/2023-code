package frc.robot.subsystems;

import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SuctionArmBase extends SubsystemBase {

  /** Creates a new SuctionArmBase. */

  private DoubleSolenoid armLocker;
  private Solenoid cubeGrabber;
  private Solenoid coneGrabber;

  public SuctionArmBase() {
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

  public void release_cone() {
    coneGrabber.set(true);//cone grabber is released BEFORE cube grabber
    release_cube();
  }

  public void lock_arm() {
    armLocker.set(Value.kReverse);
  }

  public void unlock_arm() {
    armLocker.set(Value.kForward);
  }

  public void stop() {
  }
}
