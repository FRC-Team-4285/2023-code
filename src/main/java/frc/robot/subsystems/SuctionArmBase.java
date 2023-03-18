package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;
import frc.robot.Constants.SuctionConstants;
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
  private RobotContainer robotContainer;

  private boolean isEngaged = false;
  private boolean isJiggled = false;
  private double lastJiggleEventTime = 0.0;
  private int jiggleCount = 0;

  public SuctionArmBase(RobotContainer container) {
    robotContainer = container;

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
    // This method will be called once per scheduler run

    // If we are engaged...
    if (isEngaged) {

      // Get jiggle status.
      int jiggleStatus = checkIsJiggleReady();

      // Jiggle is ready!
      if (jiggleStatus == SuctionConstants.PUMP_READY) {
        // So JIGGLE!
        doJiggle();
      }
      // Pump is ready for a reset!
      else if (jiggleStatus == SuctionConstants.PUMP_NEED_RESET) {
        // So RESET IT!
        resetJiggle();
      }
      else if (jiggleStatus == SuctionConstants.PUMP_COMPLETE) {
        stop();
      }
      else {
        // Everything that requires us to do something has been handled
        // so if we land into this block we do nothing.
      }
    }
    // We are not engaged, are we still jiggled position?
    else if (isJiggled) {
      // We are still jiggled.
      // Call stop to clear everything.
      stop();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void grab_cube(){
    robotContainer.ledCubeIndicator.set(false);
    cubeGrabber.set(true);
  }

  public void release_cube(){
    cubeGrabber.set(false);
  }

  public void toggle_cube_light(boolean disabled) {
    robotContainer.ledCubeIndicator.set(disabled);
  }

  public void toggle_cone_light(boolean disabled) {
    robotContainer.ledConeIndicator.set(disabled);
  }

  public void grab_cone() {
    grab_cube();
    // cone grabber is engaged AFTER cube grabber
    coneGrabber.set(false);
  }

  public void release_cone() {
    // cone grabber is released BEFORE cube grabber
    coneGrabber.set(true);

    release_cube();
  }

  public void lock_arm() {
    armLocker.set(Value.kReverse);
  }

  public void unlock_arm() {
    armLocker.set(Value.kForward);
  }

  public void stop() {
    /*
     * Stop subsystem and reset.
     */

     resetJiggle();
     lastJiggleEventTime = 0.0;
     jiggleCount = 0;
     isEngaged = false;
  }

  public void engage_jiggle(boolean jiggleEngage) {
    /*
     * Engages jiggle system. See periodic() for
     * further implementation.
     */

    // If we are telling it to turn on.
    if (jiggleEngage) {
      // Engage the system.
      isEngaged = true;
    }
    else {
      stop();
    }
  }

  public void doJiggle() {
    /*
     * Moves the jiggler.
     */

    lastJiggleEventTime = getCurrentTime();
    coneGrabber.set(true);
    isJiggled = true;
    jiggleCount++;
  }

  public void resetJiggle() {
    /* 
     * Pulls in the pumping solenoid so we can
     * later engage it again.
     */

    lastJiggleEventTime = getCurrentTime();
    coneGrabber.set(false);
    isJiggled = false;
  }


  private double getCurrentTime() {
    /*
     * Returns current time in milliseconds.
     */

    return System.currentTimeMillis();
  }

  public int checkIsJiggleReady() {
    /*
     * Evaluates current state of jiggle system
     * and determines the next action to be performed.
     */

    boolean deltaJiggleTime = (getCurrentTime() - lastJiggleEventTime) > SuctionConstants.PUMP_DELAY_MS;

    // If we are past the delay interval...
    if (deltaJiggleTime) {
      // ... and we are pumped 
      if (isJiggled) {
        // Reset the solenoid.
        return SuctionConstants.PUMP_NEED_RESET;
      }
      // ... we are not pumped and we haven't pumped enough yet.
      else if (jiggleCount < 10) {
        // Pump!
        return SuctionConstants.PUMP_READY;
      }
      // OK we are ready to exit!
      else if (jiggleCount >= 10) {
        // Tell it we're done.
        return SuctionConstants.PUMP_COMPLETE;
      }
      else {
        // ... so do nothing!
        return SuctionConstants.PUMP_WAIT;
      }
    }
    // We are not past the delay interval yet...
    else {
      // ... so do nothing!
      return SuctionConstants.PUMP_WAIT;
    }
  }

  public boolean getIsEngaged() {
    return isEngaged;
  }
}
