package frc.robot.subsystems;

import frc.robot.Constants.SuctionConstants;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SuctionCupBase extends SubsystemBase {
  /*
   * ---------------------
   * Suction Cup Subsystem
   * ---------------------
   * 
   * This class is implements the suction cup system.
   * 
   * Performs the following actions:
   *  - Monitors the state of the suction cup and autonomously
   *    actuates the suction cup button by engaging and releasing
   *    a piston via a solenoid. 
   */

  private Solenoid pump_solenoid;
  private DoubleSolenoid release_solenoid;

  private boolean isEngaged = false;
  private boolean isPumped = false;
  private double lastPumpEventTime = 0.0;
  private int suctionCount = 0;

  public SuctionCupBase() {
    // This solenoid pumps the suction cup,
    // holding us to the balance station.
    pump_solenoid = new Solenoid(
      HardwareCAN.PNEUMATIC_HUB,
      PneumaticsModuleType.REVPH,
      PneumaticChannels.CUP_PUMPER
    );

    // This solenoid releases the suction cup
    // disconnecting us from the balance station.
    release_solenoid = new DoubleSolenoid(
      HardwareCAN.PNEUMATIC_HUB, 
      PneumaticsModuleType.REVPH, 
      PneumaticChannels.CUP_RELEASE_OFF,
      PneumaticChannels.CUP_RELEASE_ON
    );
  }

  public void engageSuctionCup(boolean suctionCupEngage) {
    /*
     * Engages suction cup system. See periodic() for
     * further implementation.
     */

    // If we are telling it to turn on.
    if (suctionCupEngage) {
      // Engage the system.
      release_solenoid.set(DoubleSolenoid.Value.kForward);
      isEngaged = true;
    }
    else {
      stop();
      release_solenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void pumpSuctionCup() {
    /* 
     * Pushes out the pumping solenoid,
     * pumping the suction cup.
     */

    lastPumpEventTime = getCurrentTime();
    pump_solenoid.set(true);
    isPumped = true;
    suctionCount++;
  }

  public void resetPump() {
    /* 
     * Pulls in the pumping solenoid so we can
     * later engage it again.
     */

    lastPumpEventTime = getCurrentTime();
    pump_solenoid.set(false);
    isPumped = false;
  }

  public void stop() {
    /*
     * Stop subsystem and reset.
     */

    resetPump();
    lastPumpEventTime = 0.0;
    suctionCount = 0;
    isEngaged = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If we are engaged...
    if (isEngaged) {

      // Get pump status.
      int pumpStatus = checkIsPumpReady();

      // Pump is ready!
      if (pumpStatus == SuctionConstants.PUMP_READY) {
        // So PUMP!
        pumpSuctionCup();
      }
      // Pump is ready for a reset!
      else if (pumpStatus == SuctionConstants.PUMP_NEED_RESET) {
        // So RESET IT!
        resetPump();
      }
      else if (pumpStatus == SuctionConstants.PUMP_COMPLETE) {
        stop();
      }
      else {
        // Everything that requires us to do something has been handled
        // so if we land into this block we do nothing.
      }
    }
    // We are not engaged, are we still pumped up?
    else if (isPumped) {
      // We are still pumped.
      // Call stop to clear everything.
      stop();
    }
  }

  private double getCurrentTime() {
    /*
     * Returns current time in milliseconds.
     */

    return System.currentTimeMillis();
  }

  private int checkIsPumpReady() {
    /*
     * Evaluates current state of suction cup system
     * and determines the next action to be performed.
     * 
     * Responses (See Constants.SuctionConstants): 
     *    PUMP_READY ........ Suction cup is ready to be pumped again.
     *    PUMP_NEED_RESET ... Piston is in the pumped state and needs
     *                        to be reset before we can pump again.
     *    PUMP_WAIT ......... We are in a state where we should do nothing.
     *                        This will happen when either:
     *                          - We have pumped the suction cup 10 times.
     *                          - We have just performed a pump action (pump/reset)
     *                            and need to wait a few moments to allow the mechanical
     *                            system to catch up with us.
     *    PUMP_COMPLETE ..... Pump has finished pumping and the command is able to
     *                        successfully exit and cleanup.
     */

    boolean deltaPumpTime = (getCurrentTime() - lastPumpEventTime) > SuctionConstants.PUMP_DELAY_MS;

    // If we are past the delay interval...
    if (deltaPumpTime) {
      // ... and we are pumped 
      if (isPumped) {
        // Reset the solenoid.
        return SuctionConstants.PUMP_NEED_RESET;
      }
      // ... we are not pumped and we haven't pumped enough yet.
      else if (suctionCount < 10) {
        // Pump!
        return SuctionConstants.PUMP_READY;
      }
      // OK we are ready to exit!
      else if (suctionCount >= 10) {
        // Tell it we're done.
        return SuctionConstants.PUMP_COMPLETE;
      }
      // ... we are not pumped and we have did all the pumping
      // we need to do.
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
