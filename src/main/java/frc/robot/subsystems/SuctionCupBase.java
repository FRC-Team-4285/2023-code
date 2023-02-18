package frc.robot.subsystems;

import frc.robot.Constants.SuctionConstants;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SuctionCupBase extends SubsystemBase {

  /** Creates a new SuctionCupBase. */

  private DoubleSolenoid pump_solenoid;

  private int suctionCount = 0;
  private boolean isEngaged = false;
  private double lastPumpEventTime = 0.0;
  private boolean isPumped = false;

  public SuctionCupBase() {
    pump_solenoid = new DoubleSolenoid(
      HardwareCAN.PneumaticHUB, 
      PneumaticsModuleType.REVPH, 
      PneumaticChannels.FORWARD, 
      PneumaticChannels.REVERSE
    );
  }

  public void engageSuctionCup(boolean suctionCupEngage) {
    if (suctionCupEngage) {
      isEngaged = true;
    }
    else {
      stop();
    }
  }

  public void pumpSuctionCup() {
    lastPumpEventTime = getCurrentTime();
    pump_solenoid.set(Value.kForward);
    isPumped = true;
  }

  public void resetPump() {
    lastPumpEventTime = getCurrentTime();
    pump_solenoid.set(Value.kReverse);
    isPumped = false;
  }

  public void stop() {
    /*
     * Stop subsystem and reset.
     */

    resetPump();
    lastPumpEventTime = 0.0;
    isEngaged = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If we are engaged...
    if (isEngaged) {

      // Get the pump status.
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
      else {
        // Everything that requires us to do something has been handled
        // so if we land into this block we do nothing.
      }
    }
    // We are not engaged, are we still pumped up?
    else if (isPumped) {
      // We are still pumped, call stop to clear everything.
      stop();
    }
  }

  private double getCurrentTime() {
    /*
     * Returns current time in miliseconds.
     */

    return System.currentTimeMillis();
  }

  private int checkIsPumpReady() {
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

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation

  }

}