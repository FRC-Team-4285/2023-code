package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveBase;

public class TeleopSwerve extends CommandBase {
  /*
   * Teleoperated Swerve Drive Command
   * ---------------------------------
   * 
   * This command hooks up to the Swerve Drive subsystem
   * and passes in our joystick inputs into it.
   */

  private final SwerveBase drive;

  /*
   * Joysticks return DoubleSuppliers when the get methods are called
   * This is so that joystick getter methods can be passed in as a parameter but
   * will continuously update,
   * versus using a double which would only update when the constructor is called
   */
  private final DoubleSupplier forwardX;
  private final DoubleSupplier forwardY;
  private final DoubleSupplier rotation;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Supplier<Boolean> isFieldOriented;

  public TeleopSwerve(
      SwerveBase subsystem,
      DoubleSupplier fwdX,
      DoubleSupplier fwdY,
      DoubleSupplier rot,
      Supplier<Boolean> isFieldOriented) {

    drive = subsystem;
    forwardX = fwdX;
    forwardY = fwdY;
    rotation = rot;

    this.isFieldOriented = isFieldOriented;

    this.xLimiter = new SlewRateLimiter(Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(Swerve.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    /*
     * Units are given in meters per second radians per second. Since joysticks give output
     * from -1 to 1, we multiply the outputs by the max speed. Otherwise, our max speed
     * would be only 1 meter per second and 1 radian per second.
     */

    double fwdX = forwardX.getAsDouble();
    double fwdY = forwardY.getAsDouble();
    double rot = rotation.getAsDouble();

    // 2. Apply deadband, can edit this later to have smoother behavior
    fwdX = Math.abs(fwdX) > 0.1 ? Math.copySign((1.111*Math.abs(fwdX)-0.111), fwdX) : 0.0; 
    fwdY = Math.abs(fwdY) > 0.1 ? Math.copySign((1.111*Math.abs(fwdY)-0.111), fwdY) : 0.0;
    rot = Math.abs(rot) > 0.1 ? Math.copySign((1.111*Math.abs(rot)-0.111), rot) : 0.0;

    // 3. Make the driving smoother
    fwdX = xLimiter.calculate(fwdX) * Swerve.kTeleDriveMaxSpeedMetersPerSecond;
    fwdY = yLimiter.calculate(fwdY) * Swerve.kTeleDriveMaxSpeedMetersPerSecond;
    rot = turningLimiter.calculate(rot) * Swerve.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    drive.drive(
        -fwdX,
        -fwdY,
        rot,
        isFieldOriented.get()
      );
  }
}
