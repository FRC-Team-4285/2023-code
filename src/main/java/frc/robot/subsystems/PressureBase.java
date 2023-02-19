package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PressureBase extends SubsystemBase{
    private Compressor andy;
    private PneumaticHub REV_PH;
    /*
     * tank pressure getter, compressor state getter
     * make this subsystem disallow commands that use air if tank pressure <60psi
     * Battery Leveling PID (???)
     */

    public PressureBase(){
        andy = new Compressor(PneumaticsModuleType.REVPH);
        REV_PH = new PneumaticHub(1);
    } 

    public double getPressure(){
        return REV_PH.getPressure(0);
    }

    public void runCompressor(){
        andy.enableAnalog(90, 120);
    }
}
