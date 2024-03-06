package frc.robot.subsystems;

import frc.robot.Constants.MechanismConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {
    Spark blinkin;

    public Blinkin() {
        blinkin = new Spark(MechanismConstants.LightsPWM);
    }

    public void LightsLoading() {
        blinkin.set(-0.45);
    }

    public void LightsBlue(){
        blinkin.set(-0.75);
    }

    public void LightsRed(){
        blinkin.set(-.073);
    }

    public void HumanPlayer(){
        blinkin.set(-0.99);
    }

    public void ShooterReady(){
        blinkin.set(0.77);
    }

    public void ShootRed(){
        blinkin.set(-0.11);
    }

    public void ShootBlue(){
        blinkin.set(-0.09);
    }
}
