package frc.robot.subsystems;

import frc.robot.Constants.MechanismConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {
    Spark blinkin;

    public Blinkin() {
        blinkin = new Spark(Constants.LightsPWM));
    }
}
