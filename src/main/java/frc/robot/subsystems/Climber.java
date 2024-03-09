
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final DoubleSolenoid clawSolenoid;
    private final Compressor compressor;
    private final boolean state = false;

    /**
     * Creates a new ExampleSubsystem.
     */
    public Climber(DoubleSolenoid clawSolenoid) {
        this.clawSolenoid = clawSolenoid;
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        this.clawSolenoid.set(DoubleSolenoid.Value.kReverse);
        SmartDashboard.putBoolean("Has closed", false);

    }

    @Override

    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }


    public void up() {
        // singleSolenoid.set(true);
        // clawSolenoid.toggle();
        clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void down() {
        clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public Command upCommand() {
        return this.runOnce(this::up);
    }

    public Command downCommand() {
        return this.runOnce(this::down);
    }
}