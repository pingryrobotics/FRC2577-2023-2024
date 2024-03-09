
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
<<<<<<< Updated upstream

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Climber extends SubsystemBase {
	}

	@Override
=======
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
	private DoubleSolenoid clawSolenoid;
	private Compressor compressor;
	private boolean state = false;
	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Claw(DoubleSolenoid clawSolenoid) {
		this.clawSolenoid = clawSolenoid;
		this.isShutoff = true;
		compressor = new Compressor(PneumaticsModuleType.CTREPCM);
		this.clawSolenoid.set(DoubleSolenoid.Value.kForward);
		// compressor.disable();
		// this.setDefaultCommand(new ClawStop(this));
		SmartDashboard.putBoolean("Has closed", false);
		
	}
	@Override

>>>>>>> Stashed changes
	public void periodic() {
		// This method will be called once per scheduler run
	}

<<<<<<< Updated upstream
	public void setSpeed(double speed) {
        climberLeft.set(speed);
        climberRight.set(speed);
=======
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
		state = false;
>>>>>>> Stashed changes
	}
}