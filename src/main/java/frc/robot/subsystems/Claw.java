/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// DEPRECATED - DOESN'T USE SOLENOIDS
// REWRITE TO USE SOLENOIDS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

public class Claw extends SubsystemBase {
    private DoubleSolenoid clawSolenoid;
	// private Solenoid singleSolenoid;
	private CANSparkMax wheelsMotor;
	private boolean state = false;
	private Compressor compressor;
	public ColorSensorV3 colorSensor;
	public boolean autoClaw = false;
	public boolean objectExists = false;
	public boolean objectExisted = false;
	// private int cnt = 0;
	public double speed = 0;
	public int disconnectedCount = 0;
	public double prox = -1;
	public boolean isConnected = true;
	public boolean hasReset = false;
	public boolean isShutoff = false;

    
	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Claw(CANSparkMax wheelsMotor, DoubleSolenoid clawSolenoid) {
		this.wheelsMotor = wheelsMotor;
		wheelsMotor.setIdleMode(IdleMode.kBrake);
		// wheelsMotor.setVoltage(6);
		this.clawSolenoid = clawSolenoid;
		this.isShutoff = true;
		this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
		colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate400ms);
		// distanceSensor.setAutomaticMode(true);
		// this.clawSolenoid.set(DoubleSolenoid.Value.kOff);
		// this.singleSolenoid = solenoid;
		// solenoid.set(true);
		compressor = new Compressor(PneumaticsModuleType.CTREPCM);
		this.clawSolenoid.set(DoubleSolenoid.Value.kForward);
		// compressor.disable();
		// this.setDefaultCommand(new ClawStop(this));
		SmartDashboard.putBoolean("Has closed", false);
		
	}

	// public void enableAutomatic() {
	// 	colorSensor.setAutomaticMode(true);
	// }

	@Override
	public void periodic() {
		wheelsMotorActivate();
		
		
		SmartDashboard.putBoolean("CS Connected", isConnected);
		SmartDashboard.putBoolean("AutoClaw (tm) On", autoClaw);
		SmartDashboard.putBoolean("Claw Is Open", state);
		SmartDashboard.putBoolean("Autoclaw is shutoff", this.isShutoff);
		if (isShutoff) {
			return;
		} else if (!isConnected && !colorSensor.isConnected()) {
			SmartDashboard.putBoolean("Autoclaw is shutoff", this.isShutoff);
			SmartDashboard.putNumber("Distance Sensor Range", colorSensor.getProximity());
			SmartDashboard.putBoolean("CS Active", colorSensor.isConnected());
			colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
			colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate400ms);
			isShutoff = true;
			return;
		}
		// } else if (isShutoff) {
		// 	return;
		// }

		prox = colorSensor.getProximity();
		SmartDashboard.putBoolean("CS Active", colorSensor.isConnected());
		SmartDashboard.putNumber("Distance Sensor Range", prox);
		
		

		if(isConnected && !colorSensor.isConnected()){
			isConnected = false;
			// prox = colorSensor.getProximity();
			colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
			colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate400ms);
			System.out.println("********Color sensor disabled********");
		}

		

		if(!isConnected && colorSensor.isConnected()) {
			colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
			colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate400ms);
			isConnected = true;
			hasReset = true;
			objectExists = false;
			System.out.println("********Color sensor reset********");
		}

		if (isConnected) {
			// prox = colorSensor.getProximity();
			// if(colorSensor.getProximity() == 0) {
			// 	colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
			// 	colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate400ms);
			// 	// isConnected = true;

			// }

		// distanceSensor =  new ColorSensorV3(ColorSensorV3.Port.kOnboard, Unit.kInches, RangeProfile.kHighSpeed);
		// SmartDashboard.putNumber("Color Sensor Timestamp", colorSensor.getBlue());
		
		

		// if (!colorSensor.isConnected() && (disconnectedCount < 5 || disconnectedCount % 500 == 0)) {
		// 	colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
		// 	disconnectedCount += 1;
		// } else {
		// 	disconnectedCount = 0;
		// }
		
		
		// double prox = colorSensor.getProximity();
		// if (prox == 0 && colorSensor.getBlue() == 0 && colorSensor.getRed() == 0) {
			// colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
			// colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes9bit, ProximitySensorMeasurementRate.kProxRate100ms);
		// }
		// SmartDashboard.putNumber("Distance Sensor Range", prox);
		// SmartDashboard.putNumber("blue", colorSensor.getBlue());

		// if (System.nanoTime() % 10 == 0) {
			if (autoClaw && colorSensor.isConnected()) {

			 	if (prox != -1 && prox > 130) {
				objectExisted = objectExists;
				objectExists = true;
				// this.close();
				}
			} else {
				objectExisted = objectExists;
				objectExists = false;
			}
		// }
		}
		// SmartDashboard.putBoolean("Existed", objectExisted);
		// SmartDashboard.putBoolean("Exists", objectExists);

		// This method will be called once per scheduler run
	}

	public void wheelsMotorActivate() {
		wheelsMotor.set(-1 * speed);
	}

	public void setWheelsSpeed(double speed) {
		this.speed = speed;
	}

	public void toggleAutoClaw() {
		autoClaw = !autoClaw;
	}

	public void autoClawOn() {
		autoClaw = true;
	}

	public void autoClawOff() {
		autoClaw = false;
	}

	public void reinitializeSensor() {
		colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
		colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate400ms);
		isConnected = true;
	}
	public void enableCompressor() {
		compressor.enableDigital();
	}

	public void disableCompressor() {
		compressor.disable();
	}

	public void disableShutoff() {
		System.out.println("*******Shutoff disabled******");
		this.isShutoff = false;
		isConnected = true;
		colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
		colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate400ms);
		SmartDashboard.putNumber("Distance Sensor Range", colorSensor.getProximity());
		SmartDashboard.putBoolean("CS Active", colorSensor.isConnected());
	}

	public void enableShutoff() {
		System.out.println("*******Shutoff enabled******");
		this.isShutoff = true;
		isConnected = false;
		// colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
		// colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate400ms);
		SmartDashboard.putNumber("Distance Sensor Range", colorSensor.getProximity());
		SmartDashboard.putBoolean("CS Active", colorSensor.isConnected());
	}


	// public void stop() {
	// 	clawSolenoid.set(DoubleSolenoid.Value.kOff);
	// }
	public void close() {
		// singleSolenoid.set(true);
		// clawSolenoid.toggle();
		clawSolenoid.set(DoubleSolenoid.Value.kForward);
		state = false;
	}

	public void toggleClawState() {
		state = !state;
		clawSolenoid.toggle();
	}

	public void open() {
		// singleSolenoid.set(true);
		// clawSolenoid.toggle();
		clawSolenoid.set(DoubleSolenoid.Value.kReverse);
		state = true;
	}

	// DoubleSolenoid.toggle() is unreliable since it handles toggling away from kOff badly
	// public void toggle() {
	// 	if(state) close();
	// 	else open();
	// }
}