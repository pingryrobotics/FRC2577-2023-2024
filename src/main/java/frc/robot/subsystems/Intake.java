/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.MechanismConstants;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;
	private CANSparkMax intakeFlipMotor;
	private SparkPIDController m_pid;
	private double desiredPos;
	private boolean positionMode = false;
	private double flipSpeed;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Intake(CANSparkMax intakeMotor, CANSparkMax intakeFlipMotor) {
		this.intakeMotor = intakeMotor;
		this.intakeFlipMotor = intakeFlipMotor;

		intakeFlipMotor.restoreFactoryDefaults();
		intakeMotor.restoreFactoryDefaults();

		intakeMotor.setInverted(false);
		intakeFlipMotor.setInverted(true);

		intakeMotor.setIdleMode(IdleMode.kCoast);
		intakeFlipMotor.setIdleMode(IdleMode.kBrake);

		m_pid = intakeFlipMotor.getPIDController();

		m_pid.setP(MechanismConstants.kIntakeP);
		m_pid.setI(MechanismConstants.kIntakeI);
		m_pid.setD(MechanismConstants.kIntakeD);
		m_pid.setIZone(MechanismConstants.kIntakeIZone);
		m_pid.setFF(MechanismConstants.kIntakeFF);
		m_pid.setOutputRange(MechanismConstants.kIntakeMinOutput, MechanismConstants.kIntakeMaxOutput);

		// m_pid.setP(1000);
		// m_pid.setI(10);
		// m_pid.setD(1000);
		// m_pid.setIZone(1000);
		// m_pid.setFF(1000);
		// m_pid.setOutputRange(-1000, 1000);

		positionMode = false;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		double currPos = intakeFlipMotor.getEncoder().getPosition();
		
		SmartDashboard.putNumber("Current Intake Position (rotations)", currPos);
		SmartDashboard.putNumber("Desired Intake Position (rotations)", desiredPos);
		SmartDashboard.putBoolean("intake position mode", positionMode);

		// intakeFlipMotor.getPIDController().setReference(1000, ControlType.kPosition);
		SmartDashboard.putNumber("max output", m_pid.getOutputMax());
		SmartDashboard.putNumber("min output", m_pid.getOutputMin());

		if (positionMode) {
			SmartDashboard.putBoolean("PID worked", m_pid.setReference(desiredPos, ControlType.kPosition).equals(REVLibError.kOk));
		} else {
			intakeFlipMotor.set(flipSpeed);
		}

		SmartDashboard.putNumber("Intake motor speed", intakeFlipMotor.getAppliedOutput());
	}

	public void setSpeed(double speed) {
		intakeMotor.set(speed);
	}

	public void setPosition(double pos) {
		positionMode = true;
		desiredPos = pos;
	}

	public void setFlipSpeed(double speed) {
		positionMode = false;
		desiredPos = intakeFlipMotor.getEncoder().getPosition();
		flipSpeed = speed;
	}

	public void resetEncoder() {
		intakeFlipMotor.getEncoder().setPosition(0);
	}
}