/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.MechanismConstants;;

public class Shooter extends SubsystemBase {
	private CANSparkMax shooterLeft;
	private CANSparkMax shooterRight;
	private CANSparkMax shooterAdjusterLeft;
	private CANSparkMax shooterAdjusterRight;
	private SparkPIDController m_pidLeft;
	private SparkPIDController m_pidRight;
	private double desiredPos;
	private boolean positionMode = false;
	private double shooterSpeed = 0;
	private double adjusterSpeed = 0;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Shooter(CANSparkMax shooterLeft, CANSparkMax shooterRight, CANSparkMax shooterAdjusterLeft,
			CANSparkMax shooterAdjusterRight) {
		this.shooterLeft = shooterLeft;
		this.shooterRight = shooterRight;
		this.shooterAdjusterLeft = shooterAdjusterLeft;
		this.shooterAdjusterRight = shooterAdjusterRight;
		this.shooterLeft.setInverted(true);
		this.shooterRight.setInverted(false);
		this.shooterAdjusterLeft.setInverted(true);

		this.shooterLeft.setIdleMode(IdleMode.kCoast);
		this.shooterRight.setIdleMode(IdleMode.kCoast);

		this.shooterAdjusterLeft.setIdleMode(IdleMode.kBrake);
		this.shooterAdjusterRight.setIdleMode(IdleMode.kBrake);

		this.m_pidLeft = shooterAdjusterLeft.getPIDController();

		m_pidLeft.setP(MechanismConstants.kShooterAdjusterP);
		m_pidLeft.setI(MechanismConstants.kShooterAdjusterI);
		m_pidLeft.setD(MechanismConstants.kShooterAdjusterD);
		m_pidLeft.setIZone(MechanismConstants.kShooterAdjusterIZone);
		m_pidLeft.setFF(MechanismConstants.kShooterAdjusterFF);
		m_pidLeft.setOutputRange(MechanismConstants.kShooterAdjusterMinOutput, MechanismConstants.kShooterAdjusterMaxOutput);

		this.m_pidRight = shooterAdjusterLeft.getPIDController();

		m_pidRight.setP(MechanismConstants.kShooterAdjusterP);
		m_pidRight.setI(MechanismConstants.kShooterAdjusterI);
		m_pidRight.setD(MechanismConstants.kShooterAdjusterD);
		m_pidRight.setIZone(MechanismConstants.kShooterAdjusterIZone);
		m_pidRight.setFF(MechanismConstants.kShooterAdjusterFF);
		m_pidRight.setOutputRange(MechanismConstants.kShooterAdjusterMinOutput, MechanismConstants.kShooterAdjusterMaxOutput);
	}
	

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		shooterLeft.set(shooterSpeed);
		shooterRight.set(shooterSpeed);

		double adjusterPosLeft = shooterAdjusterLeft.getEncoder().getPosition();
		double adjusterPosRight = shooterAdjusterRight.getEncoder().getPosition();

		SmartDashboard.putNumber("Current Shooter Position Left (rotations)", adjusterPosLeft);
		SmartDashboard.putNumber("Current Shooter Position Right (rotations)", adjusterPosRight);
		SmartDashboard.putNumber("Desired Shooter Position (rotations)", desiredPos);
		
		if (positionMode) {
			m_pidLeft.setReference(desiredPos, ControlType.kPosition);
			m_pidRight.setReference(desiredPos, ControlType.kPosition);
		} else {
			shooterAdjusterLeft.set(adjusterSpeed);
			shooterAdjusterRight.set(adjusterSpeed);
		}
	}

	public void setPosition(double position) {
		positionMode = true;
		desiredPos = position;
	}

	public void setSpeed(double speed) {
		this.shooterSpeed = speed;
	}

	public void setAdjusterSpeed(double speed) {
		positionMode = false;
		this.adjusterSpeed = speed;
	}

	public void adjusterOff() {
		setAdjusterSpeed(0);
	}

	public void resetEncoder() {
		shooterAdjusterLeft.getEncoder().setPosition(0);
		shooterAdjusterRight.getEncoder().setPosition(0);
	}

	public double getPosition() {
		return (shooterAdjusterLeft.getEncoder().getPosition() + shooterAdjusterRight.getEncoder().getPosition()) / 2;
	}
}