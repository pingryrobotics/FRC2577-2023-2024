/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.MechanismConstants;

public class Indexer extends SubsystemBase {
	private CANSparkMax indexerMotor;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Indexer(CANSparkMax indexerMotor) {
		this.indexerMotor = indexerMotor;
		
		this.indexerMotor.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void indexerOn() {
		indexerMotor.set(MechanismConstants.kIndexerSpeed);
	}

	public void indexerOff() {
		indexerMotor.set(0);
	}

	public void indexerReverse() {
		indexerMotor.set(-MechanismConstants.kIndexerSpeed);
	}
}