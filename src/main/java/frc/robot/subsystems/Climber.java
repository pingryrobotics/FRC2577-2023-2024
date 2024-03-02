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
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController;

public class Climber extends SubsystemBase {
    private CANSparkMax climberLeft;
    private CANSparkMax climberRight;

	/**
	 * Creates a new ExampleSubsystem.
	 */
    public Climber(CANSparkMax left, CANSparkMax right) {
        this.climberLeft = left;
        this.climberRight = right;
        climberLeft.setInverted(true);
        climberRight.setInverted(false);
		climberLeft.setIdleMode(IdleMode.kBrake);
        climberRight.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void setSpeed(double speed) {
        climberLeft.set(speed);
        climberRight.set(speed);
	}
}