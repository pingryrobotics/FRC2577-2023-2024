/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// commented because i don't like seeing yellow dots on my sidebar - christian
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private CANSparkMax armMotor;
	private double speed = 0;
	private double desiredPosition = 0;
	private boolean positionMode = false;
    private SparkPIDController m_pid;
    private boolean armLimitEnabled = true;

//    private boolean pidMode = false;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Arm(CANSparkMax armMotor) {
		this.armMotor = armMotor;
        this.m_pid = armMotor.getPIDController();
        m_pid.setP(Constants.MechanismConstants.kArmP);
        m_pid.setI(Constants.MechanismConstants.kArmI);
        m_pid.setD(Constants.MechanismConstants.kArmD);
        m_pid.setIZone(Constants.MechanismConstants.kArmIZone);
        m_pid.setFF(Constants.MechanismConstants.kArmFF);
        m_pid.setOutputRange(Constants.MechanismConstants.kArmMinOutput, Constants.MechanismConstants.kArmMaxOutput);
        armMotor.setIdleMode(IdleMode.kBrake);
	}

	@Override
    public void periodic() {
        // This method will be called once per scheduler run
		double armPos = armMotor.getEncoder().getPosition();

        SmartDashboard.putNumber("Arm Position (rotations)", armPos);
        SmartDashboard.putNumber("Desired Arm Rotation (rotations)", desiredPosition);
        SmartDashboard.putNumber("Arm speed", speed);

        // armMotor.enableSoftLimit(SoftLimitDirection.kForward, armLimitEnabled);
        // armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.MechanismConstants.kMaxArmRetraction);

        // stop from going too farn
        if (armLimitEnabled) {
            // extending decreases ticks
            // if (armPos >= Constants.MechanismConstants.kMaxArmRetraction) {
            //     if (speed > 0) {
            //         armMotor.set(0);
            //         return;
            //     }
            // } 
        //     // else if (armPos <= Constants.MechanismConstants.kMaxArmExtension) {
        //     //     if (speed < 0) {
        //     //         armMotor.set(0);
        //     //         return;
        //     //     }
        //     // }
        }

        // run normally
        if (positionMode) {
            m_pid.setReference(desiredPosition, ControlType.kPosition);
            // if (Math.abs(armPos - desiredPosition) < Constants.MechanismConstants.karmPositionTolerance)
                // armMotor.set(0);
            // else if (armPos > desiredPosition)
            //     armMotor.set(-Constants.MechanismConstants.karmSpeed);
            // else if (armPos < desiredPosition)
            //     armMotor.set(Constants.MechanismConstants.karmSpeed);
        } else {
            armMotor.set(speed);
        }
    }

    public void toggleArmLimit() {
        armLimitEnabled = !armLimitEnabled;
    }
    
    public void moveArmDirection(double direction) {
        speed = direction * Constants.MechanismConstants.kArmSpeed;
        positionMode = false;
    }

    public void resetEncoder() {
        armMotor.getEncoder().setPosition(0);
    }

    public void setDesiredTicks(double desiredPosition) {
        this.desiredPosition = desiredPosition;
        positionMode = true;
    }

	public double getArmPosition() {
		return armMotor.getEncoder().getPosition();
	}

    public void setArmSpeed(double speed) {
        this.speed = speed;
        positionMode = false;
    }

    public void stop() {
        // armMotor.set(0);
        speed = 0;
        positionMode = false;
        // desiredPosition = armMotor.getEncoder().getPosition();
        // positionMode = true;
    }

    // public void enableLimit() {
    //     armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    //     armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)armMotor.getEncoder().getPosition());
    // }

    // public void toggleLimit(boolean isOn) {
    //     armMotor.enableSoftLimit(SoftLimitDirection.kForward, isOn);
    // }
}