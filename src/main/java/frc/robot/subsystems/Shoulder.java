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
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
    private CANSparkMax shoulderMotor;
	private double speed = 0;
	private double desiredPosition = 0;
	private boolean positionMode = false;
    private SparkPIDController m_pid;
//    private boolean pidMode = false;

	/**
	 * Creates a new ExampleSubsystem.
	 */
	public Shoulder(CANSparkMax shoulderMotor) {
		this.shoulderMotor = shoulderMotor;
        this.shoulderMotor.setInverted(true);
        this.m_pid = shoulderMotor.getPIDController();
        m_pid.setP(Constants.MechanismConstants.kShoulderP);
        m_pid.setI(Constants.MechanismConstants.kShoulderI);
        m_pid.setD(Constants.MechanismConstants.kShoulderD);
        m_pid.setIZone(Constants.MechanismConstants.kShoulderIZone);
        m_pid.setFF(Constants.MechanismConstants.kShoulderFF);
        m_pid.setOutputRange(Constants.MechanismConstants.kShoulderMinOutput, Constants.MechanismConstants.kShoulderMaxOutput);
        shoulderMotor.setIdleMode(IdleMode.kBrake);
	}

	@Override
    public void periodic() {
        double shoulderPos = shoulderMotor.getEncoder().getPosition();

        SmartDashboard.putNumber("Shoulder Position (rotations)", shoulderPos);
        SmartDashboard.putNumber("Desired Shoulder Rotation (rotations)", desiredPosition);
        SmartDashboard.putNumber("Shoulder speed", speed);
        // This method will be called once per scheduler run

        // stop from going too far
        // if (shoulderPos < Constants.MechanismConstants.kMinShoulderRotation
        //         || shoulderPos > Constants.MechanismConstants.kMaxShoulderRotation) {
        //     shoulderMotor.set(0);
        //     return;
        // }

        if (positionMode) {
            m_pid.setReference(desiredPosition, ControlType.kPosition);
            // if (Math.abs(shoulderPos - desiredPosition) < Constants.MechanismConstants.kShoulderPositionTolerance)
                // shoulderMotor.set(0);
            // else if (shoulderPos > desiredPosition)
            //     shoulderMotor.set(-Constants.MechanismConstants.kShoulderSpeed);
            // else if (shoulderPos < desiredPosition)
            //     shoulderMotor.set(Constants.MechanismConstants.kShoulderSpeed);
        } else {
            
            // m_pid.setReference(shoulderPos+(10*speed), ControlType.kPosition);
            
            shoulderMotor.set(speed);
            
        }
    }
    
    public void moveShoulderDirection(double direction) {
        shoulderMotor.set(direction * Constants.MechanismConstants.kShoulderSpeed);
        positionMode = false;
    }

    public void setDesiredTicks(double desiredPosition) {
        this.desiredPosition = desiredPosition;
        positionMode = true;
    }

	public double getShoulderPosition() {
		return shoulderMotor.getEncoder().getPosition();
	}

    public void setShoulderSpeed(double speed) {
        this.speed = speed;
        positionMode = false;
    }

    public void stop() {
        shoulderMotor.set(0);
        desiredPosition = shoulderMotor.getEncoder().getPosition();
        positionMode = true;
    }

    public void resetEncoder() {
        shoulderMotor.getEncoder().setPosition(0);
    }
}