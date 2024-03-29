// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class MechanismConstants { // remove the "FINE TUNE VALUE" comments as you tweak them
    public static final double kIntakeInSpeed = 0.45; // FINE TUNE VALUE
    public static final double kIntakeOutSpeed = 0.45;
    public static final double kIntakeFlipUpSpeed = 0.1; // FINE TUNE VALUE
    public static final double kIntakeFlipDownSpeed = 0.1; // FINE TUNE VALUE
    public static final double kShooterSpeed = 0.65; // FINE TUNE VALUE
    public static final double kShooterAutoSpeed = 0.7;
    // public static final double kShooterAdjusterSpeed = 0.4;
    // public static final double kShooterAdjusterAutoSpeed = 0.8;
    public static final double kRampInSpeed = 0.3; // FINE TUNE values
    public static final double kRampOutSpeed = 0.3; // FINE TUNE values
    public static final double kIndexerSpeed = -0.5; // FINE TUNE
    public static final double kSlowdownMultiplier = 0.5;
    public static final double kClimberSpeed = 0.15;

    public static final double kIntakeDown = -0.25; // FINE TUNE VALUE 21.3
    public static final double kIntakeUp = 0;

    // public static final double kShooterAdjusterLow = 0; // FINE TUNE VALUE
    // public static final double kShooterAdjusterMid = 0; // FINE TUNE VALUE
    // public static final double kShooterAdjusterHigh = 140; // FINE TUNE VALUE
    
    // PID values for intake flipper
    public static final double kIntakeP = 1; // FINE TUNE VALUE; increase if not moving, decrease if oscillating
    public static final double kIntakeI = 1e-4; // FINE TUNE VALUE; should be around 0.1x P and makes the movement 
                                              // less smooth but faster
    public static final double kIntakeD = 0; // FINE TUNE VALUE; should be around 10x P
    public static final double kIntakeFF = 0.0; // FINE TUNE VALUE
    public static final int kIntakeIZone = 0; // FINE TUNE VALUE
    public static final double kIntakeMinOutput = -0.05; // FINE TUNE VALUE
    public static final double kIntakeMaxOutput = 0.08; // FINE TUNE VALUE


    // PID values for shooter adjuster
    // public static final double kShooterAdjusterP = 0.1; // FINE TUNE VALUE; increase if not moving, decrease if oscillating
    // public static final double kShooterAdjusterI = 1e-4; // FINE TUNE VALUE; should be around 0.1x P and makes the movement 
    //                                           // less smooth but faster
    // public static final double kShooterAdjusterD = 1; // FINE TUNE VALUE; should be around 10x P
    // public static final double kShooterAdjusterFF = 0.0; // FINE TUNE VALUE
    // public static final int kShooterAdjusterIZone = 0; // FINE TUNE VALUE
    // public static final double kShooterAdjusterMinOutput = -kShooterAdjusterSpeed; // FINE TUNE VALUE
    // public static final double kShooterAdjusterMaxOutput = kShooterAdjusterSpeed; // FINE TUNE VALUE
    public static final int fullyLoadedRpm = 3700; ///random ahh number FINE TUNE AT COMP YEssSSIR
    public static final int kRampID = 51;
    public static final int kShooterLeftID = 31; // check
    public static final int kShooterRightID = 30; // check
    // public static final int kShooterAdjusterLeftID = 61;
    // public static final int kShooterAdjusterRightID = 60;
    public static final int kIndexerID = 50;
    public static final int kIntakeID = 40;
    public static final int kIntakeFlipID = 41;
    public static final int kLeftClimberID = 54;
    public static final int kRightClimberID = 53; 

    //Blinkin Port
     public static final int LightsPWM = 0;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 7.62;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
    public static final double kSlowModeMagnitudeSlewRate = kMagnitudeSlewRate/2; // percent per second (1 = 100%)
    public static final double kSlowModeRotationalSlewRate = kRotationalSlewRate/2; // percent per second (1 = 100%)

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 12;
    public static final int kRearLeftDrivingCanId = 11;
    public static final int kFrontRightDrivingCanId = 14;
    public static final int kRearRightDrivingCanId = 13;

    public static final int kFrontLeftTurningCanId = 22;
    public static final int kRearLeftTurningCanId = 21;
    public static final int kFrontRightTurningCanId = 24;
    public static final int kRearRightTurningCanId = 23;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 16;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22(new is 19) teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 19) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final CANSparkMax.IdleMode kDrivingMotorIdleMode = CANSparkMax.IdleMode.kBrake;
    public static final CANSparkMax.IdleMode kTurningMotorIdleMode = CANSparkMax.IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
}
