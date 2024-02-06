// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// commented because i don't like seeing yellow dots on my sidebar - christian
//import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MechanismConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autos.AutoBalanceAuto;
import frc.robot.commands.autos.DoNothingAuto;
import frc.robot.commands.autos.MoveForwardAuto;
import frc.robot.commands.autos.OnePieceParkAuto;
import frc.robot.commands.autos.PlaceParkAuto;
import frc.robot.commands.arm_commands.*;
import frc.robot.commands.claw_commands.*;
import frc.robot.commands.drive_commands.DriveX;
import frc.robot.commands.grouped_commands.SequentialCommands;
// import frc.robot.commands.drive_commands.*;
import frc.robot.commands.shoulder_commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shoulder;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/*
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

    // The robot's subsystems
    // private final Drive m_robotDrive = new Drive();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Arm m_arm = new Arm(new CANSparkMax(MechanismConstants.kArmID, MotorType.kBrushless));
    // private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final DoubleSolenoid m_DoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private final Claw m_claw = new Claw(new CANSparkMax(MechanismConstants.kClawID, MotorType.kBrushless), m_DoubleSolenoid);
    // private final Claw m_claw = new Claw(m_DoubleSolenoid, new ColorSensorV3(I2C.Port.kOnboard));
    // new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard));
    // , Unit.kInches, RangeProfile.kHighAccuracy));
    private final Shoulder m_shoulder = new Shoulder(new CANSparkMax(MechanismConstants.kShoulderID, MotorType.kBrushless));



    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    // Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    // Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    SendableChooser<Integer> side_chooser = new SendableChooser<>();

    private boolean leftJoystickPressed = false;
    private boolean rightJoystickPressed = false;
    private boolean driveOn = false;
    private boolean wheelsWereOn = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setFPS(30);
        camera.setResolution(256, 144);
        // CameraServer.
        
        // CameraServer.

        side_chooser.setDefaultOption("Red Left", 0);
        side_chooser.addOption("Red Center", 1);
        side_chooser.addOption("Red Right", 2);
        side_chooser.addOption("Blue Left", 3);
        side_chooser.addOption("Blue Center", 4);
        side_chooser.addOption("Blue Right", 5);

        SmartDashboard.putData("Side", side_chooser);
        
                // Configure default commands
        // m_robotDrive.setDefaultCommand(
        //     // The left stick controls translation of the robot.
        //     // Turning is controlled by the X axis of the right stick.
            
        //     new RunCommand(
        //         () -> m_robotDrive.drive(
        //             -MathUtil.applyDeadband(m_driverJoystick.getY() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //             -MathUtil.applyDeadband(m_driverJoystick.getX() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1), OIConstants.kDriveDeadband),
        //             -MathUtil.applyDeadband(m_driverJoystick.getTwist() * 0.7 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.3 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //             false, true),
        //             m_robotDrive));
        // m_robotDrive.setDefaultCommand(
        //     // The left stick controls translation of the robot.
        //     // Turning is controlled by the X axis of the right stick.
            
        //     // new RunCommand(
        //     //     () -> m_robotDrive.drive(
        //     //         -MathUtil.applyDeadband(m_driverController.getLeftY() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //     //         -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.65 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //     //         -MathUtil.applyDeadband(m_driverController.getRightX() * 0.7 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.6 : 1), OIConstants.kDriveDeadband),
        //     //         false, true),
        //     //         m_robotDrive));
        //     new RunCommand(
        //             () -> m_robotDrive.drive(
        //                 -MathUtil.applyDeadband(m_driverController.getLeftY() * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //                 -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.65 * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //                 -MathUtil.applyDeadband(m_driverController.getRightX() * 0.7 * (m_robotDrive.m_slowMode ? 0.6 : 1), OIConstants.kDriveDeadband),
        //                 true, false),
        //                 m_robotDrive));
            
        
        // // Add commands to Autonomous Sendable Chooser
        m_chooser.setDefaultOption("Do Nothing", new DoNothingAuto());
        m_chooser.addOption("New Place Auto", new PlaceParkAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true, false, false, false));
        m_chooser.addOption("New Place Park Auto", new PlaceParkAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true, false, true, false));
        m_chooser.addOption("New Place Balance Auto", new PlaceParkAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true, true, false, false));
        m_chooser.addOption("New New Place Balance Auto", new PlaceParkAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true, false, false, true));
        m_chooser.addOption("New New Balance Auto", new PlaceParkAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, false, false, false, true));



        m_chooser.addOption("One Piece Park Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, true, true));
        m_chooser.addOption("One Piece Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, true, false));
        m_chooser.addOption("Park Auto", new OnePieceParkAuto(m_robotDrive, m_arm, m_claw, m_shoulder, side_chooser, false, true));
        m_chooser.addOption("Move Forward Auto", new MoveForwardAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, false, true));
        m_chooser.addOption("Place and Move Forward Auto", new MoveForwardAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true, true));
        m_chooser.addOption("Place Auto", new MoveForwardAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true, false));
        m_chooser.addOption("Auto Balance Auto", new AutoBalanceAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, false));
        m_chooser.addOption("Place and Auto Balance Auto", new AutoBalanceAuto(m_robotDrive, m_claw, m_shoulder, m_arm, side_chooser, true));

        
        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto mode", m_chooser);
    }

    public void resetEncodersAndGyro() {
        m_arm.resetEncoder();
        m_shoulder.resetEncoder();
        m_robotDrive.zeroHeading();
    }

    public void resetGyro() {
        m_robotDrive.zeroHeading();
    }

    // public void calibrateGyro() { // TODO: does this work??
    //     m_robotDrive.m_gyro.zeroYaw();
    //     m_robotDrive.m_gyro.reset();
    //     m_robotDrive.m_gyro.resetDisplacement();
    // }

    

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /*
        DRIVER CONTROLLER
        */
        // m_driverController.x().onTrue(
        //     new DriveX(m_robotDrive, true));
        //     m_driverController.x().onFalse(
        //         new DriveX(m_robotDrive, false));
          
        m_operatorController.a().onTrue(SequentialCommands.fullyRetract(m_robotDrive, m_claw, m_shoulder, m_arm));
        m_operatorController.y().onTrue(SequentialCommands.fullyExtend(m_robotDrive, m_claw, m_shoulder, m_arm));
        m_operatorController.x().onTrue(new ArmPID(m_arm, 0));
        m_operatorController.b().onTrue(new ArmPID(m_arm, Constants.MechanismConstants.kArmFullExtension));
        

        // SHOULDER COMMANDS

        // move shoulder to level 3
        m_operatorController.pov(0).onTrue(new ShoulderToHigh(m_shoulder));
        // move shoulder to level 2
        m_operatorController.pov(90).onTrue(new ShoulderToMid(m_shoulder));
        // move shoulder to level 1
        m_operatorController.pov(270).onTrue(new ShoulderToLow(m_shoulder));
        // extend shoulder to vertical down
        m_operatorController.pov(180).onTrue(new ShoulderToIn(m_shoulder));
        
        // m_operatorController.rightTrigger().onTrue(new RunCommand(
        //     () -> m_arm.toggleArmLimit()
        // ));

        SmartDashboard.putNumber("Arm Position (ticks)", m_arm.getArmPosition());
        SmartDashboard.putNumber("Shoulder Position (ticks)", m_shoulder.getShoulderPosition());
    }


    public void containerPeriodic() {


        if (m_driverController.getHID().getXButtonPressed()) {
            m_robotDrive.setX();
        } else if (!m_driverController.getHID().getXButton()) {
            m_robotDrive.drive(
                        -MathUtil.applyDeadband(m_driverController.getLeftY() * (m_robotDrive.m_slowMode ? 0.4 : 1) * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.65 * (m_robotDrive.m_slowMode ? 0.4 : 1) * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getRightX() * 0.7 * (m_robotDrive.m_slowMode ? 0.6 : 1) * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1), OIConstants.kDriveDeadband),
                        true, false);
        }
    
        if (m_claw.autoClaw && !m_claw.objectExisted && m_claw.objectExists) {
			m_claw.close();
			SmartDashboard.putBoolean("Has closed", true);
		}

        if (m_driverController.getHID().getStartButtonPressed()) {
            resetGyro();
        }

        // ARM COMMANDS
        // while held, extend/retract the arm
        if (Math.abs(m_operatorController.getRightY()) > 0.1) {
            m_arm.setArmSpeed(m_operatorController.getRightY());
            rightJoystickPressed = true;
        } else {
            // joystick was just pressed -- means we're transitioning from arm extension to stop
            if (rightJoystickPressed) {
                m_arm.setArmSpeed(0);
                // m_arm.stop();
            }
            rightJoystickPressed = false;
        }

        // while held, extend/retract the shoulder
        if (Math.abs(m_operatorController.getLeftY()) > 0.1) {
            m_shoulder.setShoulderSpeed(m_operatorController.getLeftY() * MechanismConstants.kShoulderSpeed);
            leftJoystickPressed = true;
        } else {
            // joystick was just pressed -- means we're transitioning from shoulder extension to stop
            if (leftJoystickPressed) {
                m_shoulder.setShoulderSpeed(0);
                // m_shoulder.stop();
            }
            leftJoystickPressed = false;
        }

        if (m_operatorController.getHID().getLeftStickButtonPressed()) {
            m_claw.disableShutoff();
        }

        if (m_operatorController.getHID().getRightStickButtonPressed()) {
            m_claw.enableShutoff();
        }

        
        // if (m_driverController.getHID().getXButton()) {
        //     m_robotDrive.setX();
        // }

        if (m_operatorController.getHID().getLeftBumperPressed()) {
            m_claw.open();
        } else if (m_operatorController.getHID().getRightBumperPressed()) {
            m_claw.close();
        }

        if (m_operatorController.getHID().getRightTriggerAxis() > 0.1) {
            m_claw.setWheelsSpeed(-0.3);
            m_claw.autoClawOn();
            wheelsWereOn = true;
        } else if (m_operatorController.getHID().getLeftTriggerAxis() > 0.1) {
            m_claw.setWheelsSpeed(0.5);
            wheelsWereOn = true;
        } else if (wheelsWereOn) {
            m_claw.setWheelsSpeed(0);
            m_claw.autoClawOff();
            wheelsWereOn = false;
        }

        if(m_operatorController.getHID().getStartButtonPressed()) {
            m_arm.resetEncoder();
        }

        if(m_operatorController.getHID().getBackButtonPressed()) {
            m_shoulder.resetEncoder();
        }

        if (m_driverController.getHID().getLeftBumper()) {
            m_robotDrive.slowModeOn();
        } else if (m_driverController.getHID().getLeftBumperReleased()) {
            m_robotDrive.slowModeOff();
        }

       if (m_driverController.getHID().getRightBumper()) {
            m_robotDrive.ultraSlowModeOn();
        } else if (m_driverController.getHID().getRightBumperReleased()) {
            m_robotDrive.ultraSlowModeOff();
        } 

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected().withTimeout(14.5);
    }

    public void enableClaw() {
        m_claw.reinitializeSensor();
    }
    
    // public void enableAutomaticDistance() {
    //     m_claw.enableAutomatic();
    // }
}

// configureButtonBindings comments

  //         () -> m_robotDrive.drive(
            //             -MathUtil.applyDeadband(m_driverController.getLeftY() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
            //             -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.65 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
            //             -MathUtil.applyDeadband(m_driverController.getRightX() * 0.7 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.6 : 1), OIConstants.kDriveDeadband),
            //             false, true),
            //             m_robotDrive));
            
        // m_driverController.y().onTrue(
        //     new RunCommand(() -> m_robotDrive.setDefaultCommand(
        //         // The left stick controls translation of the robot.
        //         // Turning is controlled by the X axis of the right stick.
                
        //         // new RunCommand(
        //         //     () -> m_robotDrive.drive(
        //         //         -MathUtil.applyDeadband(m_driverController.getLeftY() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //         //         -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.65 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //         //         -MathUtil.applyDeadband(m_driverController.getRightX() * 0.7 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.6 : 1), OIConstants.kDriveDeadband),
        //         //         false, true),
        //         //         m_robotDrive)))
        //         new RunCommand(
        //             () -> m_robotDrive.drive(
        //                 -MathUtil.applyDeadband(m_driverController.getLeftY() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //                 -MathUtil.applyDeadband(m_driverController.getLeftX() * 0.65 * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.4 : 1) * m_robotDrive.m_reverseModeCoeff, OIConstants.kDriveDeadband),
        //                 -MathUtil.applyDeadband(m_driverController.getRightX() * Constants.DriveConstants.kDriveSpeed * (m_robotDrive.m_slowMode ? 0.6 : 1), OIConstants.kDriveDeadband),
        //                 true, true),
        //                 m_robotDrive)))
        // );
        
        // new JoystickButton(m_driverJoystick, 5).onTrue(
        //     new RunCommand(() ->
        //     m_robotDrive.forwardMode())
        // );

        // new JoystickButton(m_driverJoystick,3).onTrue(
        //     new RunCommand(() ->
        //     m_robotDrive.reverseMode())
        // );

        // new JoystickButton(m_driverJoystick, 2).whileTrue(
        //     new RunCommand(() ->
        //     m_robotDrive.slowModeOn(), m_robotDrive));
        //     new JoystickButton(m_driverJoystick, 2).whileFalse(
        //         new RunCommand(() ->
        //         m_robotDrive.slowModeOff(), m_robotDrive));
            

        // // setX (brake robot)
        // m_driverController.y().whileTrue(new RunCommand(
        //     () -> m_robotDrive.setX(),
        //     m_robotDrive));

        // m_driverController.a().whileTrue(new RunCommand(
        //     () -> m_robotDrive.m_frontLeft.setDriveSpeed(0.1)
        // ));

        // m_driverController.b().whileTrue(new RunCommand(
        //     () -> m_robotDrive.m_frontLeft.setTurnSpeed(0.1)
        // ));


        // slow mode
        // m_driverController.leftTrigger().onTrue(new RunCommand(
            // () -> m_robotDrive.toggleSlowMode(),
                    // m_robotDrive));
        // reverse mode
        // m_driverController.rightTrigger().onTrue(new RunCommand(
            // () -> m_robotDrive.toggleReverseMode(),
                    // m_robotDrive));

        /*
        OPERATOR CONTROLLER
        */
        /**
         * Buttons:
         * 1 (trigger): toggle claw
         * 3: extend arm while held
         * 2: retract arm while held
         * 4: fully extend arm (pid)
         * 5: fully retract arm (pid)
         * : shoulder to level 3 height (pid)
         * : shoulder to pickup height (pid)
         * : shoulder to level 2 height (pid)
         * : shoulder to level 1 height (pid)
         * big joystick: front is extend shoulder while held, back is retract shoulder while held
         * speed dial upper half: regular mode for arm and shoulder
         * speed dial lower half: slow mode for arm and shoulder
         */

        //  new JoystickButton(m_driverJoystick, 7).onTrue(new RunCommand(
        //     () -> m_claw.enableCompressor()
        // ));

        // new JoystickButton(m_driverJoystick, 8).onTrue(new RunCommand(
        //     () -> m_claw.disableCompressor()
        // ));

        // m_operatorController.leftTrigger().onTrue(new InstantCommand(
        //     () -> m_claw.toggleAutoClaw()
        // ));

        // // extend arm fully (level 3)
        // m_operatorController.y().onTrue(new ArmToHigh(m_arm));
        // // extend arm to level 2 height
        // m_operatorController.b().onTrue(new ArmToMid(m_arm));
        // // extend arm to level 1 height
        // m_operatorController.x().onTrue(new ArmToLow(m_arm));
        // // retract arm fully
        // m_operatorController.a().onTrue(new ArmToIn(m_arm));

        // m_operatorController.y().onTrue(SequentialCommands.autoEject(m_robotDrive, m_claw, m_shoulder, m_arm));



        // m_operatorController.x().onTrue(new RunCommand(
        //     () -> m_claw.reinitializeSensor()
        // ));

        // reset arm encoder



        // m_operatorController.leftTrigger().onTrue(new RunCommand(
        //     () -> m_arm.toggleLimit(false)
        // ));

        // m_operatorController.leftTrigger().onFalse(new RunCommand(
        //     () -> m_arm.toggleLimit(true)
        // ));

        // CLAW COMMANDS
        // toggle claw
        // m_operatorController.leftBumper().onTrue(new ClawToggle(m_claw));

        // m_operatorController.rightBumper().onTrue(new RunCommand(
        //     () -> m_claw.toggleClawState()
        // ));

        // m_operatorController.leftBumper().onTrue(new RunCommand(
        //     () -> m_claw.open()
        // ));

        // m_operatorController.rightBumper().onTrue(new RunCommand(
        //     () -> m_claw.close()
        // ));

        // // claw wheels out
        // m_operatorController.y().onTrue(new RunCommand(
        //     () -> m_claw.setWheelsSpeed(1)
        // ));
        // // claw wheels off
        // m_operatorController.b().onTrue(new RunCommand(
        //     () -> m_claw.setWheelsSpeed(0)
        // ));
        // claw wheels in
        // m_operatorController.a().whileTrue(new RunCommand(
        //     () -> m_claw.setWheelsSpeed(-1)
        // ));


 // public void driveControl() {
    //     if ((Math.abs(m_driverController.getLeftX()) > 0.1) || (Math.abs(m_driverController.getLeftY()) > 0.1) || (Math.abs(m_driverController.getRightX()) > 0.1)) {
    //         m_robotDrive.drive(
    //             -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //             false, true);
    //         driveOn = true;
    //     } else {
    //         if (driveOn) {
    //             // m_robotDrive.drive(0, 0, 0, true, true);
    //             m_robotDrive.stop();
    //         }
    //         driveOn = false;
    //     }
    // }