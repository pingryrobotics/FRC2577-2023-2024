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

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.autos.DoNothingAuto;

import frc.robot.commands.intake_commands.IntakeCommands;
import frc.robot.commands.shooter_commands.ShooterCommands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Ramp;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;

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
    private final Intake m_intake = new Intake(
            new CANSparkMax(Constants.MechanismConstants.kIntakeID, CANSparkMax.MotorType.kBrushless),
            new CANSparkMax(Constants.MechanismConstants.kIntakeFlipID, CANSparkMax.MotorType.kBrushless));
    private final Shooter m_shooter = new Shooter(
            new CANSparkMax(MechanismConstants.kShooterLeftID, CANSparkMax.MotorType.kBrushless),
            new CANSparkMax(MechanismConstants.kShooterRightID, CANSparkMax.MotorType.kBrushless),
            new CANSparkMax(MechanismConstants.kShooterAdjusterLeftID, CANSparkMax.MotorType.kBrushless),
            new CANSparkMax(MechanismConstants.kShooterAdjusterRightID, CANSparkMax.MotorType.kBrushless));
    private final Ramp m_ramp = new Ramp(new CANSparkMax(MechanismConstants.kRampID, CANSparkMax.MotorType.kBrushless));
    private final Indexer m_indexer = new Indexer(
            new CANSparkMax(MechanismConstants.kIndexerID, CANSparkMax.MotorType.kBrushless));

    // The driver's controller
    CommandPS4Controller m_driverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);
    // Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    // Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    CommandPS4Controller m_operatorController = new CommandPS4Controller(OIConstants.kOperatorControllerPort);

    SendableChooser<Command> m_chooser = new SendableChooser<>();
    SendableChooser<Integer> side_chooser = new SendableChooser<>();

    Map<String, Boolean> buttonStates = new HashMap<String, Boolean>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // UsbCamera camera = CameraServer.startAutomaticCapture();
        // camera.setFPS(30);
        // camera.setResolution(256, 144);

        side_chooser.setDefaultOption("Red Left", 0);
        side_chooser.addOption("Red Center", 1);
        side_chooser.addOption("Red Right", 2);
        side_chooser.addOption("Blue Left", 3);
        side_chooser.addOption("Blue Center", 4);
        side_chooser.addOption("Blue Right", 5);

        SmartDashboard.putData("Side", side_chooser);

        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                    () -> m_robotDrive.drive(
                        -MathUtil.applyDeadband(m_driverController.getLeftY() * (m_robotDrive.m_slowMode ? 0.4 : 1) * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getLeftX() * (m_robotDrive.m_slowMode ? 0.4 : 1) * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getRightX() * (m_robotDrive.m_slowMode ? 0.4 : 1) * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1), OIConstants.kDriveDeadband),
                        true, false),
                        m_robotDrive));
            
        
        // // Add commands to Autonomous Sendable Chooser
        m_chooser.setDefaultOption("Do Nothing", new DoNothingAuto());

        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto mode", m_chooser);
    }

    public void resetEncoders() {
        m_intake.resetEncoder();
        m_shooter.resetEncoder();
    }

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

        // INTAKE/RAMP COMMANDS
          
        m_operatorController.cross().onTrue(IntakeCommands.IntakeUp(m_intake)); // intake down PID
        m_operatorController.triangle().onTrue(IntakeCommands.IntakeDown(m_intake)); // intake up PID

        m_operatorController.square().onTrue(IntakeCommands.IntakeIn(m_intake, m_ramp)); // intake & ramp wheels in
        m_operatorController.square().onFalse(IntakeCommands.IntakeStop(m_intake, m_ramp)); // intake & ramp wheels stop
        m_operatorController.circle().onTrue(IntakeCommands.IntakeOut(m_intake, m_ramp)); // intake & ramp wheels out
        m_operatorController.circle().onFalse(IntakeCommands.IntakeStop(m_intake, m_ramp)); // intake & ramp wheels stop

        // SHOOTER ADJUSTMENT COMMANDS

        // UP POV
        m_operatorController.pov(0).onTrue(ShooterCommands.AdjustShooterHigh(m_shooter)); // shooter adjust to high
        // RIGHT POV
        m_operatorController.pov(90).onTrue(ShooterCommands.AdjustShooterMid(m_shooter)); // shooter adjust to mid
        // DOWN POV
        m_operatorController.pov(180).onTrue(ShooterCommands.AdjustShooterLow(m_shooter)); // shooter adjust to low

        // SHOOTER COMMANDS

        m_operatorController.L2().onTrue(ShooterCommands.Shoot(m_shooter)); // shooter on
        m_operatorController.L2().onFalse(ShooterCommands.StopShooter(m_shooter)); // shooter off
        m_operatorController.R2().onTrue(ShooterCommands.Index(m_indexer)); // indexer out
        m_operatorController.R2().onFalse(ShooterCommands.StopIndexer(m_indexer)); // indexer stop

        m_operatorController.L1().onTrue(ShooterCommands.ShootAndIndex(m_shooter, m_indexer)); // shooter on, wait, and indexer out
    }

    public void containerPeriodic() {

        // DRIVE COMMANDS

        if (m_driverController.getHID().getL2Button()) {
            m_robotDrive.slowModeOn();
        } else if (m_driverController.getHID().getL2ButtonReleased()) {
            m_robotDrive.slowModeOff();
        }

       if (m_driverController.getHID().getR2Button()) {
            m_robotDrive.ultraSlowModeOn();
        } else if (m_driverController.getHID().getR2ButtonReleased()) {
            m_robotDrive.ultraSlowModeOff();
        }
        
        // OPERATOR JOYSTICK COMMANDS
        // purpose of having it here is to be able to track when the joystick passes a certain threshold or goes under a certain threshold (we only want to track the transition)
        if (m_operatorController.getLeftY() > 0.1) {
            double flipSpeed = m_operatorController.getLeftY() * MechanismConstants.kIntakeFlipUpSpeed;
            m_intake.setFlipSpeed(flipSpeed);
            buttonStates.put("operatorLeftJoystick", true);
        } else if (m_operatorController.getLeftY() < -0.1) {
            double flipSpeed = m_operatorController.getLeftY() * MechanismConstants.kIntakeFlipDownSpeed;
            m_intake.setFlipSpeed(flipSpeed);
            buttonStates.put("operatorLeftJoystick", true);
        } else {
            if (buttonStates.containsKey("operatorLeftJoystick")) {
                m_intake.setFlipSpeed(0);
                buttonStates.put("operatorLeftJoystick", false);
            }
        }

        if (Math.abs(m_operatorController.getRightY()) > 0.1) {
            double adjusterSpeed = m_operatorController.getRightY() * MechanismConstants.kShooterAdjusterSpeed;
            m_shooter.setAdjusterSpeed(adjusterSpeed);
            buttonStates.put("operatorRightJoystick", true);
        } else {
            if (buttonStates.containsKey("operatorRightJoystick")) {
                m_shooter.setAdjusterSpeed(0);
                buttonStates.put("operatorRightJoystick", false);
            }
        }

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
