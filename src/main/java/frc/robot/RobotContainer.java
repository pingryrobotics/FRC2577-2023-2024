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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.autos.LeftSideAuto;
import frc.robot.commands.autos.MiddleTwoAuto;
import frc.robot.commands.autos.RightSideAuto;
import frc.robot.commands.autos.OneNoteParkAuto;
import frc.robot.commands.climber_commands.Climb;
import frc.robot.commands.intake_commands.IntakeCommands;
import frc.robot.commands.shooter_commands.ShooterCommands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
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
            new CANSparkMax(MechanismConstants.kShooterRightID, CANSparkMax.MotorType.kBrushless));
    private final Ramp m_ramp = new Ramp(new CANSparkMax(MechanismConstants.kRampID, CANSparkMax.MotorType.kBrushless));
    private final Indexer m_indexer = new Indexer(
            new CANSparkMax(MechanismConstants.kIndexerID, CANSparkMax.MotorType.kBrushless));
    // private final Climber m_climber = new Climber(
    //         new CANSparkMax(MechanismConstants.kLeftClimberID, CANSparkMax.MotorType.kBrushless),
    //         new CANSparkMax(MechanismConstants.kRightClimberID, CANSparkMax.MotorType.kBrushless));

    // The driver's controller
    CommandPS4Controller m_driverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);
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

        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setFPS(30);
        camera.setResolution(256, 144);

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
                                -MathUtil
                                        .applyDeadband(
                                                m_driverController.getLeftY() * (m_robotDrive.m_slowMode ? 0.4 : 1)
                                                        * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1),
                                                OIConstants.kDriveDeadband),
                        
                                -MathUtil
                                        .applyDeadband(
                                                m_driverController.getLeftX() * (m_robotDrive.m_slowMode ? 0.4 : 1)
                                                        * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1),
                                                OIConstants.kDriveDeadband),
                        
                                -MathUtil
                                        .applyDeadband(
                                                m_driverController.getRightX() * (m_robotDrive.m_slowMode ? 0.4 : 1)
                                                        * (m_robotDrive.m_ultraSlowMode ? 0.5 : 1),
                                                OIConstants.kDriveDeadband),
                        
                        true, false),
                        m_robotDrive));
            
        
        // // Add commands to Autonomous Sendable Chooser
        m_chooser.setDefaultOption("Do Nothing", new DoNothingAuto(m_robotDrive));
        m_chooser.addOption("Side 3 Note Auto",
                new LeftSideAuto(m_robotDrive, m_shooter, m_indexer, m_intake, m_ramp, side_chooser));
        m_chooser.addOption("regular 1 note auto",
                new OneNoteParkAuto(m_robotDrive, m_shooter, m_indexer, m_intake, m_ramp, side_chooser, false));
        m_chooser.addOption("park",
                new RightSideAuto(m_robotDrive, m_shooter, m_indexer, m_intake, m_ramp, side_chooser, false, false, false, true));
        m_chooser.addOption("Right 1 Note Auto No Park", 
                new RightSideAuto(m_robotDrive, m_shooter, m_indexer, m_intake, m_ramp, side_chooser, true, false, false, false));
        m_chooser.addOption("Right 1 Note Auto Park", 
                new RightSideAuto(m_robotDrive, m_shooter, m_indexer, m_intake, m_ramp, side_chooser, true, false, false, true));
        m_chooser.addOption("Right Two Note Auto",
                new RightSideAuto(m_robotDrive, m_shooter, m_indexer, m_intake, m_ramp, side_chooser, true, true, false, false));
        m_chooser.addOption("Right Three Note Auto",
                new RightSideAuto(m_robotDrive, m_shooter, m_indexer, m_intake, m_ramp, side_chooser, true, false, false, false));
        m_chooser.addOption("Middle Two Note Auto",
                new MiddleTwoAuto(m_robotDrive, m_shooter, m_indexer, m_intake, m_ramp, side_chooser));

        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto mode", m_chooser);
    }

    public void resetEncoders() {
        m_intake.resetEncoder();
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
          
        m_operatorController.cross().onTrue(IntakeCommands.IntakeUp(m_intake)); // intake up PID
        m_operatorController.triangle().onTrue(IntakeCommands.IntakeDown(m_intake)); // intake down PID

        m_operatorController.square().onTrue(IntakeCommands.IntakeRampIn(m_intake, m_ramp)); // intake & ramp wheels in
        m_operatorController.square().onFalse(IntakeCommands.IntakeRampStop(m_intake, m_ramp)); // intake & ramp wheels stop
        m_operatorController.circle().onTrue(IntakeCommands.IntakeRampOut(m_intake, m_ramp)); // intake & ramp wheels out
        m_operatorController.circle().onFalse(IntakeCommands.IntakeRampStop(m_intake, m_ramp)); // intake & ramp wheels stop

        // SHOOTER ADJUSTMENT COMMANDS

        // // UP POV
        // m_operatorController.pov(0).onTrue(ShooterCommands.AdjustShooterHigh(m_shooter)); // shooter adjust to high
        // // RIGHT POV
        // m_operatorController.pov(90).onTrue(ShooterCommands.AdjustShooterMid(m_shooter)); // shooter adjust to mid
        // // DOWN POV
        // m_operatorController.pov(180).onTrue(ShooterCommands.AdjustShooterLow(m_shooter)); // shooter adjust to low

        // SHOOTER COMMANDS

        m_operatorController.L1().onTrue(ShooterCommands.ShootReverse(m_shooter)); // shooter on
        m_operatorController.L1().onFalse(ShooterCommands.StopShooter(m_shooter)); // shooter off
        m_operatorController.R1().onTrue(ShooterCommands.IndexReverse(m_indexer)); // indexer out
        m_operatorController.R1().onFalse(ShooterCommands.StopIndexer(m_indexer)); // indexer stop

        m_operatorController.share().onTrue(new InstantCommand(() -> resetEncoders()));

        //old alan code
        //m_operatorController.touchpad().onTrue(ShooterCommands.ShootAndIndex(m_shooter, m_indexer));

        //new jame code
        m_operatorController.touchpad().onTrue(ShooterCommands.ShootForward(m_shooter));

        m_driverController.share().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

        // m_driverController.square().onTrue(new Climb(m_climber, MechanismConstants.kClimberSpeed));
        // m_driverController.square().onFalse(new Climb(m_climber, 0));
    }

    public void containerPeriodic() {

        // DRIVE COMMANDS
        double shooterSpeed = m_shooter.getSpeed();

        if(shooterSpeed > .7) {
            m_operatorController.getHID().setRumble(RumbleType.kLeftRumble, .5);
            m_operatorController.getHID().setRumble(RumbleType.kRightRumble, .5);
        } else {
           m_operatorController.getHID().setRumble(RumbleType.kLeftRumble, .0);
            m_operatorController.getHID().setRumble(RumbleType.kRightRumble, .0);
        }

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
        if (-m_operatorController.getLeftY() > 0.2) { // moving joystick down is positive
            double flipSpeed = -m_operatorController.getLeftY() * MechanismConstants.kIntakeFlipUpSpeed;
            m_intake.setFlipSpeed(flipSpeed);
            buttonStates.put("operatorLeftJoystick", true);
        } else if (-m_operatorController.getLeftY() < -0.1) { // moving joystick up is negative
            double flipSpeed = -m_operatorController.getLeftY() * MechanismConstants.kIntakeFlipDownSpeed;
            m_intake.setFlipSpeed(flipSpeed);
            buttonStates.put("operatorLeftJoystick", true);
        } else {
            if (buttonStates.containsKey("operatorLeftJoystick") && buttonStates.get("operatorLeftJoystick")) {
                m_intake.setFlipSpeed(0);
                buttonStates.put("operatorLeftJoystick", false);
            }
        }

        // if (Math.abs(m_operatorController.getRightY()) > 0.1) {
        //     double adjusterSpeed = m_operatorController.getRightY() * MechanismConstants.kShooterAdjusterSpeed;
        //     m_shooter.setAdjusterSpeed(adjusterSpeed);
        //     buttonStates.put("operatorRightJoystick", true);
        // } else {
        //     if (buttonStates.containsKey("operatorRightJoystick") && buttonStates.get("operatorRightJoystick")) {
        //         m_shooter.setAdjusterSpeed(0);
        //         buttonStates.put("operatorRightJoystick", false);
        //     }
        // }

        // OPERATOR CONTROLLER COMMANDS

        if (m_operatorController.getHID().getL2Button()) {
            m_shooter.setSpeed(MechanismConstants.kShooterSpeed * m_operatorController.getHID().getL2Axis());
        } else if (m_operatorController.getHID().getL2ButtonReleased()) {
            m_shooter.setSpeed(0);
        }

        if (m_operatorController.getHID().getR2Button()) {
            m_indexer.setSpeed(MechanismConstants.kIndexerSpeed * m_operatorController.getHID().getR2Axis());
            m_intake.setSpeed(MechanismConstants.kIntakeInSpeed * m_operatorController.getHID().getR2Axis());
        } else if (m_operatorController.getHID().getR2ButtonReleased()) {
            m_indexer.setSpeed(0);
            m_intake.setSpeed(0);
            m_shooter.setSpeed(0);
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
