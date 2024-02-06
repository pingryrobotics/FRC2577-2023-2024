package frc.robot.commands.autos;

import java.util.List;

// commented because i don't like seeing yellow dots on my sidebar - christian
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
//import frc.robot.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.FieldConstants.Community;
import frc.robot.FieldConstants.Grids;
import frc.robot.commands.shoulder_commands.*;
import frc.robot.commands.arm_commands.*;
import frc.robot.commands.claw_commands.*;
import frc.robot.commands.drive_commands.CheckGyro;
import frc.robot.commands.drive_commands.DriveForward;
import frc.robot.commands.drive_commands.DriveForwardGyro;
import frc.robot.commands.drive_commands.DriveX;
import frc.robot.commands.grouped_commands.SequentialCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shoulder;
import frc.robot.util.trajectory.TrajectoryCommandGenerator;

/** An example command that uses an example subsystem. */
public class PlaceParkAuto extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_robotDrive;
    private final Claw m_claw;
    private final Shoulder m_shoulder;
    private final Arm m_arm;

	/**
	* One Piece Park Auto.
	* Places preloaded piece (cube) onto node and parks on a third of the charge station
	*/
    public PlaceParkAuto(DriveSubsystem drive, Claw claw, Shoulder shoulder, Arm arm, SendableChooser<Integer> side_chooser, boolean place, boolean balance, boolean park, boolean newBalance) {
        m_robotDrive = drive;
        m_claw = claw;
        m_shoulder = shoulder;
        m_arm = arm;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_robotDrive);
        addRequirements(m_claw);
		addRequirements(m_arm);
		addRequirements(m_shoulder);


        /**
         * COORDINATE SYSTEM
         * All coordinates based on close right corner (from view of blue alliance driver station)
         * Y is movement along the same distance from driver station, left is positive
         * X is movement away and towards driver station, away is positive
         */

        // addCommands(
        //     new DriveForward(m_robotDrive, false).withTimeout(0.75),
        //     new RunCommand(
        //             () -> m_robotDrive.drive(0, 0, 0, false, false)
        //             // () -> m_robotDrive.stop()
        //         ).withTimeout(0.5)
        // );
        if (place) {
            addCommands(
                new RunCommand(
                    () -> m_claw.close()
                ).withTimeout(0.2),
                new ShoulderPID(m_shoulder, 73).withTimeout(2),
                new WaitCommand(0.5),
                new ArmPID(m_arm, Constants.MechanismConstants.kArmFullExtension).withTimeout(2),
                new WaitCommand(0.5),
                new ClawWheels(m_claw, 0.55),// POWER
                new WaitCommand(0.75),
                new ClawWheels(m_claw, 0),
                new ClawOpen(m_claw),
                new WaitCommand(0.25),
                new ArmPID(m_arm, 0).withTimeout(2),
                new ShoulderPID(m_shoulder, 0).withTimeout(2)
            );
        }
        if (balance) {
            addCommands(
                new DriveForwardGyro(m_robotDrive).withTimeout(12),
                // correct for the overcorrection
                new RunCommand(
                    () -> m_robotDrive.drive(-0.1, 0, 0, false, false) // 0.08
                    // () -> m_robotDrive.stop()
                ).withTimeout(1.72), //. BACKWARDS // 1.65
                new RunCommand(
                    () -> m_robotDrive.drive(0, 0, 0, false, false)
                    // () -> m_robotDrive.stop()
                ).withTimeout(0.5),
                new RunCommand(
                    () -> m_robotDrive.setX()
                ).withTimeout(5)
            );
        } else if (park) {
            addCommands(
                new DriveForward(m_robotDrive, true, 0.2).withTimeout(4.5),
                // new WaitCommand(5.0),
                new RunCommand(
                    () -> m_robotDrive.drive(0, 0, 0, false, false)
                    // () -> m_robotDrive.stop()
                ).withTimeout(0.5),
                new RunCommand(
                    () -> m_robotDrive.setX()
                ).withTimeout(1),
                new WaitCommand(2.0)        
            );
        } else if (newBalance) {
            addCommands(
                new DriveForwardGyro(m_robotDrive).withTimeout(12));
                new CheckGyro(m_robotDrive);
                // correct for the overcorrection
                // new RunCommand(
                //     () -> m_robotDrive.drive(-0.1, 0, 0, false, false) // 0.08
                    // () -> m_robotDrive.stop()
                // ).withTimeout(1.72)); //. BACKWARDS // 1.65
                // new CheckGyro(m_robotDrive).withTimeout(.2),
                // new WaitCommand(1),
                // new RunCommand(
                //     () -> m_robotDrive.drive(0, 0, 0, false, false)
                //     // () -> m_robotDrive.stop()
                // ).withTimeout(0.5),
                // new RunCommand(
                //     () -> m_robotDrive.setX()
                // ).withTimeout(5));
            // int cnt = 0;
            // for (int cnt = 0; cnt < 10; cnt++) {
            //     if (Math.abs(m_robotDrive.m_gyro.getGyroAngleY()) > 5) {
            //         addCommands(
            //             new DriveForward(m_robotDrive, (m_robotDrive.m_gyro.getGyroAngleY() > 0 ? false : true),.1).withTimeout(0.5),
            //             new RunCommand(
            //                 () ->
            //                  m_robotDrive.setX()
            //             ).withTimeout(0.1),
            //             new WaitCommand(1));
            //         // move in direc for some seconds
            //         // cnt++;
            //     } else {
            //         addCommands(                
            //         new RunCommand(
            //             () -> m_robotDrive.setX()));
            //         cnt--;
            //     }
            //     if (Math.abs(m_robotDrive.m_gyro.getGyroAngleY()) > 5) {
            //         addCommands(
            //             new DriveForward(m_robotDrive, (m_robotDrive.m_gyro.getGyroAngleY() > 0 ? false : true),.1).withTimeout(0.5),
            //             new RunCommand(
            //                 () -> m_robotDrive.setX()
            //             ).withTimeout(0.1),
            //             new WaitCommand(1));
            //         // move in direc for some seconds
            //         cnt++;
            //     } else {
            //         addCommands(                
            //         new RunCommand(
            //             () -> m_robotDrive.setX()));
            //     }
            
 
            
        }
	}
} 