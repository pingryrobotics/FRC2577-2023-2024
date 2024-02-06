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
import frc.robot.commands.drive_commands.DriveForward;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shoulder;
import frc.robot.util.trajectory.TrajectoryCommandGenerator;

/** An example command that uses an example subsystem. */
public class MoveForwardAuto extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_robotDrive;
    private final Claw m_claw;
    private final Shoulder m_shoulder;
    private final Arm m_arm;

	/**
	* One Piece Park Auto.
	* Places preloaded piece (cube) onto node and parks on a third of the charge station
	*/
    public MoveForwardAuto(DriveSubsystem drive, Claw claw, Shoulder shoulder, Arm arm, SendableChooser<Integer> side_chooser, boolean place, boolean park) {
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
                ).withTimeout(0.5),
                new WaitCommand(1),
                new ShoulderPID(m_shoulder, Constants.MechanismConstants.kshoulderHighPosition + (-20)).withTimeout(2),
                new WaitCommand(2),
                // new DriveForward(m_robotDrive, true).withTimeout(0.7),
                // new RunCommand(
                //     () -> m_robotDrive.drive(0, 0, 0, false, false)
                //     // () -> m_robotDrive.stop()
                // ).withTimeout(0.5),
                new ClawWheels(m_claw, 1),
                new WaitCommand(1.5),
                new ClawWheels(m_claw, 0),
                new WaitCommand(0.5),
                new ShoulderPID(m_shoulder, 0).withTimeout(2),
                new WaitCommand(2)
                // new ArmToHigh(m_arm).withTimeout(2),
                // new WaitCommand(2),
                // new InstantCommand(
                //     () -> m_claw.open()
                // ),
                // new ArmExtend(m_arm).withTimeout(1.5),
            );
        }
        if (park) {
            addCommands(
                new DriveForward(m_robotDrive, false, 0.2).withTimeout(5),
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
        }
	}
} 