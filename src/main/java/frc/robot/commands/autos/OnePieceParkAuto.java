package frc.robot.commands.autos;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

// commented because i don't like seeing yellow dots on my sidebar - christian
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Filesystem;
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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

import frc.robot.util.trajectory.TrajectoryCommandGenerator;

/** An example command that uses an example subsystem. */
public class OnePieceParkAuto extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_robotDrive;
    private Trajectory traj = new Trajectory();

	/**
	* One Piece Park Auto.
	* Places preloaded piece (cube) onto node and parks on a third of the charge station
	*/
    public OnePieceParkAuto(DriveSubsystem drive, SendableChooser<Integer> side_chooser, boolean place, boolean park) {
        m_robotDrive = drive;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_robotDrive);

        String trajectoryJSON;
        switch (side_chooser.getSelected()) {
            // case 0:
            // case 1:
            case 2:
                // trajectoryJSON = "pathplanner/generatedJSON/Pickup.wpilib.json";
            // case 3:
            // case 4:
            // case 5:
            default:
                trajectoryJSON = "pathplanner/generatedJSON/DoNothing.wpilib.json";
        }
        try {
            traj = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON));
        } catch (IOException ie) {

        }

        SwerveControllerCommand toPark = TrajectoryCommandGenerator.generateCommand(traj, m_robotDrive);

        if (place) {
            addCommands(

            );
        }
        if (park) {
            addCommands(
                toPark,
                new InstantCommand(() -> m_robotDrive.setX()).withTimeout(1)
            );
        }
	}
} 