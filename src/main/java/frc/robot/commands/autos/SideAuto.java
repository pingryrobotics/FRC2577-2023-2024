package frc.robot.commands.autos;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.intake_commands.IntakeCommands;
import frc.robot.commands.shooter_commands.ShooterCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.Shooter;

import frc.robot.util.trajectory.TrajectoryCommandGenerator;

/** An example command that uses an example subsystem. */
public class SideAuto extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_robotDrive;
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private final Ramp m_ramp;

	/**
	* One Piece Park Auto.
	* Places preloaded piece (cube) onto node and parks on a third of the charge station
	*/
    public SideAuto(DriveSubsystem drive, Shooter shooter, Indexer indexer, Intake intake, Ramp ramp,
            SendableChooser<Integer> side_chooser) {
        
        m_robotDrive = drive;
        m_shooter = shooter;
        m_indexer = indexer;
        m_intake = intake;
        m_ramp = ramp;

		// Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_robotDrive);
        addRequirements(m_shooter);
        addRequirements(m_indexer);
        addRequirements(m_intake);
        addRequirements(m_ramp);

        m_robotDrive.resetOdometry(new Pose2d(0.68, 6.69, new Rotation2d(-2*Math.PI/3)));

        String path1 = "2.1";
        String path2 = "2.2";

        Command path1Cmd = m_robotDrive.followPathCommand(path1);
        Command path2Cmd = m_robotDrive.followPathCommand(path2);

        addCommands(
            IntakeCommands.IntakeDown(m_intake), // move intake down. command ends instantly but might take a second to actually move
            ShooterCommands.ShootAndIndex(m_shooter, m_indexer), // shoot once. 3 seconds
            IntakeCommands.IntakeRampIn(intake, m_ramp), // intake and ramp in
            new ParallelCommandGroup(
                path1Cmd // drive to top left shooting position. 1.5 seconds
            ),
            new WaitCommand(2), // give it a second to intake and ramp the note
            IntakeCommands.StopIntake(m_intake), // keep ramp going so it feeds into indexer
            ShooterCommands.Index(m_indexer), // shoot again, shooter is already running
            new WaitCommand(1), // give it a second to shoot
            ShooterCommands.StopIndexer(m_indexer),
            ShooterCommands.StopShooter(m_shooter),
            IntakeCommands.StopRamp(m_ramp),
            new ParallelCommandGroup(
                path2Cmd, // 2nd path to drive to top right pickup and back. 2.75 seconds to get there, 2.75 seconds to get back
                new SequentialCommandGroup( // start up shooter halfway through path
                    new WaitCommand(3),
                    ShooterCommands.Shoot(m_shooter)
                ), 
                new SequentialCommandGroup( // start up intake and ramp when we're almost there
                    new WaitCommand(1.5),
                    IntakeCommands.IntakeRampIn(m_intake, m_ramp),
                    new WaitCommand(3),
                    IntakeCommands.StopIntake(m_intake)
                )
            ),
            new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive),
            new WaitCommand(0.5), // give it time to stop
            ShooterCommands.Index(m_indexer), // index the last note
            new WaitCommand(1), // give it a second to shoot
            ShooterCommands.StopIndexer(m_indexer),
            ShooterCommands.StopShooter(m_shooter),
            IntakeCommands.StopRamp(m_ramp)
        );
	}
} 