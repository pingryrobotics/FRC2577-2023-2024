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
public class OneNoteParkAuto extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_robotDrive;
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private final Ramp m_ramp;
    private double ySpeed = 0;
    private double xSpeed = 0;

	/**
	* One Piece Park Auto.
	* Places preloaded piece (cube) onto node and parks on a third of the charge station
	*/
    public OneNoteParkAuto(DriveSubsystem drive, Shooter shooter, Indexer indexer, Intake intake, Ramp ramp,
            SendableChooser<Integer> side_chooser, boolean park) {
        
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

        // switch (side_chooser.getSelected()) {
        //     case 0: // red left
        //         m_robotDrive.resetOdometry(new Pose2d(0.68, 6.69, new Rotation2d(Math.PI/3)));
        //         ySpeed = 0;
        //         xSpeed = 0;
        //     case 1: // red center
        //         m_robotDrive.resetOdometry(new Pose2d(0.68, 6.69, new Rotation2d(0)));
        //         ySpeed = 0.06;
        //         xSpeed = 0.4;
        //     case 2: // red right
        //         m_robotDrive.resetOdometry(new Pose2d(0.68, 6.69, new Rotation2d(-Math.PI/3)));
        //         // ySpeed = 0.2*Math.cos(-Math.PI/3);
        //         // xSpeed = 0.2*Math.sin(-Math.PI/3);
        //     case 3: // blue left
        //         m_robotDrive.resetOdometry(new Pose2d(0.68, 6.69, new Rotation2d(Math.PI/3)));
        //         // ySpeed = 0.2*Math.cos(Math.PI/3);
        //         // xSpeed = 0.2*Math.sin(Math.PI/3);
        //     case 4: // blue center
        //         m_robotDrive.resetOdometry(new Pose2d(0.68, 6.69, new Rotation2d(0)));
        //         ySpeed = -0.06;
        //         xSpeed = 0.4;
        //     case 5: // blue right
        //         m_robotDrive.resetOdometry(new Pose2d(0.68, 6.69, new Rotation2d(-Math.PI/3)));
        //         ySpeed = 0;
        //         xSpeed = 0;
        // }

        // if (!park) {
        //     xSpeed = 0;
        //     ySpeed = 0;
        // }

        addCommands(
            // IntakeCommands.IntakeRampIn(m_intake, m_ramp),
            IntakeCommands.RampIn(m_ramp),
            ShooterCommands.ShootForward(m_shooter),
            new WaitCommand(3),
            ShooterCommands.IndexForward(m_indexer),
            new WaitCommand(2),
            ShooterCommands.StopShooter(m_shooter),
            ShooterCommands.StopIndexer(m_indexer),
            IntakeCommands.StopRamp(m_ramp),
            new InstantCommand(() -> m_robotDrive.drive(0, 0.5, 0, false, false)),
            new WaitCommand(3),
            new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false))
        );
	}
} 