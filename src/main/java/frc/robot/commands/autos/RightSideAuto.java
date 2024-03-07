package frc.robot.commands.autos;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

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
import frc.robot.Constants.MechanismConstants;
import frc.robot.commands.intake_commands.IntakeCommands;
import frc.robot.commands.shooter_commands.ShooterCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.Shooter;

import frc.robot.util.trajectory.TrajectoryCommandGenerator;

/** An example command that uses an example subsystem. */
public class RightSideAuto extends SequentialCommandGroup {
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
    public RightSideAuto(DriveSubsystem drive, Shooter shooter, Indexer indexer, Intake intake, Ramp ramp,
            SendableChooser<Integer> side_chooser, boolean one, boolean two, boolean three, boolean park) {

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

        Pose2d start = new Pose2d(0.73, 4.43, Rotation2d.fromDegrees(-60));

        boolean flip = false;

        switch (side_chooser.getSelected()) {
            case 0:
                flip = true;
            case 1:
                flip = true;
            case 2:
                flip = true;
            case 3:
            case 4:
            case 5:
        }
        Pose2d flippedStart;
        if (flip) {
            Translation2d flippedStartTranslation = GeometryUtil.flipFieldPosition(start.getTranslation());
            Rotation2d flippedStartRotation = GeometryUtil.flipFieldRotation(start.getRotation());
            flippedStart = new Pose2d(flippedStartTranslation, flippedStartRotation);
        } else {
            flippedStart = start;
        }
        m_robotDrive.resetOdometry(flippedStart);

        String path1 = "3.1";
        String path2 = "3.2";

        Command path1Cmd = m_robotDrive.followPathCommand(path1);
        Command path2Cmd = m_robotDrive.followPathCommand(path2);

        if (one) {
            addCommands(
                // IntakeCommands.IntakeDown(m_intake), // move intake down. command ends instantly but might take a second to actually move
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> m_intake.setFlipSpeed(-MechanismConstants.kIntakeFlipDownSpeed), m_intake),
                        new WaitCommand(1),
                        new InstantCommand(() -> m_intake.setFlipSpeed(0), m_intake)
                    ),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(() -> m_shooter.setAdjusterSpeed(MechanismConstants.kShooterAdjusterAutoSpeed), m_shooter),
                        new WaitCommand(1.5),
                        new InstantCommand(() -> m_shooter.setAdjusterSpeed(0), m_shooter)
                    )
                    // ShooterCommands.AdjustShooterHigh(m_shooter)
                ),
                ShooterCommands.ShootAndIndex(m_shooter, m_indexer) // shoot once. 2ish seconds
            );
        }
        if (park) { // park for 1 note
            addCommands(
                new InstantCommand(() -> m_robotDrive.drive(0, 0.5, 0, false, false)),
                new WaitCommand(2),
                new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false))
            );
        }
        if (two) {
            addCommands(
                IntakeCommands.IntakeRampIn(intake, m_ramp), // intake and ramp in
                new ParallelCommandGroup(
                    path1Cmd // drive to top left pickup position. 1.5 seconds
                ),
<<<<<<< Updated upstream
                new WaitCommand(1), // give it a second to intake and ramp the note
                IntakeCommands.StopIntake(m_intake), // keep ramp going so it feeds into indexer
                ShooterCommands.ShootForward(shooter),
                new ParallelCommandGroup(
                    path2Cmd
                ),
                new WaitCommand(1),
                ShooterCommands.IndexForward(m_indexer), // shoot again, shooter is already running
                new WaitCommand(1), // give it a second to shoot
                ShooterCommands.StopIndexer(m_indexer),
                ShooterCommands.StopShooter(m_shooter),
                IntakeCommands.StopRamp(m_ramp)
            );
        } 
=======
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    ShooterCommands.ShootAndIndex(m_shooter, m_indexer) // shoot once. 2ish seconds
                )
            ),
            IntakeCommands.IntakeRampIn(intake, m_ramp), // intake and ramp in
            new ParallelCommandGroup(
                path1Cmd // drive to top left pickup position. 1.5 seconds
            ),
            new WaitCommand(1), // give it a second to intake and ramp the note
            IntakeCommands.StopIntake(m_intake), // keep ramp going so it feeds into indexer
            ShooterCommands.ShootForward(shooter),
            new ParallelCommandGroup(
                path2Cmd
            ),
            new WaitCommand(1),
            ShooterCommands.IndexForward(m_indexer), // shoot again, shooter is already running
            new WaitCommand(1), // give it a second to shoot
            ShooterCommands.StopIndexer(m_indexer),
            ShooterCommands.StopShooter(m_shooter),
            IntakeCommands.StopRamp(m_ramp)
        );
>>>>>>> Stashed changes
        // go for 3rd note
        if (three) {
            String path3 = "3.3";
            String path4 = "3.4";
            Command path3Cmd = m_robotDrive.followPathCommand(path3);
            Command path4Cmd = m_robotDrive.followPathCommand(path4);
            addCommands(
                IntakeCommands.IntakeRampIn(m_intake, m_ramp),
                new ParallelCommandGroup(
                    path3Cmd
                ),
                new WaitCommand(1),
                IntakeCommands.StopIntake(m_intake), // keep ramp going so it feeds into indexer
                new ParallelCommandGroup(
                    path4Cmd,
                    new SequentialCommandGroup(
                        new WaitCommand(3),
                        ShooterCommands.ShootForward(m_shooter)
                    )
                ),
                    ShooterCommands.IndexForward(m_indexer),
                    new WaitCommand(1),
                IntakeCommands.StopRamp(m_ramp)
            );
        }
        }
} 