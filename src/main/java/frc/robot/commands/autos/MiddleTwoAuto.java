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
public class MiddleTwoAuto extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private final Intake m_intake;
  private final Ramp m_ramp;

  /**
   * One Piece Park Auto.
   * Places preloaded piece (cube) onto node and parks on a third of the charge
   * station
   */
  public MiddleTwoAuto(DriveSubsystem drive, Shooter shooter, Indexer indexer, Intake intake, Ramp ramp,
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

    addCommands(
        new ParallelCommandGroup(
            // turn on intake
            new SequentialCommandGroup(
                new InstantCommand(() -> m_intake.setFlipSpeed(-MechanismConstants.kIntakeFlipDownSpeed), m_intake),
                new WaitCommand(1),
                new InstantCommand(() -> m_intake.setFlipSpeed(0), m_intake)),

            // shoot preloaded
            ShooterCommands.ShootAndIndex(m_shooter, m_indexer) // shoot once. 2ish seconds
        ),

        // intake is now down; turn it on and drive to pickup
        IntakeCommands.IntakeRampIn(m_intake, m_ramp),
        new InstantCommand(() -> m_robotDrive.drive(0.5, 0, 0, false, false)),
        new WaitCommand(3),
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)),

        // turn on shooter to give it time to rev up
        ShooterCommands.ShootForward(m_shooter),
        new WaitCommand(1), // wait to let the note intake

        // drive back to start position
        new InstantCommand(() -> m_robotDrive.drive(-0.5, 0, 0, false, false)),
        new WaitCommand(3),
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)),
        
        // shoot out
        ShooterCommands.IndexForward(m_indexer),
        new WaitCommand(2),

        // stop everything
        ShooterCommands.StopShooter(m_shooter),
        ShooterCommands.StopIndexer(m_indexer),
        IntakeCommands.IntakeRampStop(m_intake, m_ramp));
  }
}