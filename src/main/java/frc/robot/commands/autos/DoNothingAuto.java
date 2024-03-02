package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DoNothingAuto extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DriveSubsystem m_robotDrive;

	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public DoNothingAuto(DriveSubsystem drive) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drive);
		m_robotDrive = drive;
		m_robotDrive.resetOdometry(new Pose2d(1.38, 5.54, new Rotation2d(Math.PI)));
	}
}