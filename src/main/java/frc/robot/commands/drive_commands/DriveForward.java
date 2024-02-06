package frc.robot.commands.drive_commands; //CTV

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem; //CTV

/** An example command that uses an example subsystem. */
public class DriveForward extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_subsystem;
    private boolean forward;
    private double speed;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveForward(DriveSubsystem subsystem, boolean forward, double speed) {
        m_subsystem = subsystem;
        this.forward = forward;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.drive((forward ? 1 : -1) * speed, 0, 0, false, false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.setX();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}