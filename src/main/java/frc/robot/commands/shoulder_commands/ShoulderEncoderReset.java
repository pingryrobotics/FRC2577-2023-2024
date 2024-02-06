package frc.robot.commands.shoulder_commands; //CTV

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ExampleSubsystem; //CTV

/** An example command that uses an example subsystem. */
public class ShoulderEncoderReset extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shoulder m_shoulder;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShoulderEncoderReset(Shoulder shoulder) {
        m_shoulder = shoulder;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_shoulder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shoulder.resetEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}