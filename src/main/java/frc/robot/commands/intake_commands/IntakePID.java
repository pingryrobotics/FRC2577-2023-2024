package frc.robot.commands.intake_commands; //CTV

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake; //CTV

/** An example command that uses an example subsystem. */
public class IntakePID extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final double pos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakePID(Intake intake, double pos) {
        m_intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intake);
        this.pos = pos;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.setPosition(pos);
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
        return true; // instantly finishes (change this if you want to manually time it out)
    }
}