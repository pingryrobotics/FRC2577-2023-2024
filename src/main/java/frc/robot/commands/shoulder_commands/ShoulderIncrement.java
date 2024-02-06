package frc.robot.commands.shoulder_commands; //CTV

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;

/** An example command that uses an example subsystem. */
public class ShoulderIncrement extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shoulder m_shoulder;
    private double pos;
    private double increment;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShoulderIncrement(Shoulder shoulder, double increment) {
        m_shoulder = shoulder;
        this.increment = increment;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shoulder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.pos = increment + m_shoulder.getShoulderPosition();
        m_shoulder.setDesiredTicks(pos);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println("Shoulder position: " + m_shoulder.getShoulderPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shoulder.setDesiredTicks(m_shoulder.getShoulderPosition());
        m_shoulder.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if (pos - m_shoulder.getShoulderPosition())
        // return false;
        return (Math.abs(pos - m_shoulder.getShoulderPosition()) < 1);
    }
}