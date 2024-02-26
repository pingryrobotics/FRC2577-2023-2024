package frc.robot.commands.shooter_commands; //CTV

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class ShooterPID extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;
    private double pos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShooterPID(Shooter shooter, double pos) {
        m_shooter = shooter;
        this.pos = pos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooter.setPosition(pos);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println("Shoulder position: " + m_shooter.getShoulderPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setPosition(m_shooter.getPosition());
        m_shooter.adjusterOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(pos - m_shooter.getPosition()) < 1);
    }
}