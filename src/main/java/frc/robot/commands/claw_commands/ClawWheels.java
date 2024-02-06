package frc.robot.commands.claw_commands; //CTV

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw; //CTV

/** An example command that uses an example subsystem. */
public class ClawWheels extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Claw m_claw;
    private final double power;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ClawWheels(Claw claw, double power) {
        this.power = power;
        m_claw = claw;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_claw.setWheelsSpeed(power);
        m_claw.wheelsMotorActivate();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_claw.setWheelsSpeed(power);
        m_claw.wheelsMotorActivate();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}