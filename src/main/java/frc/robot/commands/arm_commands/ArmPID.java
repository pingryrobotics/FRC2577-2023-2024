package frc.robot.commands.arm_commands; //CTV

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;

/** An example command that uses an example subsystem. */
public class ArmPID extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm m_arm;
    private double pos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmPID(Arm arm, double pos) {
        m_arm = arm;
        this.pos = pos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.setDesiredTicks(pos);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println("Shoulder position: " + m_arm.getShoulderPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_arm.setDesiredTicks(m_arm.getArmPosition());
        m_arm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if (pos - m_arm.getShoulderPosition())
        // return false;
        return (Math.abs(pos - m_arm.getArmPosition()) < 2);
    }
}