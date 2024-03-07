// package frc.robot.commands.shooter_commands; //CTV

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter; //CTV

// /** An example command that uses an example subsystem. */
// public class AdjustShooter extends Command {
//     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//     private final Shooter m_shooter;
//     private final double m_speed;

//     /**
//      * Creates a new ExampleCommand.
//      *
//      * @param subsystem The subsystem used by this command.
//      */
//     public AdjustShooter(Shooter shooter, double speed) {
//         m_shooter = shooter;
//         m_speed = speed;
//         // Use addRequirements() here to declare subsystem dependencies.
//         addRequirements(shooter);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         m_shooter.setAdjusterSpeed(m_speed);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {}

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {}

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return true;
//     }
// }