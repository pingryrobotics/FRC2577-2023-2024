package frc.robot.commands.drive_commands; //CTV

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem; //CTV
import com.kauailabs.navx.frc.AHRS;

/** An example command that uses an example subsystem. */
public class CheckGyro extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_subsystem;
    private final AHRS m_gyro;
    private boolean tilted = false;
    private boolean straight = false;
    private double initGyroPos;
    private double targetTime;
    private boolean isDriving;
    private boolean timerOn = false;
    private double startMatchTime;
    

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public CheckGyro(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        m_gyro = subsystem.m_gyro;
        isDriving = false;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_gyro.reset();
        initGyroPos = m_gyro.getRoll();
        m_subsystem.drive(0.0, 0, 0, false, false);
        m_subsystem.setX();
        targetTime = 1e18;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (currTime.get
        if (isDriving && System.nanoTime() > targetTime) {
            isDriving = false;
            m_subsystem.drive(0.0, 0, 0, false, false);
            m_subsystem.setX();
        }   
        if (!isDriving && (Math.abs(m_gyro.getRoll()) > 5)) {
            // started the tilt
            m_subsystem.drive(-0.1, 0, 0, false, false);
            // timerOn = true;
            targetTime = System.nanoTime() + 1e9/5;
            isDriving = true;
            // tilted = true;
        } else if (!isDriving) {
            m_subsystem.drive(0.0, 0, 0, false, false);
            m_subsystem.setX();
            straight = true;
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.drive(0.0, 0, 0, false, false);
        m_subsystem.setX();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return tilted && straight;
        return false;
    }
}