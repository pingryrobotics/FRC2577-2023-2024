package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    public static SequentialCommandGroup ShootAndIndex(Shooter m_shooter, Indexer m_indexer) {
        return new SequentialCommandGroup(
                new Shoot(m_shooter, MechanismConstants.kShooterSpeed),
                new WaitCommand(2),
                new Index(m_indexer, MechanismConstants.kIndexerSpeed),
                new WaitCommand(1),
                new Index(m_indexer, 0),
                new Shoot(m_shooter, 0));
    }

    public static Command Shoot(Shooter m_shooter) {
        return new Shoot(m_shooter, MechanismConstants.kShooterSpeed);
    }

    public static Command Index(Indexer m_indexer) {
        return new Index(m_indexer, MechanismConstants.kIndexerSpeed);
    }

    public static Command StopShooter(Shooter m_shooter) {
        return new Shoot(m_shooter, 0);
    }

    public static Command StopIndexer(Indexer m_indexer) {
        return new Index(m_indexer, 0);
    }

    public static Command AdjustShooterHigh(Shooter m_shooter) {
        return new ShooterPID(m_shooter, MechanismConstants.kShooterAdjusterHigh);
    }

    public static Command AdjustShooterLow(Shooter m_shooter) {
        return new ShooterPID(m_shooter, MechanismConstants.kShooterAdjusterLow);
    }

    public static Command AdjustShooterMid(Shooter m_shooter) {
        return new ShooterPID(m_shooter, MechanismConstants.kShooterAdjusterMid);
    }
}
