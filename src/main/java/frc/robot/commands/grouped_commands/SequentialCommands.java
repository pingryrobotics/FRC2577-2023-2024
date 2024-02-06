package frc.robot.commands.grouped_commands; //CTV

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm; //CTV
import frc.robot.Constants;
import frc.robot.commands.arm_commands.ArmPID;
import frc.robot.commands.claw_commands.ClawWheels;
import frc.robot.commands.drive_commands.DriveForward;
import frc.robot.commands.drive_commands.DriveForwardGyro;
import frc.robot.commands.shoulder_commands.ShoulderIncrement;
import frc.robot.commands.shoulder_commands.ShoulderPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shoulder;

/** An example command that uses an example subsystem. */
public class SequentialCommands {
    public static SequentialCommandGroup autoEject(DriveSubsystem m_robotDrive, Claw m_claw, Shoulder m_shoulder, Arm m_arm) {
        return new SequentialCommandGroup(
            new RunCommand(
                () -> m_claw.close()
            ).withTimeout(0.5),
            new ShoulderPID(m_shoulder, 76).withTimeout(2),
            new WaitCommand(0.5),
            new ArmPID(m_arm, Constants.MechanismConstants.kArmFullExtension).withTimeout(2),
            new ClawWheels(m_claw, 0.55),
            new WaitCommand(0.5),
            new ClawWheels(m_claw, 0),
            new ArmPID(m_arm, 0).withTimeout(2),
            new ShoulderPID(m_shoulder, 0).withTimeout(2)
        );
    }
    public static SequentialCommandGroup fullyRetract(DriveSubsystem m_robotDrive, Claw m_claw, Shoulder m_shoulder, Arm m_arm) {
        return new SequentialCommandGroup(
            // negative is forward
            // new ShoulderIncrement(m_shoulder, -5).withTimeout(2),
            new ArmPID(m_arm, 0).withTimeout(3),
            new ShoulderPID(m_shoulder, 0).withTimeout(4)
        );
    }

    public static SequentialCommandGroup fullyExtend(DriveSubsystem m_robotDrive, Claw m_claw, Shoulder m_shoulder, Arm m_arm) {
        return new SequentialCommandGroup(
            new ShoulderPID(m_shoulder, Constants.MechanismConstants.kshoulderHighPosition).withTimeout(4),
            new ArmPID(m_arm, Constants.MechanismConstants.kArmFullExtension).withTimeout(3)
            
        );
    }
}