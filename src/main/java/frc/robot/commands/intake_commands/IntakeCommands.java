package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
    public static ParallelCommandGroup IntakeIn(Intake m_intake, Ramp m_ramp) {
        return new ParallelCommandGroup(
                new IntakeWheels(m_intake, MechanismConstants.kIntakeInSpeed),
                new RampWheels(m_ramp, MechanismConstants.kRampInSpeed));
    }

    public static ParallelCommandGroup IntakeOut(Intake m_intake, Ramp m_ramp) {
        return new ParallelCommandGroup(
                new IntakeWheels(m_intake, -MechanismConstants.kIntakeOutSpeed),
                new RampWheels(m_ramp, -MechanismConstants.kRampOutSpeed));
    }

    public static ParallelCommandGroup IntakeStop(Intake m_intake, Ramp m_ramp) {
        return new ParallelCommandGroup(
                new IntakeWheels(m_intake, 0),
                new RampWheels(m_ramp, 0));
    }

    public static Command IntakeIn(Intake m_intake) {
        return new IntakeWheels(m_intake, MechanismConstants.kIntakeInSpeed);
    }

    public static Command IntakeOut(Intake m_intake) {
        return new IntakeWheels(m_intake, -MechanismConstants.kIntakeOutSpeed);
    }

    public static Command IntakeStop(Intake m_intake) {
        return new IntakeWheels(m_intake, 0);
    }

    public static Command RampIn(Ramp m_ramp) {
        return new RampWheels(m_ramp, MechanismConstants.kRampInSpeed);
    }

    public static Command RampOut(Ramp m_ramp) {
        return new RampWheels(m_ramp, -MechanismConstants.kRampOutSpeed);
    }

    public static Command RampStop(Ramp m_ramp) {
        return new RampWheels(m_ramp, 0);
    }

    public static Command IntakeUp(Intake m_intake) {
        return new IntakePID(m_intake, MechanismConstants.kIntakeUp);
    }

    public static Command IntakeDown(Intake m_intake) {
        return new IntakePID(m_intake, MechanismConstants.kIntakeDown);
    }
}
