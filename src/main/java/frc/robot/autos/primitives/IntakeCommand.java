package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.RunIntakeMotorAnyDirection.RollerDirection;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(IntakeSubsystem intakeSubsystem, double duration) {
        RunIntakeMotorAnyDirection intakeNote = new RunIntakeMotorAnyDirection(intakeSubsystem, duration, RollerDirection.CONSUME);

        addCommands(intakeNote);
    }
}
