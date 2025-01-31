package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.RunIntakeMotorAnyDirection.RollerDirection;
import frc.robot.subsystems.IntakeSubsystem;

public class OutTakeCommand extends SequentialCommandGroup {
    public OutTakeCommand(IntakeSubsystem intakeSubsystem, double duration) {
        RunIntakeMotorAnyDirection intakeNote = new RunIntakeMotorAnyDirection(intakeSubsystem, duration, RollerDirection.EXPEL);

        addCommands(intakeNote);
    }
    
}