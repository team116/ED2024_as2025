package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.RunIntakeMotorAnyDirection.RollerDirection;
import frc.robot.subsystems.IntakeSubsystem;

public class PrepNoteToShoot extends SequentialCommandGroup {
    public PrepNoteToShoot(IntakeSubsystem intakeSubsystem, double duration) {
        RunIntakeMotorAnyDirection expelNoteALittleBit = new RunIntakeMotorAnyDirection(intakeSubsystem, duration, RollerDirection.EXPEL);

        addCommands(expelNoteALittleBit);
    }

    public PrepNoteToShoot(IntakeSubsystem intakeSubsystem) {
        this(intakeSubsystem, 0.3); // TODO: Find a good time for this
    }
}
