package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.RunIntakeMotorAnyDirection.RollerDirection;
import frc.robot.subsystems.IntakeSubsystem;

public class FeedNoteToShooter extends SequentialCommandGroup {
    public FeedNoteToShooter(IntakeSubsystem intakeSubsystem, double duration) {
        RunIntakeMotorAnyDirection feedNoteToShooter = new RunIntakeMotorAnyDirection(intakeSubsystem, duration, RollerDirection.CONSUME);

        addCommands(feedNoteToShooter);
    }

    public FeedNoteToShooter(IntakeSubsystem intakeSubsystem) {
        this(intakeSubsystem, 1.0);
    }
}
