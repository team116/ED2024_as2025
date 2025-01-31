package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DurationCommand;
import frc.robot.autos.primitives.FeedNoteToShooter;
import frc.robot.autos.primitives.MonitorCommand;
import frc.robot.autos.primitives.MoveArmToAngle;
import frc.robot.autos.primitives.RunShooterAtSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class KnownPosSpeakerMacro extends ParallelCommandGroup {
    KnownPosSpeakerMacro(IntakeSubsystem intakeSubsystem, Shooter shooter, Arm arm, double shooterSpeed, double angle) {
        super(
            new ParallelDeadlineGroup(
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new MonitorCommand(shooter, arm, angle, shooterSpeed),
                        new FeedNoteToShooter(intakeSubsystem),
                        new DurationCommand(0.25)
                        ),
                    new RunShooterAtSpeed(shooter, shooterSpeed, Double.POSITIVE_INFINITY)
                ),
                new MoveArmToAngle(arm, angle, 3.0)
            )
        );
    }
    
}
