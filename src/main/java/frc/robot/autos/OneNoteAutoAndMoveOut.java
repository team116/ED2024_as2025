package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autos.primitives.DriveDistanceAtAngle;
import frc.robot.autos.primitives.DurationCommand;
import frc.robot.autos.primitives.IntakeCommand;
import frc.robot.autos.primitives.RunShooterAtPowerAndDuration;
import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class OneNoteAutoAndMoveOut extends SequentialCommandGroup {

    public OneNoteAutoAndMoveOut(SwerveSubsystem swerve, Arm arm, IntakeSubsystem intakeSubsystem, Shooter shooter, double timeToWaitAtStart) {
        this(swerve, arm, intakeSubsystem, shooter, timeToWaitAtStart, Direction.REVERSE);
    }

    public OneNoteAutoAndMoveOut(SwerveSubsystem swerve, Arm arm, IntakeSubsystem intakeSubsystem, Shooter shooter, double timeToWaitAtStart, Direction leaveDirection) {
        DurationCommand waitToShoot = new DurationCommand(timeToWaitAtStart);

        RunShooterAtPowerAndDuration runShooterAtPowerAndDuration =
            new RunShooterAtPowerAndDuration(shooter, Constants.SPEAKER_SHOOTING_POWER, 5);

        ParallelCommandGroup shootingSequence = new ParallelCommandGroup(
            runShooterAtPowerAndDuration,
            new SequentialCommandGroup(
                new DurationCommand(2),
                new IntakeCommand(intakeSubsystem, 1.0)
            )
        );

        DriveDistanceAtAngle driveBackASmallDistance = new DriveDistanceAtAngle(swerve, 3, Direction.REVERSE);

        DriveDistanceAtAngle driveOutOfZone = new DriveDistanceAtAngle(swerve, 108, leaveDirection);

        addCommands(
                    new InstantCommand(() -> SmartDashboard.putNumber("autoStart OneNote", Timer.getFPGATimestamp())),
                    waitToShoot,
                    shootingSequence,
                    driveBackASmallDistance,
                    driveOutOfZone,
                    new InstantCommand(() -> SmartDashboard.putNumber("autoFinish OneNote", Timer.getFPGATimestamp()))
        );
    }
}
