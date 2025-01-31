package frc.robot.autos;

import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class OneNoteRightAutoAndMoveOut extends OneNoteAutoAndMoveOut {

    public OneNoteRightAutoAndMoveOut(SwerveSubsystem swerve, Arm arm, IntakeSubsystem intakeSubsystem, Shooter shooter, double timeToWaitAtStart) {
        super(swerve, arm, intakeSubsystem, shooter, timeToWaitAtStart, Direction.DIAGONAL_BACKWARD_LEFT);  // Opposite Direction
    }
}
