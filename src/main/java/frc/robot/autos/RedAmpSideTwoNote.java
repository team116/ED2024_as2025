package frc.robot.autos;

import frc.robot.autos.primitives.DriveDistanceAtAngle.Direction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class RedAmpSideTwoNote extends AmpSideTwoNote {
    public RedAmpSideTwoNote(SwerveSubsystem swerve, Arm arm, IntakeSubsystem intakeSubsystem, Shooter shooter) {
        super(swerve, arm, intakeSubsystem, shooter, Direction.DIAGONAL_BACKWARD_RIGHT);
    }
}
