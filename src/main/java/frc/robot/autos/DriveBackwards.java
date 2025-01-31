package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDistanceAtAngle;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveBackwards extends SequentialCommandGroup {
    public DriveBackwards(SwerveSubsystem swerveSubsystem) {
        DriveDistanceAtAngle driveBackAndOutOfTheStartingZone = new DriveDistanceAtAngle(swerveSubsystem, 111, DriveDistanceAtAngle.Direction.REVERSE);
        
        addCommands(driveBackAndOutOfTheStartingZone);
    }
}
