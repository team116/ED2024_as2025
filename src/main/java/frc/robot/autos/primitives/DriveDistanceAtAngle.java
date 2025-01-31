package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveDistanceAtAngle extends SequentialCommandGroup {

    public enum Direction {
        FORWARD(0.0d, 1.0),
        REVERSE(0.0d, -1.0),
        LEFT(90.0d, 1.0),
        RIGHT(90.0d, -1.0),
        DIAGONAL_FORWARD_LEFT(45.0d, 1.0),
        DIAGONAL_FORWARD_RIGHT(-45.0d, 1.0),
        DIAGONAL_BACKWARD_LEFT(-45.0d, -1.0),
        DIAGONAL_BACKWARD_RIGHT(45.0d, -1.0),
        SHALLOW_DIAGONAL_FORWARD_LEFT(22.5d, 1.0),
        SHALLOW_DIAGONAL_FORWARD_RIGHT(-22.5d, 1.0),
        SHALLOW_DIAGONAL_BACKWARD_LEFT(-22.5d, -1.0),
        SHALLOW_DIAGONAL_BACKWARD_RIGHT(22.5d, -1.0);

        public final double angleDegrees;
        public final double directionMultiplier;

        private Direction(double angleDegrees, double directionMultiplier) {
            this.angleDegrees = angleDegrees;
            this.directionMultiplier = directionMultiplier;
        }

        public Direction getInverse() {
            switch(this) {
                case FORWARD: return REVERSE;
                case REVERSE: return FORWARD;
                case LEFT:    return RIGHT;
                case RIGHT:   return LEFT;
                case DIAGONAL_FORWARD_LEFT: return DIAGONAL_BACKWARD_RIGHT;
                case DIAGONAL_BACKWARD_RIGHT: return DIAGONAL_FORWARD_LEFT;
                case DIAGONAL_FORWARD_RIGHT: return DIAGONAL_BACKWARD_LEFT;
                case DIAGONAL_BACKWARD_LEFT: return DIAGONAL_FORWARD_RIGHT;
                default: return REVERSE;
            }
        }
    }

    public enum Speed {
        SLOW(DriveDistance.SLOW_PID_SLOT),
        NORMAL(DriveDistance.NORMAL_PID_SLOT),
        FAST(DriveDistance.FAST_PID_SLOT);

        private final int driveDistancePidSlot;

        private Speed(int pidSlot) {
            this.driveDistancePidSlot = pidSlot;
        }

        public int getDriveDistancePidSlot() {
            return driveDistancePidSlot;
        }
    }


    /**
     * The constructor to utilize to move the robot in the indicated direction for the distance specified in inches.
     *
     * @param swerveSubsystem the swerve subsystem
     * @param distanceInches a positive value for the distance to drive
     * @param direction the directioon to move
     */
    public DriveDistanceAtAngle(SwerveSubsystem swerveSubsystem, double distanceInches, Direction direction) {
        this(swerveSubsystem, distanceInches * direction.directionMultiplier, direction.angleDegrees, Speed.NORMAL);
    }

    /**
     * The constructor to utilize to move the robot in the indicated direction for the distance specified in inches.
     * 
     * @param swerveSubsystem the swerve subsystem
     * @param distanceInches a positive value for the distance to drive
     * @param direction the directioon to move
     */
    public DriveDistanceAtAngle(SwerveSubsystem swerveSubsystem, double distanceInches, Direction direction, Speed speed) {
        this(swerveSubsystem, distanceInches * direction.directionMultiplier, direction.angleDegrees, speed);
    }

    /**
     * Constructor that really shouldn't need to be used, as Direction based constructor is preferred.
     * 
     * @param swerveSubsystem the swerve subsystem
     * @param distanceInches positive and negative distance in inches
     * @param angleDegrees the direction to move
     */
    public DriveDistanceAtAngle(SwerveSubsystem swerveSubsystem, double distanceInches, double angleDegrees, Speed speed) {
        TurnWheelsToAngle turnWheelsToAngle = new TurnWheelsToAngle(swerveSubsystem, angleDegrees);
        DriveDistance driveDistance = new DriveDistance(swerveSubsystem, distanceInches, speed.driveDistancePidSlot);

        addCommands(
            turnWheelsToAngle,
            driveDistance
        );
    }
}
