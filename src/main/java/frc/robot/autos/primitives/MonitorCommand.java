package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class MonitorCommand extends Command {
    private static final double ANGLE_EPSILON = 0.1; // TODO: Find correct angle for this
    private static final double SPEED_EPSILON_VELOCITY = 0.1; // TODO: Find correct unit for this and correct number
    private Shooter shooter;
    private Arm arm;
    private boolean isFinished;
    private double targetAngle;
    private double targetSpeed;

    public MonitorCommand(Shooter shooter, Arm arm, double targetAngle, double targetSpeed) {
        this.shooter = shooter;
        this.arm = arm;
        this.targetAngle = targetAngle;
        this.targetSpeed = targetSpeed;
    }

    @Override
    public void initialize() {
        super.initialize();
        isFinished = false;
    }

    @Override
    public void execute() {
        super.execute();

        double currentAngle = arm.getAngleDegrees();
        double currentMotorSpeed = shooter.getMotor1Velocity();

        double angleDiff  = Math.abs(targetAngle - currentAngle);
        double speedDiff = Math.abs(targetSpeed - currentMotorSpeed);
        if ((angleDiff < ANGLE_EPSILON) && (speedDiff < SPEED_EPSILON_VELOCITY)) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished || super.isFinished();
    }
}
