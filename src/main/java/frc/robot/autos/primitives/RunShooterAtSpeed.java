package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class RunShooterAtSpeed extends DurationCommand {

    private static final double VELOCITY_EPSILON = 0.01;

    private Shooter shooter;
    private double targetSpeed;
    private double motor1Power;
    private double motor2Power;
    private double motor1LastVelocity;
    private double motor2LastVelocity;
    private int motor1CountAtSameVelocity;
    private int motor2CountAtSameVelocity;

    public RunShooterAtSpeed(Shooter shooterSubsystem, double speed, double duration) {
        super(duration);
        shooter = shooterSubsystem;
        this.targetSpeed = speed;
        this.motor1LastVelocity = 0.0;
        this.motor2LastVelocity = 0.0;
        this.motor1CountAtSameVelocity = 0;
        this.motor2CountAtSameVelocity = 0;
        motor1Power = defaultPowerForSpeed(speed);
        motor2Power = defaultPowerForSpeed(speed);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.setMotor1Power(motor1Power);
        shooter.setMotor2Power(motor2Power);
    }

    @Override
    public void execute() {
        super.execute();

        // First motor fun...
        double motor1CurrentVelocity = shooter.getMotor1Velocity();
        if (Math.abs(motor1CurrentVelocity - motor1LastVelocity) < VELOCITY_EPSILON) {
            ++motor1CountAtSameVelocity;
        } else {
            motor1CountAtSameVelocity = 0;
        }

        double motor1SpeedDiff = Math.abs(motor1CurrentVelocity) - targetSpeed;
        if (Math.abs(motor1SpeedDiff) < VELOCITY_EPSILON) {
            // DO NOTHING... right where we want to be!!!!
        } else if (motor1CountAtSameVelocity > 3) {
            if (motor1SpeedDiff < 0.0) {
                motor1Power += 0.01;
            } else {
                motor1Power -= 0.01;
            }
            shooter.setMotor1Power(motor1Power);
        }
        motor1LastVelocity = motor1CurrentVelocity;
        SmartDashboard.putNumber("shooterMotor1Velocity", motor1LastVelocity);
 
        // Second motor fun...
        double motor2CurrentVelocity = shooter.getMotor2Velocity();
        if (Math.abs(motor2CurrentVelocity - motor2LastVelocity) < VELOCITY_EPSILON) {
            ++motor2CountAtSameVelocity;
        } else {
            motor2CountAtSameVelocity = 0;
        }

        double motor2SpeedDiff = Math.abs(motor2CurrentVelocity) - targetSpeed;
        if (Math.abs(motor2SpeedDiff) < VELOCITY_EPSILON) {
            // DO NOTHING... right where we want to be!!!!
        } else if (motor2CountAtSameVelocity > 3) {
            if (motor2SpeedDiff < 0.0) {
                motor2Power += 0.01;
            } else {
                motor2Power -= 0.01;
            }
            shooter.setMotor1Power(motor2Power);
        }
   
        motor2LastVelocity = motor2CurrentVelocity;
        SmartDashboard.putNumber("shooterMotor2Velocity", motor2LastVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    private double defaultPowerForSpeed(double speed) {
        return 0.5; // This should be derived by testing to determine what is usually "close" to target
    }
}
