package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class RunShooterAtPowerAndDuration extends DurationCommand {

    private static final double VELOCITY_EPSILON = 0.01;

    private Shooter shooter;
    private double targetPower;

    public RunShooterAtPowerAndDuration(Shooter shooterSubsystem, double power, double duration) {
        super(duration);
        shooter = shooterSubsystem;
        this.targetPower = power;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.setMotor1Power(targetPower);
        shooter.setMotor2Power(targetPower);
    }

    @Override
    public void execute() {
        super.execute();
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

}
