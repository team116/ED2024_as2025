package frc.robot.autos.primitives;

import frc.robot.subsystems.Shooter;

public class ConfirmShooterSpeed extends DurationCommand {

    private Shooter shooter;
    private double shooterRPMs;
    private boolean atSpeed;

    public ConfirmShooterSpeed(Shooter shooter, double shooterRPMs, double totalDuration) {
        super(totalDuration);
        this.shooter = shooter;
        this.shooterRPMs = shooterRPMs;
    }

    @Override
    public void initialize() {
        super.initialize();
        atSpeed = false;
    }

    @Override
    public void execute() {
        super.execute();
        if (shooter.getMotor1Velocity() >= shooterRPMs) {
            atSpeed = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
    
    @Override
    public boolean isFinished() {
        return atSpeed || super.isFinished();
    }
}
