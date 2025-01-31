package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooterAtPower extends Command {
    
    
    private Shooter shooter;
    private Joystick gunner;
    private double lastShooterPower;

    public RunShooterAtPower(Shooter shooter, Joystick gunner) {

        this.shooter = shooter;
        this.gunner = gunner;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        lastShooterPower = getShooterPowerFromJoystick();
        shooter.setAllMotorPower(lastShooterPower);
    }

    @Override
    public void execute() {
        super.execute();
        double currentShooterPower = getShooterPowerFromJoystick();
        if(Math.abs(currentShooterPower - lastShooterPower) > 0.1) {
            shooter.setAllMotorPower(currentShooterPower);
            lastShooterPower = currentShooterPower;
        }
    }
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        super.end(interrupted);
    }

    private double getShooterPowerFromJoystick() {
        return -1.0*gunner.getThrottle();
    }
}

