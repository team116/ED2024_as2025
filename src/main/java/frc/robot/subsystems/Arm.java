package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private DutyCycleEncoder armEncoder;

    private SparkMax armRotationMotor;
    //private SparkClosedLoopController armMotorController;

    private SparkLimitSwitch arm1LimitSwitch;
    private SparkLimitSwitch arm2LimitSwitch;

    public Arm() {
/*
        armEncoder = new DutyCycleEncoder(Constants.ARM_ENCODER_CHANNEL, 360.0d, 0.0d);
        armRotationMotor = new SparkMax(Constants.ARM_ROTATION_MOTOR_ID, MotorType.kBrushless);

        armRotationMotor.setIdleMode(IdleMode.kBrake);

        arm1LimitSwitch = armRotationMotor.getForwardLimitSwitch(Type.kNormallyClosed);
        arm2LimitSwitch = armRotationMotor.getReverseLimitSwitch(Type.kNormallyClosed);

        arm1LimitSwitch.enableLimitSwitch(true);
        arm2LimitSwitch.enableLimitSwitch(true);

        //armMotorController = armRotationMotor.getClosedLoopController();

        armRotationMotor.setInverted(true); //Originally  was false 

        enableLimitSwitches();

        //armEncoder.setDistancePerRotation(360);  NOTE: Done through constructor now

        armRotationMotor.burnFlash();  // FIXME: configure with all SparkBaseConfig settings...
*/
    }

    public void moveUp() {
        armRotationMotor.set(0.65); // TODO: Find a good speed for this
    }

    public void moveUpSlow() {
        armRotationMotor.set(0.1);
    }

    public void moveDown() {
        armRotationMotor.set(-0.65); // TODO: Find a good speed for this
    }

    public void moveDownSlow() {
        armRotationMotor.set(-0.1);
    }

    public void stop() {
        armRotationMotor.set(0);
    }

    public void move(double power) {
        armRotationMotor.set(power);
    }

    public void enableLimitSwitches() {
        // FIXME: We don't care about arm
        // arm1LimitSwitch.enableLimitSwitch(true);
        // arm2LimitSwitch.enableLimitSwitch(true);
    }

    public void disableLimitSwitches() {
        // FIXME: still don't care about arm
        // arm1LimitSwitch.enableLimitSwitch(false);
        // arm2LimitSwitch.enableLimitSwitch(false);
    }

    public double getAngleDegrees() {
        //return armEncoder.getDistance();
        return armEncoder.get();   // FIXME: Need to double check that get() will return up to 360.0 as expected
    }
}
