package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private DutyCycleEncoder armEncoder;

    private SparkMax armRotationMotor;
    private SparkMaxConfig armRotationMotorConfig = new SparkMaxConfig();
    //private SparkClosedLoopController armMotorController;

    private SparkLimitSwitch arm1LimitSwitch;
    private SparkLimitSwitch arm2LimitSwitch;

    public Arm() {

        armEncoder = new DutyCycleEncoder(Constants.ARM_ENCODER_CHANNEL, 360.0d, 0.0d);
        armRotationMotor = new SparkMax(Constants.ARM_ROTATION_MOTOR_ID, MotorType.kBrushless);

        armRotationMotorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        armRotationMotorConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchType(Type.kNormallyClosed)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchEnabled(true);

        //armMotorController = armRotationMotor.getClosedLoopController();

        //enableLimitSwitches();

        //armEncoder.setDistancePerRotation(360);  NOTE: Done through constructor now

        //armRotationMotor.burnFlash();  // FIXME: configure with all SparkBaseConfig settings...
        armRotationMotor.configure(armRotationMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        armRotationMotorConfig.limitSwitch
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchEnabled(true);

        // WARNING: No thread safety on armRotationMotorConfig assignments here...
        armRotationMotor.configure(armRotationMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void disableLimitSwitches() {
        // FIXME: still don't care about arm
        // arm1LimitSwitch.enableLimitSwitch(false);
        // arm2LimitSwitch.enableLimitSwitch(false);

        armRotationMotorConfig.limitSwitch
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(false);

        // WARNING: No thread safety on armRotationMotorConfig assignments here...
        armRotationMotor.configure(armRotationMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getAngleDegrees() {
        //return armEncoder.getDistance();
        return armEncoder.get();   // FIXME: Need to double check that get() will return up to 360.0 as expected
    }
}
