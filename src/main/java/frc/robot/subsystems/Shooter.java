package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.net.http.HttpClient.Redirect;
import java.util.Locale.IsoCountryCode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private SparkMax shooterMotor1;  // TODO: Might be "closer"/"further", find out when installed
    private SparkMax shooterMotor2;

    private SparkMaxConfig shooterMotor1Config = new SparkMaxConfig();
    private SparkMaxConfig shooterMotor2Config = new SparkMaxConfig();

    private RelativeEncoder shooter1Encoder;
    private RelativeEncoder shooter2Encoder;

    public Shooter() {
        shooterMotor1 = new SparkMax(Constants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
        shooterMotor2 = new SparkMax(Constants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);

        shooterMotor1Config.idleMode(IdleMode.kBrake);
        shooterMotor1.configure(shooterMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterMotor2Config
        .idleMode(IdleMode.kBrake)
        .inverted(true);
        shooterMotor2.configure(shooterMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooter1Encoder = shooterMotor1.getEncoder();
        shooter2Encoder = shooterMotor2.getEncoder();
    }

    public double getMotor1Velocity() {
        return shooter1Encoder.getVelocity();
    }

    public double getMotor2Velocity() {
        return shooter2Encoder.getVelocity();
    }

    public void setAllMotorPower(double percentagePower) {
        setMotor1Power(percentagePower);
        setMotor2Power(percentagePower);
    }

    public void setMotor1Power(double percentagePower) {
        shooterMotor1.set(percentagePower);
    }

    public void setMotor2Power(double percentagePower) {
        shooterMotor2.set(percentagePower);
    }

    public void stop() {
        shooterMotor1.set(0.0);
        shooterMotor2.set(0.0);
    }
}
