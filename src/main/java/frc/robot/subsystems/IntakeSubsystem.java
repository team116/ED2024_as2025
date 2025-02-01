package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private SparkMax intakeMotor1;
    private SparkMax intakeMotor2;

    private SparkMaxConfig intakeMotor1Config = new SparkMaxConfig();
    private SparkMaxConfig intakeMotor2Config = new SparkMaxConfig();

    public IntakeSubsystem() {
        intakeMotor1 = new SparkMax(Constants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.INTAKE_MOTOR_2_ID, MotorType.kBrushless);

        intakeMotor1Config.inverted(true);
        intakeMotor1.configure(intakeMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotor2Config.inverted(true);
        intakeMotor2.configure(intakeMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runRollersToConsume() {
        intakeMotor1.set(1.0);
        intakeMotor2.set(1.0);
    }

    public void runRollersToVomit() {
        intakeMotor1.set(-0.5);
        intakeMotor2.set(-0.5);
    }

    public void stopMotor() {
        intakeMotor1.set(0);
        intakeMotor2.set(0);
    }
}
