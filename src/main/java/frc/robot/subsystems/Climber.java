package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
   private SparkMax climberMotor;
   private SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
   
   public Climber() {
      climberMotor = new SparkMax(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

      climberMotorConfig.inverted(true);
      climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   public void pullUp() {
      climberMotor.set(-1.0);
   }

   public void pullUpSlow() {
      climberMotor.set(-0.5);
   }

   public void stop() {
      climberMotor.set(0);
   }
}
