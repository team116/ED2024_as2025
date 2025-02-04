package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;

@SuppressWarnings(value = { "removal" })
public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
     // Configure the CANcoder for basic use
    swerveCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0d;
    swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
 
  }
}
