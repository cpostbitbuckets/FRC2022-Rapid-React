package frc.robot.utils.modifiedswervelib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class WPI_TalonFXSteerConfiguration<EncoderConfiguration> {

  private final WPI_TalonFX motor;
  private final EncoderConfiguration encoderConfiguration;

  public WPI_TalonFXSteerConfiguration(
    WPI_TalonFX motor,
    EncoderConfiguration encoderConfiguration
  ) {
    this.motor = motor;
    this.encoderConfiguration = encoderConfiguration;
  }

  public WPI_TalonFX getMotor() {
    return motor;
  }

  public EncoderConfiguration getEncoderConfiguration() {
    return encoderConfiguration;
  }
}
