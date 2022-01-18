package frc.robot.utils.modifiedswervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;

public final class WPI_TalonFXDriveControllerFactoryBuilder {

  private static final double TICKS_PER_ROTATION = 2048.0;

  private static final int CAN_TIMEOUT_MS = 250;
  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

  private double nominalVoltage = Double.NaN;
  private double currentLimit = Double.NaN;

  public WPI_TalonFXDriveControllerFactoryBuilder withVoltageCompensation(
    double nominalVoltage
  ) {
    this.nominalVoltage = nominalVoltage;
    return this;
  }

  public boolean hasVoltageCompensation() {
    return Double.isFinite(nominalVoltage);
  }

  public DriveControllerFactory<ControllerImplementation, WPI_TalonFX> build() {
    return new FactoryImplementation();
  }

  public WPI_TalonFXDriveControllerFactoryBuilder withCurrentLimit(
    double currentLimit
  ) {
    this.currentLimit = currentLimit;
    return this;
  }

  public boolean hasCurrentLimit() {
    return Double.isFinite(currentLimit);
  }

  private class FactoryImplementation
    implements DriveControllerFactory<ControllerImplementation, WPI_TalonFX> {

    @Override
    public ControllerImplementation create(
      WPI_TalonFX driveConfiguration,
      ModuleConfiguration moduleConfiguration
    ) {
      TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

      double sensorPositionCoefficient =
        Math.PI *
        moduleConfiguration.getWheelDiameter() *
        moduleConfiguration.getDriveReduction() /
        TICKS_PER_ROTATION;
      double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

      if (hasVoltageCompensation()) {
        motorConfiguration.voltageCompSaturation = nominalVoltage;
      }

      if (hasCurrentLimit()) {
        motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
      }

      WPI_TalonFX motor = driveConfiguration;
      CtreUtils.checkCtreError(
        motor.configAllSettings(motorConfiguration),
        "Failed to configure Falcon 500"
      );

      if (hasVoltageCompensation()) {
        // Enable voltage compensation
        motor.enableVoltageCompensation(true);
      }

      motor.setNeutralMode(NeutralMode.Brake);

      motor.setInverted(
        moduleConfiguration.isDriveInverted()
          ? TalonFXInvertType.Clockwise
          : TalonFXInvertType.CounterClockwise
      );
      motor.setSensorPhase(true);

      // Reduce CAN status frame rates
      CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(
          StatusFrameEnhanced.Status_1_General,
          STATUS_FRAME_GENERAL_PERIOD_MS,
          CAN_TIMEOUT_MS
        ),
        "Failed to configure Falcon status frame period"
      );

      return new ControllerImplementation(motor, sensorVelocityCoefficient);
    }
  }

  private class ControllerImplementation implements DriveController {

    private final TalonFX motor;
    private final double sensorVelocityCoefficient;
    private final double nominalVoltage = hasVoltageCompensation()
      ? WPI_TalonFXDriveControllerFactoryBuilder.this.nominalVoltage
      : 12.0;

    private ControllerImplementation(
      TalonFX motor,
      double sensorVelocityCoefficient
    ) {
      this.motor = motor;
      this.sensorVelocityCoefficient = sensorVelocityCoefficient;
    }

    @Override
    public void setReferenceVoltage(double voltage) {
      motor.set(TalonFXControlMode.PercentOutput, voltage / nominalVoltage);
    }

    @Override
    public double getStateVelocity() {
      return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
    }
  }
}
