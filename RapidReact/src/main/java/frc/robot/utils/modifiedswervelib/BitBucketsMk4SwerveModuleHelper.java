package frc.robot.utils.modifiedswervelib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SteerControllerFactory;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.SwerveModuleFactory;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class BitBucketsMk4SwerveModuleHelper {

  private BitBucketsMk4SwerveModuleHelper() {}

  private static DriveControllerFactory<?, WPI_TalonFX> getWPI_TalonFXDriveFactory(
    Mk4ModuleConfiguration configuration
  ) {
    return new WPI_TalonFXDriveControllerFactoryBuilder()
      .withVoltageCompensation(configuration.getNominalVoltage())
      .withCurrentLimit(configuration.getDriveCurrentLimit())
      .build();
  }

  private static SteerControllerFactory<?, WPI_TalonFXSteerConfiguration<BitBucketsCanCoderAbsoluteConfiguration>> getWPI_TalonFXSteerFactory(
    Mk4ModuleConfiguration configuration
  ) {
    return new WPI_TalonFXSteerControllerFactoryBuilder()
      .withVoltageCompensation(configuration.getNominalVoltage())
      .withPidConstants(0.2, 0.0, 0.1) //  original
      .withCurrentLimit(configuration.getSteerCurrentLimit())
      .build(new BitBucketsCanCoderFactoryBuilder().withReadingUpdatePeriod(100).build());
  }

  /**
   * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
   * Module information is displayed in the specified ShuffleBoard container.
   *
   * @param container        The container to display module information in.
   * @param configuration    Module configuration parameters to use.
   * @param gearRatio        The gearing configuration the module is in.
   * @param driveMotor       The Falcon 500 drive motor.
   * @param steerMotor       The instance of the WPI_TalonFX to use
   * @param steerEncoder     The steer CANCoder.
   * @param steerOffset      The offset of the CANCoder in radians.
   * @return The configured swerve module.
   */
  public static SwerveModule createWPI_TalonFX(
    ShuffleboardLayout container,
    Mk4ModuleConfiguration configuration,
    GearRatio gearRatio,
    WPI_TalonFX driveMotor,
    WPI_TalonFX steerMotor,
    CANCoder steerEncoder,
    double steerOffset
  ) {
    return new SwerveModuleFactory<>(
      gearRatio.getConfiguration(),
      getWPI_TalonFXDriveFactory(configuration),
      getWPI_TalonFXSteerFactory(configuration)
    )
    .create(
        container,
        driveMotor,
        new WPI_TalonFXSteerConfiguration<>(
          steerMotor,
          new BitBucketsCanCoderAbsoluteConfiguration(steerEncoder, steerOffset)
        )
      );
  }

  /**
   * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
   * Module information is displayed in the specified ShuffleBoard container.
   *
   * @param container        The container to display module information in.
   * @param gearRatio        The gearing configuration the module is in.
   * @param driveMotor       The Falcon 500 drive motor.
   * @param steerMotor       The Falcon 500 steer motor.
   * @param steerEncoder     The steer CANCoder.
   * @param steerOffset      The offset of the CANCoder in radians.
   * @return The configured swerve module.
   */
  public static SwerveModule createWPI_TalonFX(
    ShuffleboardLayout container,
    GearRatio gearRatio,
    WPI_TalonFX driveMotor,
    WPI_TalonFX steerMotor,
    CANCoder steerEncoder,
    double steerOffset
  ) {
    return createWPI_TalonFX(
      container,
      new Mk4ModuleConfiguration(),
      gearRatio,
      driveMotor,
      steerMotor,
      steerEncoder,
      steerOffset
    );
  }

  public enum GearRatio {
    L1(SdsModuleConfigurations.MK4_L1),
    L2(SdsModuleConfigurations.MK4_L2),
    L3(SdsModuleConfigurations.MK4_L3),
    L4(SdsModuleConfigurations.MK4_L4);

    private final ModuleConfiguration configuration;

    GearRatio(ModuleConfiguration configuration) {
      this.configuration = configuration;
    }

    public ModuleConfiguration getConfiguration() {
      return configuration;
    }
  }
}
