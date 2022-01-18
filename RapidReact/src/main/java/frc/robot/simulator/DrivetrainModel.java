package frc.robot.simulator;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.simulator.swerve.QuadSwerveSim;
import frc.robot.simulator.swerve.SwerveModuleSim;
import java.util.ArrayList;

class DrivetrainModel {

  QuadSwerveSim swerveDt;
  ArrayList<SwerveModuleSim> modules = new ArrayList<>(
    QuadSwerveSim.NUM_MODULES
  );

  ArrayList<WPI_TalonFX> azmthMotorControllers = new ArrayList<>(
    QuadSwerveSim.NUM_MODULES
  );
  ArrayList<WPI_TalonFX> wheelMotorControllers = new ArrayList<>(
    QuadSwerveSim.NUM_MODULES
  );

  SimGyroSensorModel gyro;

  Field2d field;
  Pose2d endPose;

  static SwerveModuleSim swerveModuleFactory() {
    return new SwerveModuleSim(
      DCMotor.getFalcon500(1),
      DCMotor.getFalcon500(1),
      SdsModuleConfigurations.MK4_L2.getWheelDiameter() / 2,
      1 / SdsModuleConfigurations.MK4_L2.getSteerReduction(),
      1 / SdsModuleConfigurations.MK4_L2.getDriveReduction(),
      1.0,
      1.0,
      1.1,
      0.8,
      Constants.ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES,
      0.01
    );
  }

  public DrivetrainModel(
    WPI_TalonFX driveMotorFrontLeft,
    WPI_TalonFX driveMotorFrontRight,
    WPI_TalonFX driveMotorBackLeft,
    WPI_TalonFX driveMotorBackRight,
    WPI_TalonFX steerMotorFrontLeft,
    WPI_TalonFX steerMotorFrontRight,
    WPI_TalonFX steerMotorBackLeft,
    WPI_TalonFX steerMotorBackRight
  ) {
    modules.add(swerveModuleFactory()); //FL
    modules.add(swerveModuleFactory()); //FR
    modules.add(swerveModuleFactory()); //BL
    modules.add(swerveModuleFactory()); //BR

    azmthMotorControllers.add(steerMotorFrontLeft);
    azmthMotorControllers.add(steerMotorFrontRight);
    azmthMotorControllers.add(steerMotorBackLeft);
    azmthMotorControllers.add(steerMotorBackRight);

    wheelMotorControllers.add(driveMotorFrontLeft);
    wheelMotorControllers.add(driveMotorFrontRight);
    wheelMotorControllers.add(driveMotorBackLeft);
    wheelMotorControllers.add(driveMotorBackRight);

    gyro = new SimGyroSensorModel();

    field = PoseTelemetry.field;
    field.setRobotPose(Constants.DFLT_START_POSE);
    endPose = Constants.DFLT_START_POSE;

    swerveDt =
      new QuadSwerveSim(
        Constants.WHEEL_BASE_WIDTH_M,
        Constants.WHEEL_BASE_WIDTH_M,
        Constants.ROBOT_MASS_kg,
        Constants.ROBOT_MOI_KGM2,
        modules
      );
  }

  /**
   * Handles discontinuous jumps in robot pose. Used at the start of
   * autonomous, if the user manually drags the robot across the field in the
   * Field2d widget, or something similar to that.
   * @param pose
   */
  public void modelReset(Pose2d pose) {
    field.setRobotPose(pose);
    swerveDt.modelReset(pose);
    gyro.resetToPose(pose);

    wheelMotorControllers.forEach(
      m -> m.getSimCollection().setIntegratedSensorRawPosition(0)
    );
    azmthMotorControllers.forEach(
      m -> m.getSimCollection().setIntegratedSensorRawPosition(0)
    );
  }

  /**
   * Advance the simulation forward by one step
   * @param isDisabled
   * @param batteryVoltage
   */
  public void update(boolean isDisabled, double batteryVoltage) {
    // Check if the user moved the robot with the Field2D
    // widget, and reset the model if so.
    Pose2d startPose = field.getRobotPose();
    Transform2d deltaPose = startPose.minus(endPose);
    if (
      deltaPose.getRotation().getDegrees() > 0.01 ||
      deltaPose.getTranslation().getNorm() > 0.01
    ) {
      modelReset(startPose);
    }

    // Calculate and update input voltages to each motor.
    if (isDisabled) {
      for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
        modules.get(idx).setInputVoltages(0.0, 0.0);
      }
    } else {
      for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
        double azmthVolts =
          azmthMotorControllers
            .get(idx)
            .getSimCollection()
            .getMotorOutputLeadVoltage() *
          batteryVoltage *
          -1;
        double wheelVolts =
          wheelMotorControllers
            .get(idx)
            .getSimCollection()
            .getMotorOutputLeadVoltage() *
          batteryVoltage *
          -1;
        modules.get(idx).setInputVoltages(wheelVolts, azmthVolts);
      }
    }

    //Update the main drivetrain plant model
    swerveDt.update(Constants.SIM_SAMPLE_RATE_SEC);
    endPose = swerveDt.getCurPose();

    // Update each encoder
    for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
      double azmthPos = modules.get(idx).getAzimuthEncoderPositionRev();
      double wheelPos = modules.get(idx).getWheelEncoderPositionRev();
      azmthMotorControllers
        .get(idx)
        .getSimCollection()
        .setIntegratedSensorRawPosition((int) (azmthPos * 2048));
      wheelMotorControllers
        .get(idx)
        .getSimCollection()
        .setIntegratedSensorRawPosition((int) (wheelPos * 2048));
    }

    // Update associated devices based on drivetrain motion
    field.setRobotPose(endPose);
    gyro.update(startPose, endPose);
  }
}
