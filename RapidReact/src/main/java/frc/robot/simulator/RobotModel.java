package frc.robot.simulator;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Pose2d;

public class RobotModel {

  DrivetrainModel dt;

  final double BATTERY_NOMINAL_VOLTAGE = 13.2; //Nicely charged battery

  double batteryVoltage_V = BATTERY_NOMINAL_VOLTAGE;

  public RobotModel(
    WPI_TalonFX driveMotorFrontLeft,
    WPI_TalonFX driveMotorFrontRight,
    WPI_TalonFX driveMotorBackLeft,
    WPI_TalonFX driveMotorBackRight,
    WPI_TalonFX steerMotorFrontLeft,
    WPI_TalonFX steerMotorFrontRight,
    WPI_TalonFX steerMotorBackLeft,
    WPI_TalonFX steerMotorBackRight
  ) {
    dt =
      new DrivetrainModel(
        driveMotorFrontLeft,
        driveMotorFrontRight,
        driveMotorBackLeft,
        driveMotorBackRight,
        steerMotorFrontLeft,
        steerMotorFrontRight,
        steerMotorBackLeft,
        steerMotorBackRight
      );
    reset(Constants.DFLT_START_POSE);
  }

  public void reset(Pose2d pose) {
    dt.modelReset(pose);
  }

  public void update(boolean isDisabled) {
    long numIter = Math.round(
      Constants.CTRLS_SAMPLE_RATE_SEC / Constants.SIM_SAMPLE_RATE_SEC
    );

    for (long count = 0; count < numIter; count++) {
      dt.update(isDisabled, batteryVoltage_V);
    }
  }

  public Pose2d getCurActPose() {
    return dt.field.getRobotObject().getPose();
  }
}
