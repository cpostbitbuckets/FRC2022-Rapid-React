package frc.robot.simulator;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;

public class SimGyroSensorModel {

  public void resetToPose(Pose2d resetPose) {
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(
      SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")
    );
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-resetPose.getRotation().getDegrees(), 360));
  }

  public void update(Pose2d startPose, Pose2d endPose) {
    // Pass our model of what the sensor would be measuring back into the simGyro object
    // for the embedded code to interact with.
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(
      SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")
    );
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-endPose.getRotation().getDegrees(), 360));
  }
}
