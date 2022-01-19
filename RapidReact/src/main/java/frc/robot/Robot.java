// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SPIAccelerometerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;
import frc.robot.log.LogTestSubsystem;
import frc.robot.simulator.PoseTelemetry;
import frc.robot.simulator.RobotModel;
import frc.robot.simulator.SimulatorTestSubsystem;
import frc.robot.simulator.swerve.QuadSwerveSim;
import frc.robot.simulator.swerve.SwerveModuleSim;
import frc.robot.subsystem.BitBucketsSubsystem;
import frc.robot.subsystem.DrivetrainSubsystem;
import frc.robot.utils.MathUtils;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Buttons buttons;
  private Config config;

  private final List<BitBucketsSubsystem> robotSubsystems = new ArrayList<>();


  // Assumed starting location of the robot. Auto routines will pick their own location and update this.
  public static final Pose2d DFLT_START_POSE = new Pose2d(
    Units.feetToMeters(24.0),
    Units.feetToMeters(10.0),
    Rotation2d.fromDegrees(0)
  );

  // Simple robot plant model for simulation purposes
  RobotModel simModel;

  private DrivetrainSubsystem drivetrainSubsystem;

  PoseTelemetry dtPoseView;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    this.config = new Config();
    this.buttons = new Buttons();

    dtPoseView = new PoseTelemetry();

    //Add Subsystems Here
    this.robotSubsystems.add(
        drivetrainSubsystem = new DrivetrainSubsystem(this.config)
      );

    drivetrainSubsystem.setDefaultCommand(
      new DefaultDriveCommand(
        drivetrainSubsystem,
        () ->
          -MathUtils.modifyAxis(
            buttons.driverControl.getRawAxis(buttons.SwerveForward)
          ),
        () ->
          -MathUtils.modifyAxis(
            buttons.driverControl.getRawAxis(buttons.SwerveStrafe)
          ),
        () ->
          -MathUtils.modifyAxis(
            buttons.driverControl.getRawAxis(buttons.SwerveRotation)
          )
      )
    );

    // Configure the button bindings
    this.configureButtonBindings();

    //Subsystem Initialize Loop
    if (System.getenv().containsKey("CI")) {
      this.robotSubsystems.add(new LogTestSubsystem(this.config));
      this.robotSubsystems.add(new SimulatorTestSubsystem(this.config));
    }

    //Subsystem Initialize Loop
    this.robotSubsystems.forEach(BitBucketsSubsystem::init);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    this.robotSubsystems.forEach(BitBucketsSubsystem::periodic);
    updateTelemetry();
  }

  private void updateTelemetry() {
    if (isSimulation()) {
      dtPoseView.setActualPose(simModel.getCurActPose());
    }
    //    dtPoseView.setEstimatedPose(dtpe.getEstPose());
    //    dtPoseView.setDesiredPose(dt.getCurDesiredPose());
    //
    //    curBatVoltage = pdp.getVoltage();
    //    curBatCurDraw = pdp.getTotalCurrent();

    dtPoseView.update(Timer.getFPGATimestamp() * 1000);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    drivetrainSubsystem
      .logger()
      .logString(LogLevel.GENERAL, "info", "auton started");

    //Reset simulation model to zero state.
    if (isSimulation()) {
      simModel.reset(DFLT_START_POSE);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    drivetrainSubsystem
      .logger()
      .logString(LogLevel.GENERAL, "info", "still in auton!!");
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    this.robotSubsystems.forEach(BitBucketsSubsystem::disable);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    if (isSimulation()) {
      simModel =
        new RobotModel(
          drivetrainSubsystem.driveMotorFrontLeft,
          drivetrainSubsystem.driveMotorFrontRight,
          drivetrainSubsystem.driveMotorBackLeft,
          drivetrainSubsystem.driveMotorBackRight,
          drivetrainSubsystem.steerMotorFrontLeft,
          drivetrainSubsystem.steerMotorFrontRight,
          drivetrainSubsystem.steerMotorBackLeft,
          drivetrainSubsystem.steerMotorBackRight,
          drivetrainSubsystem.canCoderFrontLeft,
          drivetrainSubsystem.canCoderFrontRight,
          drivetrainSubsystem.canCoderBackLeft,
          drivetrainSubsystem.canCoderBackRight
        );
      simModel.reset(DFLT_START_POSE);
    }

  }

  @Override
  public void simulationPeriodic() {
    this.robotSubsystems.forEach(BitBucketsSubsystem::simulationPeriodic);

    simModel.update(isDisabled());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    buttons.zeroGyroscope.whenPressed(drivetrainSubsystem::zeroGyroscope);
  }
}
