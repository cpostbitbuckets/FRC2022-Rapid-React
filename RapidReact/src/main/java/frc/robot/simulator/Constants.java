package frc.robot.simulator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.MathUtils;
import java.util.Arrays;
import java.util.List;

public class Constants {

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////  Configured Constants
  //// - These are tied to the specifics on how your mechanical team,
  ////   electrical team, and the game design committee did their work.
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ROBOT PHYSICAL CONSTANTS
  // Robot physical dimensions and mass quantities.
  public static final double WHEEL_BASE_WIDTH_M = Units.feetToMeters(2.0);
  public static final double WHEEL_RADIUS_IN = 6.0 / 2.0; //six inch (diameter) wheels
  public static final double ROBOT_MASS_kg = MathUtils.lbsToKg(140);
  public static final double ROBOT_MOI_KGM2 =
    1.0 / 12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_WIDTH_M * 1.1), 2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center
  // Location of vision camera relative to robot center - currently front middle.
  public static final Transform2d robotToCameraTrans = new Transform2d(
    new Translation2d(WHEEL_BASE_WIDTH_M / 2, 0),
    new Rotation2d(0.0)
  );
  // Drivetrain Performance Mechanical limits
  public static final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12.0);
  public static final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(8.0);
  public static final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(
    360.0
  );
  public static final double MAX_TRANSLATE_ACCEL_MPS2 =
    MAX_FWD_REV_SPEED_MPS / 0.25; //0-full time of 0.25 second
  public static final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 =
    MAX_ROTATE_SPEED_RAD_PER_SEC / 0.25; //0-full time of 0.25 second

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ROBOT ELECTRICAL CONSTANTS - update these to match how you wired your robot.
  // PWM Bank
  public static final int FL_WHEEL_MOTOR_IDX = 0;
  public static final int FL_AZMTH_MOTOR_IDX = 1;
  public static final int FR_WHEEL_MOTOR_IDX = 2;
  public static final int FR_AZMTH_MOTOR_IDX = 3;
  public static final int BL_WHEEL_MOTOR_IDX = 4;
  public static final int BL_AZMTH_MOTOR_IDX = 5;
  public static final int BR_WHEEL_MOTOR_IDX = 6;
  public static final int BR_AZMTH_MOTOR_IDX = 7;
  //static public final int UNUSED = 8;
  //static public final int UNUSED = 9;
  // DIO Bank
  public static final int FL_WHEEL_ENC_A_IDX = 0;
  public static final int FL_WHEEL_ENC_B_IDX = 1;
  public static final int FL_AZMTH_ENC_A_IDX = 2;
  public static final int FL_AZMTH_ENC_B_IDX = 3;
  public static final int FR_WHEEL_ENC_A_IDX = 4;
  public static final int FR_WHEEL_ENC_B_IDX = 5;
  public static final int FR_AZMTH_ENC_A_IDX = 6;
  public static final int FR_AZMTH_ENC_B_IDX = 7;
  public static final int BL_WHEEL_ENC_A_IDX = 8;
  public static final int BL_WHEEL_ENC_B_IDX = 9;
  public static final int BL_AZMTH_ENC_A_IDX = 10;
  public static final int BL_AZMTH_ENC_B_IDX = 11;
  public static final int BR_WHEEL_ENC_A_IDX = 12;
  public static final int BR_WHEEL_ENC_B_IDX = 13;
  public static final int BR_AZMTH_ENC_A_IDX = 14;
  public static final int BR_AZMTH_ENC_B_IDX = 15;
  //static public final int UNUSED = 16;
  //static public final int UNUSED = 17;
  //static public final int UNUSED = 18;
  //static public final int UNUSED = 19;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // SENSOR CONSTANTS
  // Sensor-related constants - pulled from datasheets for the sensors and gearboxes
  public static final int ENC_PULSE_PER_REV = 1024;
  public static final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV; //Assume 1-1 gearing for now
  public static final int AZMTH_ENC_COUNTS_PER_MODULE_REV = ENC_PULSE_PER_REV; //Assume 1-1 gearing for now
  public static final double WHEEL_ENC_WHEEL_REVS_PER_COUNT =
    1.0 / ((double) (Constants.WHEEL_ENC_COUNTS_PER_WHEEL_REV));
  public static final double AZMTH_ENC_MODULE_REVS_PER_COUNT =
    1.0 / ((double) (Constants.AZMTH_ENC_COUNTS_PER_MODULE_REV));
  // Vision Camera
  public static final String PHOTON_CAM_NAME = "MainCamera";

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // FIELD PHYSICAL CONSTANTS
  // Field wall locations which limit drivetrain motion
  public static final double FIELD_WIDTH_M = Units.feetToMeters(27.0);
  public static final double FIELD_LENGTH_M = Units.feetToMeters(54.0);
  public static final Translation2d MAX_ROBOT_TRANSLATION = new Translation2d(
    FIELD_LENGTH_M,
    FIELD_WIDTH_M
  );
  public static final Translation2d MIN_ROBOT_TRANSLATION = new Translation2d(
    0.0,
    0.0
  );
  // Assumed starting location of the robot. Auto routines will pick their own location and update this.
  public static final Pose2d DFLT_START_POSE = new Pose2d(
    Units.feetToMeters(24.0),
    Units.feetToMeters(10.0),
    Rotation2d.fromDegrees(180)
  );
  // Expected vision target locations on the field
  public static final Transform2d fieldToFarVisionTargetTrans = new Transform2d(
    new Translation2d(FIELD_LENGTH_M, Units.feetToMeters(9.8541)),
    Rotation2d.fromDegrees(0)
  );
  public static final Transform2d fieldToCloseVisionTargetTrans = new Transform2d(
    new Translation2d(Units.feetToMeters(0), Units.feetToMeters(17.14)),
    Rotation2d.fromDegrees(180)
  );

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////  Derived Constants
  //// - You can reference how these are calculated, but shouldn't
  ////   have to change them
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // HELPER ORGANIZATION CONSTANTS
  public static final int FL = 0; // Front Left Module Index
  public static final int FR = 1; // Front Right Module Index
  public static final int BL = 2; // Back Left Module Index
  public static final int BR = 3; // Back Right Module Index
  public static final int NUM_MODULES = 4;

  // Internal objects used to track where the modules are at relative to
  // the center of the robot, and all the implications that spacing has.
  private static double HW = WHEEL_BASE_WIDTH_M / 2.0;
  public static final List<Translation2d> robotToModuleTL = Arrays.asList(
    new Translation2d(HW, HW), //FL
    new Translation2d(HW, -HW), //FR
    new Translation2d(-HW, HW), //BL
    new Translation2d(-HW, -HW) //BR
  );

  public static final List<Transform2d> robotToModuleTF = Arrays.asList(
    new Transform2d(robotToModuleTL.get(FL), new Rotation2d(0.0)),
    new Transform2d(robotToModuleTL.get(FR), new Rotation2d(0.0)),
    new Transform2d(robotToModuleTL.get(BL), new Rotation2d(0.0)),
    new Transform2d(robotToModuleTL.get(BR), new Rotation2d(0.0))
  );

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    robotToModuleTL.get(FL),
    robotToModuleTL.get(FR),
    robotToModuleTL.get(BL),
    robotToModuleTL.get(BR)
  );

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MISC CONSTANTS
  // Nominal Periodic code execution rates
  public static final double SIM_SAMPLE_RATE_SEC = 0.001;
  public static final double CTRLS_SAMPLE_RATE_SEC = 0.02;
}