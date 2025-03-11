package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.generated.TunerConstants;

public class RobotMap {

  public static class ElevatorConstants {

    // Allows us to identify the elevator motor.
    public static final int ELEVATOR_MOTOR_CAN_ID = 29;

    /**
     * Sprocket has 2.638 inch diameter (so multiplied by pi) 8.2875 inches circumference. That
     * converts to 210.5mm per rotation of lower stage. Total travel of scoring mechanism is double
     * that at 411mm per rotation (since both stages move together and proportionally the same
     * distance) Gear Ratio is 50:1 411 / 50 = 8.22mm of travel per rotation of motor
     */
    public static final double MM_PER_ROTATION = 8.22;

    // TODO: Figure out the actual margin of error we want (in mm).
    public static final double ELEVATOR_MARGIN_OF_ERROR = 10.0;

    // TODO: Figure out the correct starting height in mm.
    public static final double STARTING_HEIGHT = 0.0;

    // TODO: Figure out the correct height in mm needed for intake.
    public static final double INTAKE_HEIGHT = 100.0;

    // TODO: Figure out correct height in mm needed for level 1.
    public static final double L1_SCORE_HEIGHT = 50.0;

    // TODO: Figure out correct height in mm needed for level 2.
    public static final double L2_SCORE_HEIGHT = 150.0;

    // TODO: Figure out correct height in mm needed for level 3.
    public static final double L3_SCORE_HEIGHT = 550.0;

    // TODO: Figure out correct height in mm needed for level 4 or if we are even going for level 4.
    public static final double L4_SCORE_HEIGHT = 1250.0;

    // Found offset using pheonix tuner subtracting, in mm.
    public static final double OFFSET = 0.438477;

    // TODO: Tune this to be reasonable for the drive team.
    public static final double MANUAL_ELEVATOR_POWER = 0.15;
  }

  public static final class LauncherConstants {
    // Allows us to identify the launcher motor.
    public static final int LAUNCHER_MOTOR_CAN_ID = 32;

    /**
     * Sets the motor speed in terms of percent power on the launcher motor to amintain contol of
     * coral.
     */
    // TODO: Figure out the actual Launch speed we want it to launch at.
    public static final double LAUNCHER_SPEED = 0.2;

    /**
     * Sets the motor speed in terms of percent power on the launcher motor to maintain contol of
     * coral.
     */
    // TODO: Figure out the actual Launch speed we want it to intake at.
    public static final double INTAKE_SPEED = -0.1;

    // TODO: Figure out the actual DIO port for Launcher sensor.
    public static final int LAUNCHER_SENSOR_DIO_PORT = 9;
  }

  public static final class AngleMotorConstants {
    // Allows us to identify the Angle motor.
    public static final int LAUNCH_ANGLE_MOTOR_CAN_ID = 31;

    // Limit switch to zero out the encoder on the LaunchAngle system
    public static final int LAUNCHER_LIMIT_SWITCH_DIO_PORT = 8;

    // TODO: find correct value of tolerance in degrees.
    public static final double ANGLE_TOLERANCE = 0.25;

    public static final double ANGLE_AT_LAUNCH = 14.0;

    public static final double ANGLE_AT_L4_LAUNCH = 15.5;

    public static final double ANGLE_AT_INTAKE = 7.5;

    // offset found by mesuring through pheonix tuner in rotations.
    public static final double OFFSET = -0.045410;

    // TODO: Tune this to be reasonable for the drive team.
    public static final double MANUAL_ANGLE_POWER = 0.10;
  }

  public static final class ClimberConstants {

    // Allows us to identify the climber motor.
    public static final int CLIMBER_MOTOR_CAN_ID = 30;

    // TODO: find correct margin of error rotations.
    public static final double CLIMBER_MARGIN_OF_ERROR = 0.5;

    /** 40:1 gear ratio and 2.25 rotations of the output shaft for complete travel. */
    public static final double CLIMBER_TRAVEL_DISTANCE = 300;

    // Offset found by measuring through pheonix tuner in rotations.
    public static final double OFFSET = -5.281250;
  }

  public static final class PilotControllerConstants {
    // Allows us to identify the usb port in the driver station.
    public static final int PILOT_CONTROLLER_USB_PORT = 0;
  }

  public static final class CopilotControllerConstants {
    // Allows us to identify the usb port in the driver station.
    public static final int COPILOT_CONTROLLER_USB_PORT = 1;
  }

  public static final class ClimberAssistConstants {
    // TODO: find corect CAN ID port.
    public static final int CLIMBER_ASSIST_MOTOR_CAN_ID = 33;

    // TODO: find the correct offset for the climber assist motor.
    public static final double OFFSET = 0;

    // TODO: find correct margin of error for the climber assist.
    public static final double CLIMBER_ASSIST_MARGIN_OF_ERROR = 5000;

    // TODO: find diameter of rack and pinion gear multiplyed by pi devided by 70 for gear ratio to
    // find travel per rotation in mm. Now uses 775Pro motor so encoder units are ticks - 4096 per.
    public static final double CLIMBER_ASSIST_TRAVEL_DISTANCE = 300000;
  }

  /**
   * Constants for the drive train.
   *
   * <p>These constants are used to configure the drive train of the robot, including the maximum
   * speed and angular rate.
   */
  public static final class DriveTrainConstants {

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // 3/4 of a rotation per second max angular velocity
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static final double DRIVE_SCALAR = 0.7;
  }

  /**
   * Constants for the field.
   *
   * <p>These constants are the field dimensions and elements. They are used for calculating the
   * robot's position on the field and for determining the location of the field elements.
   */
  public static final class FieldConstants {
    public static final double FIELD_WIDTH = 8.23;
    public static final double FIELD_LENGTH = 15.85;

    public static enum FIELD_ELEMENT {
      // 2025 - BLUE APRIL TAGS
      BLUE_CORAL_STATION_LEFT,
      BLUE_CORAL_STATION_RIGHT,
      BLUE_PROCESSOR,
      BLUE_BARGE_FRONT,
      BLUE_BARGE_BACK,
      BLUE_REEF_AB,
      BLUE_REEF_CD,
      BLUE_REEF_EF,
      BLUE_REEF_GH,
      BLUE_REEF_IJ,
      BLUE_REEF_KL,
      // 2025 - RED APRIL TAGS
      RED_CORAL_STATION_LEFT,
      RED_CORAL_STATION_RIGHT,
      RED_PROCESSOR,
      RED_BARGE_FRONT,
      RED_BARGE_BACK,
      RED_REEF_AB,
      RED_REEF_CD,
      RED_REEF_EF,
      RED_REEF_GH,
      RED_REEF_IJ,
      RED_REEF_KL,
      // 2025 - Blue Calculated Positions
      BLUE_CORAL_STATION_LEFT_ALLIANCE,
      BLUE_CORAL_STATION_LEFT_SIDEWALL,
      BLUE_CORAL_STATION_RIGHT_ALLIANCE,
      BLUE_CORAL_STATION_RIGHT_SIDEWALL,
      BLUE_LEFT_CAGE,
      BLUE_RIGHT_CAGE,
      BLUE_CENTER_CAGE,
      BLUE_REEF_CENTER,
      BLUE_REEF_A,
      BLUE_REEF_B,
      BLUE_REEF_C,
      BLUE_REEF_D,
      BLUE_REEF_E,
      BLUE_REEF_F,
      BLUE_REEF_G,
      BLUE_REEF_H,
      BLUE_REEF_I,
      BLUE_REEF_J,
      BLUE_REEF_K,
      BLUE_REEF_L,
      // 2025 - Red Calculated Positions
      RED_CORAL_STATION_LEFT_ALLIANCE,
      RED_CORAL_STATION_LEFT_SIDEWALL,
      RED_CORAL_STATION_RIGHT_ALLIANCE,
      RED_CORAL_STATION_RIGHT_SIDEWALL,
      RED_LEFT_CAGE,
      RED_RIGHT_CAGE,
      RED_CENTER_CAGE,
      RED_REEF_CENTER,
      RED_REEF_A,
      RED_REEF_B,
      RED_REEF_C,
      RED_REEF_D,
      RED_REEF_E,
      RED_REEF_F,
      RED_REEF_G,
      RED_REEF_H,
      RED_REEF_I,
      RED_REEF_J,
      RED_REEF_K,
      RED_REEF_L
    };

    public static enum REEF_OFFSETS {
      LEFT_BRANCH,
      RIGHT_BRANCH
    };

    public static enum TAG_IDS {
      // Blue
      BLUE_CORAL_STATION_LEFT_TAG(13),
      BLUE_CORAL_STATION_RIGHT_TAG(12),
      BLUE_PROCESSOR_TAG(16),
      BLUE_BARGE_FRONT_TAG(14),
      BLUE_BARGE_BACK_TAG(4),
      BLUE_REEF_AB_TAG(18),
      BLUE_REEF_CD_TAG(17),
      BLUE_REEF_EF_TAG(22),
      BLUE_REEF_GH_TAG(21),
      BLUE_REEF_IJ_TAG(20),
      BLUE_REEF_KL_TAG(19),
      // Red
      RED_CORAL_STATION_LEFT_TAG(1),
      RED_CORAL_STATION_RIGHT_TAG(2),
      RED_PROCESSOR_TAG(3),
      RED_BARGE_FRONT_TAG(5),
      RED_BARGE_BACK_TAG(15),
      RED_REEF_AB_TAG(7),
      RED_REEF_CD_TAG(8),
      RED_REEF_EF_TAG(9),
      RED_REEF_GH_TAG(10),
      RED_REEF_IJ_TAG(11),
      RED_REEF_KL_TAG(6);

      private final int value;

      TAG_IDS(int value) {
        this.value = value;
      }

      public int getValue() {
        return value;
      }
      ;
    }

    /**
     * Transforms for the field elements and the bot itself. These transforms are used when we're
     * trying to create a target pose for scoring a coral on a reef branch
     */
    public static final class FIELD_TRANSFORMS {

      /**
       * The Left coral branch is 6.5 inches left of the pose of the reef face tag. We need to make
       * sure we target that location. May need to tune slightly so 6.4 inches may be an
       * approximation.
       */
      public static final Transform3d LEFT_BRANCH =
          new Transform3d(
              Inches.of(0.0),
              Inches.of(-6.0),
              Inches.of(0.0),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0)));

      /**
       * The Right coral branch is 6.5 inches right of the pose of the reef face tag. We need to
       * make sure we target that location. May need to tune slightly so 6.4 inches may be an
       * approximation.
       */
      public static final Transform3d RIGHT_BRANCH =
          new Transform3d(
              Inches.of(0.0),
              Inches.of(6.0),
              Inches.of(0.0),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0)));

      /**
       * The transform for creating a target pose that takes into account that we want the front of
       * the bot facing the pose of the reef face tag. This means the center of the bot needs to be
       * 13 inches in front of the tag and 0 inches to the left or right. This is an approximation.
       *
       * <p>note that this also requires yaw rotation of 180 degrees.
       */
      public static final Transform3d ROBOT_TRANSFORM =
          new Transform3d(
              Inches.of(13.0),
              Inches.of(0.0),
              Inches.of(0.0),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(180.0)));
    }
  }
}
