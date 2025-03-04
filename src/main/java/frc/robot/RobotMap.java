package frc.robot;

public class RobotMap {

  public static class ElevatorConstants {

    // Allows us to identify the elevator motor.
    public static final int ELEVATOR_MOTOR_CAN_ID = 29;

    // TODO: Figure out the actual distance traveled (in mm) per one rotation of the elevator motor.
    public static final double MM_PER_ROTATION = 8.4;

    // TODO: Figure out the actual margin of error we want (in mm).
    public static final double ELEVATOR_MARGIN_OF_ERROR = 10.0;

    // TODO: Figure out the actual max number of rotations for the elevator motor.
    public static final double ELEVATOR_MOTOR_MAX_ROTATIONS = 5.0;

    // TODO: Figure out the correct starting height in mm.
    public static final double STARTING_HEIGHT = 0.0;

    // TODO: Figure out the correct height in mm needed for intake.
    public static final double INTAKE_HEIGHT = 500.0;

    // TODO: Figure out correct height in mm needed for level 1.
    public static final double L1_SCORE_HEIGHT = 300.0;

    // TODO: Figure out correct height in mm needed for level 2.
    public static final double L2_SCORE_HEIGHT = 600.0;

    // TODO: Figure out correct height in mm needed for level 3.
    public static final double L3_SCORE_HEIGHT = 900.0;

    // TODO: Figure out correct height in mm needed for level 4 or if we are even going for level 4.
    public static final double L4_SCORE_HEIGHT = 1200.0;
    // Found offset using pheonix tuner subtracting, in mm.
    public static final double OFFSET = 11.0;
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
    // TODO: find correct value of tolerance in degrees.
    public static final double ANGLE_TOLERANCE = 0.25;
    // TODO: find correct angle of movement in degrees.
    public static final double ANGLE_AT_LAUNCH = 14.0;
    // TODO: Adjust angle if needed.
    public static final double ANGLE_AT_INTAKE = 7.5;
    // offset found by mesuring through pheonix tuner in rotations.
    public static final double OFFSET = -0.051270;
  }

  public static final class ClimberConstants {
    // Allows us to identify the climber motor.
    public static final int CLIMBER_MOTOR_CAN_ID = 30;
    // TODO: find correct margin of error rotations.
    public static final double CLIMBER_MARGIN_OF_ERROR = 2.0;

    /** 40:1 gear ratio and 2.25 rotations of the output shaft for complete travel. */
    public static final double CLIMBER_TRAVEL_DISTANCE = 90.0;

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
    // TODO: find the correct offset for the climber assist motor.
    public static final double OFFSET = 2;
    // TODO: find corect CAN ID port.
    public static final int CLIMBER_ASSIST_MOTOR_CAN_ID = 33;
  }
}
