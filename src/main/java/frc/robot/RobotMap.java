package frc.robot;

public class RobotMap {

  public static class ElevatorConstants {

    // TODO: figure out the actual CAN ID for the Elevator Motor.
    public static final int ELEVATOR_MOTOR_CAN_ID = 29;

    // TODO: Figure out the actual distance traveled (in mm) per one rotation of the elevator motor.
    public static final double MM_PER_ROTATION = 150.0;

    // TODO: Figure out the actual margin of error we want (in mm).
    public static final double ELEVATOR_MARGIN_OF_ERROR = 10.0;

    // TODO: Figure out the actual max number of rotations for the elevator motor.
    public static final double ELEVATOR_MOTOR_MAX_ROTATIONS = 5.0;

    // TODO: Figure out the correct starting height in mm.
    public static final double STARTING_HEIGHT = 0.0;

    // TODO: Figure out the correct height in mm needed for intake.
    public static final double INTAKE_HEIGHT = 10.0;

    // TODO: Figure out correct height in mm needed for level 1.
    public static final double L1_SCORE_HEIGHT = 20.0;

    // TODO: Figure out correct height in mm needed for level 2.
    public static final double L2_SCORE_HEIGHT = 30.0;

    // TODO: Figure out correct height in mm needed for level 3.
    public static final double L3_SCORE_HEIGHT = 40.0;

    // TODO: Figure out correct height in mm needed for level 4 or if we are even going for level 4.
    public static final double L4_SCORE_HEIGHT = 50.0;

    public static final double POSITION_OFFSET = -0.011230;
  }

  public static final class LauncherConstants {
    // TODO: Figure out the actual motor CAN ID for launcher.
    public static final int LAUNCHER_MOTOR_CAN_ID = 32;
    // TODO: Figure out the actual Launch speed we want it to launch at.
    public static final double LAUNCHER_SPEED = 20;
    // TODO: Figure out the actual DIO port for Launcher sensor.
    public static final int LAUNCHER_SENSOR_DIO_PORT = 0;
  }

  public static final class AngleMotorConstants {
    // TODO: find correct value of tolerance in degrees.
    public static final double ANGLE_TOLERANCE = 3.0;
    // TODO: find correct angle of movement in degrees.
    public static final double ANGLE_AT_LAUNCH = 120.0;
    // TODO: Adjust angle if needed.
    public static final double ANGLE_AT_INTAKE = 60.0;
    // TODO: Figure out the actual CAN ID for the launcher angle.
    public static final int LAUNCH_ANGLE_MOTOR_CAN_ID = 31;

    public static final double OFFSET = -0.051270;
  }

  public static final class ClimberConstants {
    // TODO: find correct number of rotations.
    public static final double MM_PER_ROTATION = 5.0;
    // TODO: find correct margin of error (in mm).
    public static final double CLIMBER_MARGIN_OF_ERROR = 2.0;
    // TODO: find correct travel distance in rotations (in mm).
    public static final double CLIMBER_TRAVEL_DISTANCE = 100.0;
    // TODO: find the correct CAN ID for the Climber motor.
    public static final int CLIMBER_MOTOR_CAN_ID = 30;

    public static final double CLIMBER_OFFSET = -5.281250;
  }

  public static final class PilotControlConstants {
    // TODO find the corect usb port for the pilot controler.
    public static final int PILOT_CONTROLER_USB_PORT = 0;
  }

  public static final class CopilotControlerConstants {
    // TODO find the corect usb port for the copilot controler.
    public static final int COPILOT_CONTROLER_USB_PORT = 1;
  }
}
