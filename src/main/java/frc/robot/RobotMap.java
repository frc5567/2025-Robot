package frc.robot;

public class RobotMap {

  public static class ElevatorConstants {

    // TODO: figure out the actual CAN ID for the Elevator Motor.
    public static final int ELEVATOR_MOTOR_CAN_ID = 16;

    // TODO: Figure out the actual distance traveled (in mm) per one rotation of the elevator motor.
    public static final double ROTATIONS_TO_DISTANCE = 150.0;

    // TODO: Figure out the actual margin of error we want (in mm).
    public static final double ELEVATOR_MARGIN_OF_ERROR = 10.0;

    // TODO: Figure out the actual max number of rotations for the elevator motor.
    public static final double ELEVATOR_MOTOR_MAX_ROTATIONS = 5.0;
  }

  public static final class LauncherConstants {
    // TODO: Figure out the actual motor can ID for launcher.
    public static final int LAUNCHER_MOTOR_CAN_ID = 17;
    // TODO: Figure out the actual Launch speed we want it to launch at.
    public static final double LAUNCHER_SPEED = 20;
  }
}
