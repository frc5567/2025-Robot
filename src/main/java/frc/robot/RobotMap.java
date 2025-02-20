package frc.robot;

public class RobotMap {

  public static class ElevatorConstants {

    // TODO: figure out the actual CAN ID for the Elevator Motor.
    public static final int ELEVATOR_MOTOR_CAN_ID = 16;

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
  }

  public static final class LauncherConstants {
    // TODO: Figure out the actual motor can ID for launcher.
    public static final int LAUNCHER_MOTOR_CAN_ID = 17;
    // TODO: Figure out the actual Launch speed we want it to launch at.
    public static final double LAUNCHER_SPEED = 20;
  }

  public static final class AngleMotorConstants {
    // TODO: find correct value of tolerance in degrees.
    public static final double ANGLE_TOLERANCE = 3.0;
  }

  public static final class ClimberConstants {
    // TODO: find correct number of rotations.
    public static final double MM_PER_ROTATION = 5.0;
    // TODO: find correct margin of error (in mm).
    public static final double CLIMBER_MARGIN_OF_ERROR = 2.0;
  }
}
