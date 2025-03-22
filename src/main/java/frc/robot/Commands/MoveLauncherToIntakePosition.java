package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LaunchAngle;

public class MoveLauncherToIntakePosition extends Command {

  private final LaunchAngle m_launchAngle;
  private final Elevator m_elevator;
  private final double m_height;
  private final double m_angle;

  public MoveLauncherToIntakePosition(LaunchAngle launchAngle, Elevator elevator, double height, double angle) {
    m_launchAngle = launchAngle;
    m_elevator = elevator;
    m_height = height;
    m_angle = angle;
    addRequirements(launchAngle, elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_launchAngle.setPosition(m_angle);
    m_elevator.setElevatorPosition(m_height);
  }

  @Override
  public boolean isFinished() {
    double currentElevatorPosition = m_elevator.getElevatorPosition();
    // Returns true if the absolute value of the current position minus the target position is less
    // than the elevator margin of error.
    boolean elevatorFinished =
        Math.abs(currentElevatorPosition - m_height)
            < RobotMap.ElevatorConstants.ELEVATOR_MARGIN_OF_ERROR;
    double currentLauncherAnglePosition = m_launchAngle.getPosition();
    boolean launchAngleFinished =
        Math.abs(currentLauncherAnglePosition - m_angle)
            < RobotMap.AngleMotorConstants.ANGLE_TOLERANCE;

    return (elevatorFinished && launchAngleFinished);
  }
}
