package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LaunchAngle;

public class MoveLauncherToLaunchPosition extends Command {

  private final LaunchAngle m_launchAngle;
  private final Elevator m_elevator;
  private final double m_scoreHeight;

  public MoveLauncherToLaunchPosition(
      LaunchAngle launchAngle, Elevator elevator, double scoreHeight) {
    m_launchAngle = launchAngle;
    m_elevator = elevator;
    m_scoreHeight = scoreHeight;
    addRequirements(launchAngle, elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_launchAngle.setPosition(RobotMap.AngleMotorConstants.ANGLE_AT_LAUNCH);
    m_elevator.setElevatorPosition(m_scoreHeight);
  }

  @Override
  public boolean isFinished() {
    double currentElevatorPosition = m_elevator.getElevatorPosition();
    // Returns true if the absolute value of the current position minus the target position is less
    // than the elevator margin of error.
    boolean elevatorFinished =
        Math.abs(currentElevatorPosition - m_scoreHeight)
            < RobotMap.ElevatorConstants.ELEVATOR_MARGIN_OF_ERROR;
    double currentLauncherAnglePosition = m_launchAngle.getPosition();
    boolean launchAngleFinished =
        Math.abs(currentLauncherAnglePosition - RobotMap.AngleMotorConstants.ANGLE_AT_LAUNCH)
            < RobotMap.AngleMotorConstants.ANGLE_TOLERANCE;

    return (elevatorFinished && launchAngleFinished);
  }
}
