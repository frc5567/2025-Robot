package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LaunchAngle;

public class MoveLauncherToIntakePosition extends Command {

  private final LaunchAngle m_launchAngle;
  private final Elevator m_elevator;

  public MoveLauncherToIntakePosition(LaunchAngle launchAngle, Elevator elevator) {
    m_launchAngle = launchAngle;
    m_elevator = elevator;
    addRequirements(launchAngle, elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_launchAngle.setPosition(RobotMap.AngleMotorConstants.ANGLE_AT_INTAKE);
    m_elevator.setElevatorPosition(RobotMap.ElevatorConstants.INTAKE_HEIGHT);
  }

  @Override
  public boolean isFinished() {
    double currentElevatorPosition = m_elevator.getElevatorPosition();
    // Returns true if the absolute value of the current position minus the target position is less
    // than the elevator margin of error.
    boolean elevatorFinished =
        Math.abs(currentElevatorPosition - RobotMap.ElevatorConstants.INTAKE_HEIGHT)
            < RobotMap.ElevatorConstants.ELEVATOR_MARGIN_OF_ERROR;
    double currentLauncherAnglePosition = m_launchAngle.getPosition();
    boolean launchAngleFinished =
        Math.abs(currentLauncherAnglePosition - RobotMap.AngleMotorConstants.ANGLE_AT_INTAKE)
            < RobotMap.AngleMotorConstants.ANGLE_TOLERANCE;

    return (elevatorFinished && launchAngleFinished);
  }
}
