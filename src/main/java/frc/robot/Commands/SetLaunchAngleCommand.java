package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.LaunchAngle;

public class SetLaunchAngleCommand extends Command {

  private final LaunchAngle m_launchAngle;
  private final double m_targetAnglePostion; // In degrees

  /**
   * Constructor of the SetLaunchAngleCommand class.
   *
   * @param angleOfLaunch
   * @param targetPosition
   */
  public SetLaunchAngleCommand(LaunchAngle angleOfLaunch, double targetPosition) {
    m_launchAngle = angleOfLaunch;
    m_targetAnglePostion = targetPosition;
    addRequirements(angleOfLaunch);
  }

  /**
   * @see frc.robot.Commands.SetLaunchAngleCommand Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary.
  }

  /**
   * @see frc.robot.Commands.SetLaunchAngleCommand Makes the motor turn to the right angle.
   * @return returns nothing
   */
  @Override
  public void execute() {
    m_launchAngle.setPosition(m_targetAnglePostion);
  }

  /**
   * @see frc.robot.Commands.SetLaunchAngleCommand Command is finished if we are within the angle
   *     tolerance.
   * @return returns true if it is within the angle tolerance.
   */
  @Override
  public boolean isFinished() {
    double currentPosition = m_launchAngle.getPosition();
    return Math.abs(currentPosition - m_targetAnglePostion)
        < RobotMap.AngleMotorConstants.ANGLE_TOLERANCE;
  }

  /**
   * @see frc.robot.Commands.SetLaunchAngleCommand Ends the command when angle is met.
   * @return returns nothing
   */
  @Override
  public void end(boolean interrupted) {
    m_launchAngle.stopLauncher();
  }
}
