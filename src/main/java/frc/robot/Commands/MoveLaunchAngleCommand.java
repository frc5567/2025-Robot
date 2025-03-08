package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchAngle;

/**
 * @see frc.robot.MoveLaunchAngleCommand move the launcher angle
 * @return an instance of the launchAngle command.
 */
public class MoveLaunchAngleCommand extends Command {

  private final LaunchAngle m_launchAngle;
  private final double m_power; // In degrees

  /**
   * Constructor of the SetLaunchAngleCommand class.
   *
   * @param angleOfLaunch
   * @param power
   */
  public MoveLaunchAngleCommand(LaunchAngle angleOfLaunch, double power) {
    m_launchAngle = angleOfLaunch;
    m_power = power;
    addRequirements(angleOfLaunch);
  }

  /**
   * @see frc.robot.MoveLaunchAngleCommand Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {}

  /**
   * @see frc.robot.MoveLaunchAngleCommand Moves the launcher angle at specific percent power.
   * @return returns nothing
   */
  @Override
  public void execute() {
    m_launchAngle.moveLaunchAngle(m_power);
  }

  /**
   * @see frc.robot.MoveLaunchAngleCommand Checks if the command is finished.
   * @return returns false
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * @see frc.robot.MoveLaunchAngleCommand Stops the motor when the command is finished.
   * @return returns nothing
   */
  @Override
  public void end(boolean interrupted) {
    m_launchAngle.stopLauncher();
  }
}
