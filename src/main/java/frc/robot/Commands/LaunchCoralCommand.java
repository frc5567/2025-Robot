package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.LauncherConstants;
import frc.robot.subsystems.Launcher;

/**
 * @see frc.robot.Commands.LaunchCoralCommand Command that launches the coral and stops when there
 *     is nothing captured.
 * @return an instance of the coral command.
 */
public class LaunchCoralCommand extends Command {

  // create a representation of the launcher

  private final Launcher m_launcherSubsystem;

  /**
   * Constructor of the LaunchCoralCommand class
   *
   * @param launcherSubsystem
   */
  public LaunchCoralCommand(Launcher launcherSubsystem) {
    m_launcherSubsystem = launcherSubsystem;
  }

  /**
   * @see frc.robot.Commands.LaunchCoralCommand Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary.
  }

  /**
   * @see frc.robot.Commands.LaunchCoralCommand Spin the launcher motor to launch the coral.
   * @return returns nothing
   */
  @Override
  public void execute() {
    m_launcherSubsystem.setLauncherSpeed(LauncherConstants.LAUNCHER_SPEED);
  }

  /**
   * @see frc.robot.Commands.LaunchCoralCommand Command is finished if there is no coral in
   *     launcher.
   * @return returns true if the coral is not sensed.
   */
  @Override
  public boolean isFinished() {
    return !m_launcherSubsystem.readSensor();
  }

  /**
   * @see frc.robot.Commands.LaunchCoralCommand Ends the command when recieves that coral is not
   *     sensed.
   * @return returns nothing
   */
  @Override
  public void end(boolean interrupted) {
    m_launcherSubsystem.stopLauncher();
  }
}
