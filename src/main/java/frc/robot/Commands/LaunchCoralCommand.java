package frc.robot.commands;

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
  private int m_launchCounter;

  /**
   * Constructor of the LaunchCoralCommand class
   *
   * @param launcherSubsystem
   */
  public LaunchCoralCommand(Launcher launcherSubsystem) {
    m_launcherSubsystem = launcherSubsystem;
    m_launchCounter = 0;
    addRequirements(m_launcherSubsystem);
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
    m_launchCounter++;
  }

  /**
   * @see frc.robot.Commands.LaunchCoralCommand Command is finished if there is no coral in
   *     launcher.
   * @return returns true if the coral is not sensed.
   */
  @Override
  public boolean isFinished() {
    if (m_launchCounter > 50) {
      m_launchCounter = 0;
      return true;
    }
    return false;
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
