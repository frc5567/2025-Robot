package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.LauncherConstants;
import frc.robot.subsystems.Launcher;

/**
 * @see frc.robot.Commands.IntakeCoralCommand Command that inatkes the choral and stops when it is
 *     captured.
 * @return an instance of the coral command.
 */
public class IntakeCoralCommand extends Command {

  // create a representation of the launcher
  private final Launcher m_launcherSubsystem;

  // make constructor method
  public IntakeCoralCommand(Launcher launcherSubsystem) {
    m_launcherSubsystem = launcherSubsystem;
  }

  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary.
  }

  /** Spin the launcher motor in reverse to intake the coral. */
  @Override
  public void execute() {
    m_launcherSubsystem.setLauncherSpeed(-LauncherConstants.LAUNCHER_SPEED);
  }

  // create a method to know if we're finished intaking
  @Override
  public boolean isFinished() {
    return m_launcherSubsystem.readSensor();
  }

  @Override
  public void end(boolean interrupted) {
    m_launcherSubsystem.stopLauncher();
  }
}
