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

  /**
   * Constructor of the intakeCoralCommand class.
   *
   * @param launcherSubsystem
   */
  public IntakeCoralCommand(Launcher launcherSubsystem) {
    m_launcherSubsystem = launcherSubsystem;
    // System.out.println("Coral Contaned");
  }

  /**
   * @see frc.robot.Commands.IntakeCoralCommand Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary.
  }

  /**
   * @see frc.robot.Commands.IntakeCoralCommand Spin the launcher motor in reverse to intake the
   *     coral.
   * @return returns nothing
   */
  @Override
  public void execute() {
    m_launcherSubsystem.setLauncherSpeed(LauncherConstants.INTAKE_SPEED);
  }

  /**
   * @see frc.robot.Commands.IntakeCoralCommand Command is finished if coral is sensed.
   * @return true if coral is loaded.
   */
  @Override
  public boolean isFinished() {
    return m_launcherSubsystem.readSensor();
  }

  /**
   * @see frc.robot.Commands.IntakeCoralCommand Stops the command.
   * @return returns nothing
   */
  @Override
  public void end(boolean interrupted) {
    m_launcherSubsystem.stopLauncher();
  }
}
