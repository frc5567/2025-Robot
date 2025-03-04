package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimberAssist;

/**
 * @see frc.robot.Commands.MoveClimberAssistToPositionCommand
 *     <p>Command that moves the climber assist to the correct position.
 * @return an instance of climber assist.
 */
public class MoveClimberAssistToPositionCommand extends Command {

  private final ClimberAssist m_climberAssistSubsystem;

  private final double m_targetPosition;

  /**
   * The constructor of the MoveClimberAssistToPositionCommand class.
   *
   * @param climberAssistSubsystem the climber assist subsystem needed to construct the cilmber
   *     assist subsystem.
   * @param targetPosition the target position needed to construct the target position object.
   */
  public MoveClimberAssistToPositionCommand(
      ClimberAssist climberAssistSubsystem, double targetPosition) {
    m_climberAssistSubsystem = climberAssistSubsystem;
    m_targetPosition = targetPosition;
    addRequirements(climberAssistSubsystem);
  }

  /**
   * @see frc.robot.Commands.MoveClimberAssistToPositionCommand
   *     <p>Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {}

  /**
   * @see frc.robot.Commands.MoveClimberAssistToPositionCommand
   *     <p>Calls the method to set the climber assist position from the climber assist class using
   *     the target postition.
   * @return returns nothing
   */
  @Override
  public void execute() {
    m_climberAssistSubsystem.setClimberAssistPosition(m_targetPosition);
  }

  /**
   * @see frc.robot.Commands.MoveClimberAssistToPositionCommand
   *     <p>Method which compares the current position of the climber assist to the desired
   *     position.
   * @return A boolean which is true if the current position is within 2 mm of the desired position.
   */
  @Override
  public boolean isFinished() {

    double currentPosition = m_climberAssistSubsystem.getClimberAssistPosition();

    return Math.abs(currentPosition - m_targetPosition)
        < RobotMap.ClimberAssistConstants.CLIMBER_ASSIST_MARGIN_OF_ERROR;
  }

  /**
   * @see frc.robot.Commands.MoveClimberAssistToPositionCommand
   *     <p>Ends the command when target position is reached.
   * @return returns nothing
   */
  @Override
  public void end(boolean interupted) {
    m_climberAssistSubsystem.stopClimberAssist();
  }
}
