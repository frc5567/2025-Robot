package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;

/**
 * @see frc.robot.Commands.MoveClimberToPositionCommand
 *     <p>Command that moves the climber to the correct position in terms of mm.
 * @return an instance
 */
public class MoveClimberToPositionCommand extends Command {

  private final Climber m_climberSubsystem;

  private final double m_targetPosition;

  /**
   * The constructor of the MoveClimberToPositionCommand class
   *
   * @param climberSubsystem the climber subsystem needed to construct the cilmber subsystem.
   * @param targetPosition the target position needed to construct the target position object.
   */
  public MoveClimberToPositionCommand(Climber climberSubsystem, double targetPosition) {
    m_climberSubsystem = climberSubsystem;
    m_targetPosition = targetPosition;
    addRequirements(climberSubsystem);
  }

  /**
   * @see frc.robot.Commands.MoveClimberToPositionCommand
   *     <p>Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {}

  /**
   * @see frc.robot.Commands.MoveClimberToPositionCommand
   *     <p>Calls the method to set the climber position from the climber class using the target
   *     postition.
   * @return returns nothing
   */
  @Override
  public void execute() {
    m_climberSubsystem.setClimberPosition(m_targetPosition);
  }

  /**
   * @see frc.robot.Commands.MoveClimberToPositionCommand
   *     <p>Method which compares the current position of the climber to the desired position.
   * @return A boolean which is true if the current position is within 2 mm of the desired position.
   */
  @Override
  public boolean isFinished() {

    double currentPosition = m_climberSubsystem.getClimberPosition();
    // curpos and targetPosition are in terms of mm.
    return Math.abs(currentPosition - m_targetPosition)
        < RobotMap.ClimberConstants.CLIMBER_MARGIN_OF_ERROR;
  }

  /**
   * @see frc.robot.Commands.MoveClimberToPositionCommand
   *     <p>Ends the command when target position is reached.
   * @return returns nothing
   */
  @Override
  public void end(boolean interupted) {
    m_climberSubsystem.stopClimber();
  }
}
