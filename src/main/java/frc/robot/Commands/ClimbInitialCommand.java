package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimberAssist;

/**
 * @see frc.robot.Commands.ClimbInitialCommand
 *     <p>Command that moves the climber assist to the correct position in terms of mm.
 * @return an instance of climber assist.
 */
public class ClimbInitialCommand extends Command {

  private final ClimberAssist m_climberAssistSubsystem;

  /**
   * The constructor of the ClimbInitialCommand class.
   *
   * @param climberAssistSubsystem the climber assist subsystem needed to construct the cilmber
   *     assist subsystem.
   * @param targetPosition the target position needed to construct the target position object.
   */
  public ClimbInitialCommand(ClimberAssist climberAssistSubsystem) {
    m_climberAssistSubsystem = climberAssistSubsystem;
    addRequirements(climberAssistSubsystem);
  }

  /**
   * @see frc.robot.Commands.ClimbInitialCommand
   *     <p>Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {}

  /**
   * @see frc.robot.Commands.ClimbInitialCommand
   *     <p>Calls the method to set the climber assist position from the climber assist class using
   *     the target postition.
   * @return returns nothing
   */
  @Override
  public void execute() {
    m_climberAssistSubsystem.setClimberAssistPosition(
        RobotMap.ClimberAssistConstants.CLIMBER_ASSIST_TRAVEL_DISTANCE);
  }

  /**
   * @see frc.robot.Commands.ClimbInitialCommand
   *     <p>Method which compares the current position of the climber assist to the desired
   *     position.
   * @return A boolean which is true if the current position is within 2 mm of the desired position.
   */
  @Override
  public boolean isFinished() {

    double currentPosition = m_climberAssistSubsystem.getClimberAssistPosition();
    // curpos and target position measured in mm
    return Math.abs(
            currentPosition - RobotMap.ClimberAssistConstants.CLIMBER_ASSIST_TRAVEL_DISTANCE)
        < RobotMap.ClimberAssistConstants.CLIMBER_ASSIST_MARGIN_OF_ERROR;
  }

  /**
   * @see frc.robot.Commands.ClimbInitialCommand
   *     <p>Ends the command when target position is reached.
   * @return returns nothing
   */
  @Override
  public void end(boolean interupted) {
    m_climberAssistSubsystem.stopClimberAssist();
  }
}
