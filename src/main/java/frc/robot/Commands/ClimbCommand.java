package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;

/**
 * @see frc.robot.Commands.ClimbCommand
 *     <p>Command that moves the climber to the correct position in terms of mm.
 * @return an instance
 */
public class ClimbCommand extends Command {

  private final Climber m_climberSubsystem;

  /**
   * The constructor of the ClimbCommand class
   *
   * @param climberSubsystem the climber subsystem needed to construct the cilmber subsystem.
   */
  public ClimbCommand(Climber climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  /**
   * @see frc.robot.Commands.ClimbCommand
   *     <p>Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {}

  /**
   * @see frc.robot.Commands.ClimbCommand
   *     <p>Calls the method to set the climber position from the climber class using the target
   *     postition.
   * @return returns nothing
   */
  @Override
  public void execute() {
    m_climberSubsystem.setClimberPower(RobotMap.ClimberConstants.CLIMBER_POWER);
  }

  /**
   * @see frc.robot.Commands.ClimbCommand
   *     <p>Method which compares the current position of the climber to the desired position.
   * @return A boolean which is true if the current position is within 2 mm of the desired position.
   */
  @Override
  public boolean isFinished() {

    double currentPosition = m_climberSubsystem.getClimberPosition();
    // curpos and targetPosition are in terms of mm.
    return Math.abs(currentPosition - RobotMap.ClimberConstants.CLIMBER_TRAVEL_DISTANCE)
        < RobotMap.ClimberConstants.CLIMBER_MARGIN_OF_ERROR;
  }

  /**
   * @see frc.robot.Commands.ClimbCommand
   *     <p>Ends the command when target position is reached.
   * @return returns nothing
   */
  @Override
  public void end(boolean interupted) {
    m_climberSubsystem.stopClimber();
  }
}
