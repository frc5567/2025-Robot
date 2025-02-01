package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

/**
 * @see frc.robot.Commands.MoveElevatorCommand Command that moves elevator up and down to desired
 *     posiotions.
 * @return the instance of the elevator command.
 */
public class MoveElevatorCommand extends Command {

  // Private representation of the Elevator class within this class.
  private final Elevator m_elevatorSubsystem;

  // Target position in terms of mm.
  private final double m_targetPosition;

  /**
   * Constructor which constructs an object representing the Elevator and the target position.
   *
   * @param elevatorSubsystem The elevator subsystem needed to construct the elevator subsystem
   *     object.
   * @param targetPosition The target position needed to construct the target position object.
   */
  public MoveElevatorCommand(Elevator elevatorSubsytem, double targetPosition) {
    m_elevatorSubsystem = elevatorSubsytem;
    m_targetPosition = targetPosition;
    addRequirements(elevatorSubsytem);
  }

  /**
   * @see frc.robot.Commands.MoveElevatorCommand Code that happens before it is executed.
   * @return returns nothing
   */
  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary.
  }

  /**
   * @see frc.robot.Commands.MoveElevatorCommand Calls the method to set the elevator position from
   *     the elevator class using the target postition.
   */
  @Override
  public void execute() {
    m_elevatorSubsystem.setElevatorPosition(m_targetPosition);
  }

  /**
   * @see frc.robot.Commands.MoveElevatorCommand Method which compares the current position of the
   *     elevator to the desired position.
   * @return A boolean which is true if the current position is within 10 mm of the desired
   *     position.
   */
  @Override
  public boolean isFinished() {

    // Calls the getElevatorPostition method from the elevator class to use to figure out whether we
    // have reached the target position.
    double currentPosition = m_elevatorSubsystem.getElevatorPosition();
    // Returns true if the absolute value of the current position minus the target position is less
    // than the elevator margin of error.
    return Math.abs(currentPosition - m_targetPosition)
        < RobotMap.ElevatorConstants.ELEVATOR_MARGIN_OF_ERROR;
  }

  /**
   * @see frc.robot.Commands.MoveElevatorCommand Ends the command when target position is reached.
   * @return returns nothing
   */
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stopElevator();
  }
}
