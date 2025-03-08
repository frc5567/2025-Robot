package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * @see frc.robot.Commands.MoveElevatorCommand Command that moves elevator up and down to desired
 *     positions.
 * @return an instance of the elevator command.
 */
public class MoveElevatorCommand extends Command {

  // Private representation of the Elevator class within this class.
  private final Elevator m_elevatorSubsystem;

  // Target position in terms of mm.
  private final double m_dutyCycle;

  /**
   * Constructor which constructs an object representing the Elevator and the target position.
   *
   * @param elevatorSubsystem The elevator subsystem needed to construct the elevator subsystem
   *     object.
   * @param power percent power to send to elevator motor
   */
  public MoveElevatorCommand(Elevator elevatorSubsytem, double power) {
    m_elevatorSubsystem = elevatorSubsytem;
    m_dutyCycle = power;
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
    m_elevatorSubsystem.moveElevator(m_dutyCycle);
  }

  /**
   * @see frc.robot.Commands.MoveElevatorCommand Method which compares the current position of the
   *     elevator to the desired position.
   * @return false -- the command never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
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
