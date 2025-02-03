package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

/**
 * @see frc.robot.subsystems.Elevator Class that sets the position of the elevator.
 * @return the one instance of the elevator.
 */
public class Elevator implements Subsystem {

  // Creates a member variable that represents the voltage needed to get to a position, and uses PID
  // to not overshoot target voltage. Start at position 0 (bottom).
  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  // Defines the variable "elevatorMotor" as a TalonFX motor.
  private TalonFX m_elevatorMotor;

  /**
   * Main constructor for the elevator class.
   *
   * @param motorPort the port of the motor controlling the elevator
   */
  public Elevator(int motorPort) {
    m_elevatorMotor = new TalonFX(motorPort);
  }

  /**
   * Method that get the motor position in terms of distance.
   *
   * @return the position of the elevator in mm.
   */
  public double getElevatorPosition() {

    // Gets current motor position in terms of rotations.
    StatusSignal<Angle> rotations = m_elevatorMotor.getPosition();
    double returnValue =
        rotations.getValue().magnitude() * RobotMap.ElevatorConstants.ROTATIONS_TO_DISTANCE;
    return returnValue;
  }

  /**
   * Set the elevator position in terms of mm.
   *
   * @param position position in mm we want to go to.
   */
  public void setElevatorPosition(double position) {

    // Convert desired position to distance in mm.
    double targetRotations = position / RobotMap.ElevatorConstants.ROTATIONS_TO_DISTANCE;
    m_elevatorMotor.setControl(m_positionVoltage.withPosition(targetRotations));
    double curPos = getElevatorPosition();
    System.out.println(
        "Attempting to move to position (" + position + ") Currently at (" + curPos + ")");
  }

  /**
   * @see frc.robot.subsystems.Elevator
   *     <p>Sets the speed to 0.
   * @return returns nothing
   */
  public void stopElevator() {
    m_elevatorMotor.set(0);
  }
}
