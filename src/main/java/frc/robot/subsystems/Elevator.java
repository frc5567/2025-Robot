package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Elevator implements Subsystem {

  // Lengh of travel in mm per rotation of the motor
  private double m_rotationsToDistance = 1;
  // Creates a member variable that represents the voltage needed to get to a position, and uses PID
  // to not overshoot target voltage. Start at position 0 (bottom).
  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  // Defines the variable "elevatorMotor" as a TalonFX motor.
  private TalonFX elevatorMotor;

  // TODO: Find the max rotations of the elevator motor.
  private static final double kChainMaxRotations = 0.0;

  /**
   * Main constructor for the elevator class.
   *
   * @param motorPort the port of the motor controlling the elevator
   */
  public Elevator(int motorPort) {
    // TODO: Find device id for elevator TalonFX.
    elevatorMotor = new TalonFX(motorPort);
  }

  /**
   * Method that get the motor position in terms of rotations.
   *
   * @return the position of the motor rotations with the type double.
   */
  public double getElevatorPosition() {

    // Gets current motor position in terms of rotations.
    StatusSignal<Angle> rotations = elevatorMotor.getPosition();
    return rotations.getValueAsDouble();
  }

  /**
   * Set the elevator position in terms of mm.
   *
   * @param position take current position
   */
  public void setElevatorPosition(double position) {

    // Convert desired position to distance in mm.
    double targetDistance = position * m_rotationsToDistance;
    elevatorMotor.setControl(m_positionVoltage);
    double curPos = getElevatorPosition();
    System.out.println(
        "Attempting to move to position (" + position + ") Currently at (" + curPos + ")");
  }
}
