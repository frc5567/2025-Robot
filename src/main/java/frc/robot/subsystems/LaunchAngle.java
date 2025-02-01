package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * @see frc.robot.subsystems.LaunchAngle Mechanism to control the angle of the launcher.
 * @return the instance of the launch angle
 */
public class LaunchAngle implements Subsystem {

  // Creates a member variable that represents the voltage needed to get to a position, and uses PID
  // to not overshoot target voltage. Start at position 0 (bottom).
  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private TalonFX m_angleMotor;

  /**
   * Constructor for the LaunchAngle mechanism.
   *
   * @param motorPort the CAN ID of the angle motor.
   */
  public LaunchAngle(int motorPort) {
    m_angleMotor = new TalonFX(motorPort);
  }

  /**
   * Gets the motor position in terms of rotations.
   *
   * @return the position of the motor in rotations with the type double.
   */
  public double getPosition() {
    // Gets the current motor positions in terms of rotations.
    StatusSignal<Angle> rotations = m_angleMotor.getPosition();
    return rotations.getValueAsDouble();
  }

  /**
   * Set desired position of the angle.
   *
   * @param position the position in rotations we want to go to.
   */
  public void setPosition(double position) {
    m_angleMotor.setControl(m_positionVoltage.withPosition(position));
  }

  /** Stops the angle motor of the launcher. */
  public void stopLauncher() {
    m_angleMotor.set(0);
  }
}
