package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

/**
 * @see frc.robot.subsystems.Climber
 * @return the instance of the Climber.
 */
public class Climber implements Subsystem {

  private TalonFX m_climberMotor;

  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  /**
   * The constructor of the climber class.
   *
   * @param motorPort the CAN ID of the climber motor.
   */
  public Climber(int motorPort) {
    m_climberMotor = new TalonFX(motorPort);
  }

  /**
   * Gets the motor position in terms of distance.
   *
   * @return the position of the climber in mm.
   */
  public double getClimberPosition() {
    StatusSignal<Angle> rotations = m_climberMotor.getPosition();
    double returnValue =
        rotations.getValue().magnitude() * RobotMap.ClimberConstants.MM_PER_ROTATION;
    return returnValue;
  }

  /**
   * Sets the climber position in terms of mm.
   *
   * @param position
   */
  public void setClimberPosition(double position) {

    // Convert desired position to distance in mm.
    double targetRotations = position / RobotMap.ClimberConstants.MM_PER_ROTATION;
    m_climberMotor.setControl(m_positionVoltage.withPosition(targetRotations));
    // double curPos = getClimberPosition();
  }

  /**
   * @see frc.robot.subsystems.Climber
   *     <p>Sets the speed to 0.
   * @return returns nothing.
   */
  public void stopClimber() {
    m_climberMotor.set(0);
  }
}
