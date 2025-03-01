package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
   * Gets the motor position in terms of rotations.
   *
   * @return The position of the climber in rotations.
   */
  public double getClimberPosition() {
    StatusSignal<Angle> rotations = m_climberMotor.getPosition();
    Angle positionInRotations = rotations.getValue();
    Angle offset =
        Angle.ofRelativeUnits(
            RobotMap.ClimberConstants.CLIMBER_OFFSET, edu.wpi.first.units.Units.Rotations);
    positionInRotations = positionInRotations.plus(offset);
    double returnValue = positionInRotations.magnitude();
    return returnValue;
  }

  /**
   * Sets the climber position in terms of rotations.
   *
   * @param position The target position in terms of rotations.
   */
  public void setClimberPosition(double position) {

    m_climberMotor.setControl(m_positionVoltage.withPosition(position));
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

  /**
   * Setting the break mode will make the motor ether brake or coast when not given power. Need to
   * expose this so we can set it to coast in disabledInit so the elevator can be manually
   * controlled when the bot is disabled.
   */
  public void setBrakeMode(NeutralModeValue mode) {
    m_climberMotor.setNeutralMode(mode);
  }
}
