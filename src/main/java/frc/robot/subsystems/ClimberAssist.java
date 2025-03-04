package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

/**
 * @see frc.robot.subsystems.ClimberAssist
 * @return the instance of the ClimberAssist.
 */
public class ClimberAssist implements Subsystem {

  private TalonFX m_climberAssistMotor;

  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  /**
   * The constructor of the climber Assist class.
   *
   * @param motorPort the CAN ID of the climber Assist motor.
   */
  public ClimberAssist(int motorPort) {
    m_climberAssistMotor = new TalonFX(motorPort);
  }

  /**
   * Gets the motor position in terms of rotations.
   *
   * @return The position of the climber Assist in rotations.
   */
  public double getClimberAssistPosition() {
    StatusSignal<Angle> rotations = m_climberAssistMotor.getPosition();
    Angle positionInRotations = rotations.getValue();
    Angle offset =
        Angle.ofRelativeUnits(
            RobotMap.ClimberAssistConstants.OFFSET, edu.wpi.first.units.Units.Rotations);
    positionInRotations = positionInRotations.plus(offset);
    double returnValue = positionInRotations.magnitude();
    return returnValue;
  }

  /**
   * Sets the climber Assist position in terms of rotations.
   *
   * @param position The target position in terms of rotations.
   */
  public void setClimberAssistPosition(double position) {

    m_climberAssistMotor.setControl(m_positionVoltage.withPosition(position));
    // double curPos = getClimberAssistPosition();
  }

  /**
   * @see frc.robot.subsystems.ClimberAssist
   *     <p>Sets the speed to 0.
   * @return returns nothing.
   */
  public void stopClimberAssist() {
    m_climberAssistMotor.set(0);
  }

  /**
   * Setting the break mode will make the motor ether brake or coast when not given power. Need to
   * expose this so we can set it to coast in disabledInit so the elevator can be manually
   * controlled when the bot is disabled.
   */
  public void setBrakeMode(NeutralModeValue mode) {
    m_climberAssistMotor.setNeutralMode(mode);
  }
}
