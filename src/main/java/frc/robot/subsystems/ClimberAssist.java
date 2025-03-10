package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * @see frc.robot.subsystems.ClimberAssist
 * @return the instance of the ClimberAssist.
 */
public class ClimberAssist implements Subsystem {

  private TalonSRX m_climberAssistMotor;

  /**
   * The constructor of the climber Assist class.
   *
   * @param motorPort the CAN ID of the climber Assist motor.
   */
  public ClimberAssist(int motorPort) {
    m_climberAssistMotor = new TalonSRX(motorPort);
    m_climberAssistMotor.configSelectedFeedbackSensor(
        com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    m_climberAssistMotor.setInverted(InvertType.InvertMotorOutput);
    m_climberAssistMotor.setSensorPhase(true);
    m_climberAssistMotor.setSelectedSensorPosition(0, 0, 10); // sets the encoder position to zero
  }

  /**
   * Gets the motor position in terms of rotations.
   *
   * @return The position of the climber Assist in rotations.
   */
  public double getClimberAssistPosition() {
    double pos = m_climberAssistMotor.getSelectedSensorPosition();

    return pos;
  }

  /**
   * @see frc.robot.subsystems.ClimberAssist
   *     <p>Sets the position of the climber Assist motor.
   * @param position The position to set the climber Assist motor to.
   * @return returns nothing.
   */
  public void setClimberAssistPosition(double position) {
    m_climberAssistMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, position);

    System.out.println(
        "ClimbAssist position [" + getClimberAssistPosition() + "][" + position + "]");
  }

  /**
   * @see frc.robot.subsystems.ClimberAssist
   *     <p>Sets the speed to 0.
   * @return returns nothing.
   */
  public void stopClimberAssist() {
    m_climberAssistMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.0);
  }

  /**
   * Setting the break mode will make the motor ether brake or coast when not given power. Need to
   * expose this so we can set it to coast in disabledInit so the elevator can be manually
   * controlled when the bot is disabled.
   */
  public void setBrakeMode(NeutralMode mode) {
    m_climberAssistMotor.setNeutralMode(mode);
  }
}
