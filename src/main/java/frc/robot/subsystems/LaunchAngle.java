package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

/**
 * @see frc.robot.subsystems.LaunchAngle Mechanism to control the angle of the launcher.
 * @return the instance of the launch angle
 */
public class LaunchAngle implements Subsystem {

  // Creates a member variable that represents the voltage needed to get to a position, and uses PID
  // to not overshoot target voltage. Start at position 0 (bottom).
  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  // TODO: Must measure and tune these values, preferably under load!
  private static final Slot0Configs launcherAngleGains =
      new Slot0Configs()
          .withKP(2.5)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private TalonFX m_angleMotor;

  /**
   * Constructor for the LaunchAngle mechanism.
   *
   * @param motorPort the CAN ID of the angle motor.
   */
  public LaunchAngle(int motorPort) {
    m_angleMotor = new TalonFX(motorPort);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(2)).withPeakReverseVoltage(Volts.of(-2));
    configs.withSlot0(launcherAngleGains);
    m_angleMotor.getConfigurator().apply(configs);
  }

  /**
   * Gets the motor position in terms of rotations.
   *
   * @return the position of the motor in rotations with the type double.
   */
  public double getPosition() {
    // Gets the current motor positions in terms of rotations.
    StatusSignal<Angle> rotations = m_angleMotor.getPosition();
    Angle positionInRotations = rotations.getValue();
    Angle offset =
        Angle.ofRelativeUnits(
            RobotMap.AngleMotorConstants.OFFSET, edu.wpi.first.units.Units.Rotations);
    positionInRotations = positionInRotations.plus(offset);
    return positionInRotations.magnitude();
  }

  /**
   * Set desired position of the angle.
   *
   * @param position the position in rotations we want to go to.
   */
  public void setPosition(double position) {
    position -= RobotMap.AngleMotorConstants.OFFSET;
    m_angleMotor.setControl(m_positionVoltage.withPosition(position));
  }

  /** Stops the angle motor of the launcher. */
  public void stopLauncher() {
    m_angleMotor.set(0);
  }

  /**
   * Setting the break mode will make the motor ether brake or coast when not given power. Need to
   * expose this so we can set it to coast in disabledInit so the elevator can be manually
   * controlled when the bot is disabled.
   */
  public void setBrakeMode(NeutralModeValue mode) {
    m_angleMotor.setNeutralMode(mode);
  }
}
