package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

  private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  // TODO: Must measure and tune these values, preferably under load!
  private static final Slot0Configs launcherAngleGains =
      new Slot0Configs()
          .withKP(2.5)
          .withKI(0)
          .withKD(0)
          .withKS(0.001)
          .withKV(0.11)
          .withKA(0.003)
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
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    var motionMagicConfigs = configs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
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
    m_angleMotor.setControl(m_motionMagicVoltage.withPosition(position));
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

  /**
   * Sets the angle motor to a certain power.
   *
   * @param power the power to set the motor to.
   */
  public void moveLaunchAngle(double power) {
    DutyCycleOut mypower = new DutyCycleOut(0.0);
    m_angleMotor.setControl(mypower.withOutput(power));
  }
}
