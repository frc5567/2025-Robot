package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

/**
 * @see frc.robot.subsystems.Elevator Class that sets the position of the elevator.
 * @return the one instance of the elevator.
 */
public class Elevator implements Subsystem {

  // Defines the variable "elevatorMotor" as a TalonFX motor.
  private TalonFX m_elevatorMotor;

  private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private final DigitalInput m_limitSwitch =
      new DigitalInput(RobotMap.ElevatorConstants.ELEVATOR_LIMIT_SWITCH_DIO);

  private boolean m_isLimitSwitchPressed = false;

  /**
   * Elevator gains for default slot 0. Note that these values were not tuned and are just
   * placeholders
   */
  private static final Slot0Configs elevatorGains =
      new Slot0Configs()
          .withKP(2.5)
          .withKI(0)
          .withKD(0)
          .withKS(0.001)
          .withKV(0.11)
          .withKA(0.003)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  /**
   * Main constructor for the elevator class.
   *
   * @param motorPort the port of the motor controlling the elevator
   */
  public Elevator(int motorPort) {
    m_elevatorMotor = new TalonFX(motorPort);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    var motionMagicConfigs = configs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    configs.withSlot0(elevatorGains);
    m_elevatorMotor.getConfigurator().apply(configs);

    // Zero out the encoder to start off. Assumes the elevator is at the bottom.
    m_elevatorMotor.setPosition(0.0);
  }

  /**
   * Method that get the motor position in terms of distance.
   *
   * @return the position of the elevator in mm.
   */
  public double getElevatorPosition() {

    // Gets current motor position in terms of rotations.
    StatusSignal<Angle> rotations = m_elevatorMotor.getPosition();
    Angle positionInRotations = rotations.getValue();

    Angle offset =
        Angle.ofRelativeUnits(
            RobotMap.ElevatorConstants.OFFSET, edu.wpi.first.units.Units.Rotations);

    positionInRotations = positionInRotations.plus(offset);

    double returnValue =
        positionInRotations.magnitude() * RobotMap.ElevatorConstants.MM_PER_ROTATION;

    return returnValue;
  }

  /**
   * Set the elevator position in terms of mm.
   *
   * @param position position in mm we want to go to.
   */
  public void setElevatorPosition(double position) {

    // Convert desired position to distance in mm.
    double targetRotations = position / RobotMap.ElevatorConstants.MM_PER_ROTATION;
    targetRotations = targetRotations - RobotMap.ElevatorConstants.OFFSET; // Offset the position
    m_elevatorMotor.setControl(m_motionMagicVoltage.withPosition(targetRotations));
    // m_elevatorMotor.setControl(m_positionVoltage.withPosition(targetRotations));
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

  /**
   * @see frc.robot.subsystems.Elevator
   *     <p>Sets the speed to the given power.
   * @param power percent power to send to elevator motor
   * @return returns nothing
   */
  public void moveElevator(double power) {
    DutyCycleOut mypower = new DutyCycleOut(0.0);
    if (power < 0) {
      if (m_limitSwitch.get()) {
        // don't manually overdrive down when already at the limit
        return;
      }
    }
    m_elevatorMotor.setControl(mypower.withOutput(power));
  }

  /**
   * Setting the break mode will make the motor ether brake or coast when not given power. Need to
   * expose this so we can set it to coast in disabledInit so the elevator can be manually
   * controlled when the bot is disabled.
   */
  public void setBrakeMode(NeutralModeValue mode) {
    m_elevatorMotor.setNeutralMode(mode);
  }

  public boolean isLimitSwitchNewlyPressed() {
    if (!m_isLimitSwitchPressed) {
      m_isLimitSwitchPressed = m_limitSwitch.get();
      return m_isLimitSwitchPressed;
    } else {
      m_isLimitSwitchPressed = m_limitSwitch.get();
    }
    // System.out.println("Limit Switch Value : [" + m_limitSwitch.get() + "]");
    return false;
  }

  public void zeroEncoder() {
    m_elevatorMotor.setPosition(0.0);
  }
}
