package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * @see frc.robot.subsystems.Launcher
 *     <p>Class that sets launcher speed.
 * @return the instance of the launcher.
 */
public class Launcher implements Subsystem {

  private TalonFX m_launcherMotor;

  private DigitalInput m_coralSensor;

  private static final Slot0Configs launcherGains =
      new Slot0Configs()
          .withKP(2.5)
          .withKI(0)
          .withKD(0)
          .withKS(0.001)
          .withKV(0.11)
          .withKA(0.003)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  /**
   * The constructor method for the launcher class.
   *
   * @param motorPort CAN ID of the launcher motor.
   * @param sensorPort Digital Input port of the sensor.
   */
  public Launcher(int motorPort, int sensorPort) {
    m_launcherMotor = new TalonFX(motorPort);
    m_coralSensor = new DigitalInput(sensorPort);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    configs.withSlot0(launcherGains);
    m_launcherMotor.getConfigurator().apply(configs);
  }

  /**
   * Sets the launcher motor to the desired speed.
   *
   * @param percentPower percent power to the motor.
   */
  public void setLauncherSpeed(double percentPower) {
    DutyCycleOut power = new DutyCycleOut(percentPower);
    m_launcherMotor.setControl(power);
  }

  /** Stops the launcher motor when the desired speed is zero. */
  public void stopLauncher() {
    m_launcherMotor.set(0.0);
  }

  public boolean readSensor() {
    return !m_coralSensor.get();
  }

  /**
   * Setting the break mode will make the motor ether brake or coast when not given power. Need to
   * expose this so we can set it to coast in disabledInit so the elevator can be manually
   * controlled when the bot is disabled.
   */
  public void setBrakeMode(NeutralModeValue mode) {
    m_launcherMotor.setNeutralMode(mode);
  }
}
