package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Launcher implements Subsystem {

  private TalonFX m_launcherMotor;

  private DigitalInput m_coralSensor;

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  /**
   * The constructor method for the launcher class.
   *
   * @param motorPort CAN ID of the launcher motor.
   * @param sensorPort Digital Input port of the sensor.
   */
  public Launcher(int motorPort, int sensorPort) {
    m_launcherMotor = new TalonFX(motorPort);
    m_coralSensor = new DigitalInput(sensorPort);
  }

  /**
   * Sets the launcher motor to the desired speed.
   *
   * @param velocity velocity in rotations per seconds.
   */
  public void setLauncherSpeed(double velocity) {
    m_launcherMotor.setControl(m_velocityVoltage.withVelocity(velocity));
  }

  /** Stops the launcher motor when the desired speed is zero. */
  public void stopLauncher() {
    m_launcherMotor.set(0.0);
  }

  public boolean readSensor() {
    return m_coralSensor.get();
  }
}
