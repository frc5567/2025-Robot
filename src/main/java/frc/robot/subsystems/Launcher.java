package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

public class Launcher implements Subsystem {

  private TalonFX m_launcherMotor;

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  /**
   * The constructor method for the launcher class.
   *
   * @param motorPort CAN ID of the launcher motor.
   */
  public Launcher(int motorPort) {
    m_launcherMotor = new TalonFX(RobotMap.LauncherConstants.LAUNCHER_MOTOR_CAN_ID);
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
}
