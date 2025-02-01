package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchAngle;

public class SetLaunchAngleCommand extends Command {

    private final LaunchAngle m_launchAngle;
    private final double m_targetAnglePostion; //In degrees

    public SetLaunchAngleCommand(LaunchAngle angleOfLaunch, double targetPosition) {
        m_launchAngle = angleOfLaunch;
        m_targetAnglePostion = targetPosition;
        addRequirements(angleOfLaunch);
    }

    @Override
    public void initialize() {
    // You could add logic to smooth the movement if necessary.
    }

    @Override
    public void execute() {
        m_launchAngle.setPosition(m_targetAnglePostion);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = m_launchAngle.getPosition();
        return Math.abs
    }

}