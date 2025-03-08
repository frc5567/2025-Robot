package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.helpers.TargetPoseHelper;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToRightBranch extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private Pose2d m_targetPose = null;
  private Command m_pathCommand = null;

  /**
   * Primary constructor for the DriveToLeftBranch command. Requires the drivetrain object be passed
   * in since there should only be one instance of that.
   *
   * @param elevator The one instance of the elevator object
   */
  public DriveToRightBranch(CommandSwerveDrivetrain drivetrain) {
    this.m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  /**
   * The initiaization routine. Called when the Command is first created. This mehod is kind of
   * redundant since the constructor is doing much the same.
   */
  @Override
  public void initialize() {
    // get our current pose
    Pose2d curPose = m_drivetrain.getPose();

    // Find nearest reef tag
    RobotMap.FieldConstants.TAG_IDS reefTag = TargetPoseHelper.GetNearestReefTagID(curPose);

    // get target pose for Left Branch of nearest tag
    m_targetPose =
        TargetPoseHelper.calculateTargetPose(
            reefTag, RobotMap.FieldConstants.REEF_OFFSETS.RIGHT_BRANCH);

    // resultant Command to schedule
    m_pathCommand = m_drivetrain.driveToPose(m_targetPose);

    // Schedule the generated command
    if (m_pathCommand != null) {
      m_pathCommand.schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return m_pathCommand == null || m_pathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (m_pathCommand != null) {
      m_pathCommand.cancel(); // Cancel the path command if the parent is interrupted
    }
  }
}
