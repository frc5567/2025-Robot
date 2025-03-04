package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class that defines the GamePad used by the Copilot. This class extends the CommandGenericHID
 * class from the WPILib library.
 *
 * <p>Will need to adjust the ports and button names when we have final controls determined.
 */
public class CopilotGamePad extends CommandGenericHID {

  /**
   * Constructor, used for calling the super constructor (constructor used in the parent class
   * GenericHID).
   *
   * @param port the USB port of the Copilot's GamePad
   */
  public CopilotGamePad(final int port) {
    super(port);
  }

  /**
   * Enum that defines buttons on the GamePad and what they do when active
   *
   * <p>
   */
  public enum GamePadControls {
    Elevator_Start(1),
    Elevator_L1(2),
    Elevator_L2(3),
    Elevator_L3(4),
    Elevator_L4(5),
    Elevator_Intake(6),
    Left_Reef(7),
    Right_Reef(8),
    Launcher_IntakePosition(9),
    Launcher_ScorePosition(10),
    Launcher_Intake(11),
    Launcher_Score(12),
    Climber_Climb(13),
    Climber_Assist_Extend(14);

    public final int portNum;

    GamePadControls(int newPortNum) {
      this.portNum = newPortNum;
    }
  }

  /**
   * Checks if the Elevator Start Position button is pressed.
   *
   * @return true if the Elevator Start Position button was pressed, false if not.
   */
  public Trigger getElevatorStart() {
    Trigger theButton = button(GamePadControls.Elevator_Start.portNum);
    return theButton;
  }

  /**
   * Checks if the Elevator L1 Position button is pressed.
   *
   * @return true if the Elevator L1 Position button was pressed, false if not.
   */
  public Trigger getElevatorL1() {
    Trigger theButton = button(GamePadControls.Elevator_L1.portNum);
    return theButton;
  }

  /**
   * Checks if the Elevator L2 Position button is pressed.
   *
   * @return true if the Elevator L2 Position button was pressed, false if not.
   */
  public Trigger getElevatorL2() {
    Trigger theButton = button(GamePadControls.Elevator_L2.portNum);
    return theButton;
  }

  /**
   * Checks if the Elevator L3 Position button is pressed.
   *
   * @return true if the Elevator L3 Position button was pressed, false if not.
   */
  public Trigger getElevatorL3() {
    Trigger theButton = button(GamePadControls.Elevator_L3.portNum);
    return theButton;
  }

  /**
   * Checks if the Elevator L4 Position button is pressed.
   *
   * @return true if the Elevator L4 Position button was pressed, false if not.
   */
  public Trigger getElevatorL4() {
    Trigger theButton = button(GamePadControls.Elevator_L4.portNum);
    return theButton;
  }

  /**
   * Checks if the Elevator Intake button is pressed.
   *
   * @return true if the Elevator Intake button was pressed, false if not.
   */
  public Trigger getElevatorIntake() {
    Trigger theButton = button(GamePadControls.Elevator_Intake.portNum);
    return theButton;
  }

  /**
   * Checks if the Left Reef Position button is pressed.
   *
   * @return true if the Left Reef button was pressed, false if not.
   */
  public Trigger getLeftReef() {
    Trigger theButton = button(GamePadControls.Left_Reef.portNum);
    return theButton;
  }

  /**
   * Checks if the Right Reef button is pressed.
   *
   * @return true if the Right Reef button was pressed, false if not.
   */
  public Trigger getRightReef() {
    Trigger theButton = button(GamePadControls.Right_Reef.portNum);
    return theButton;
  }

  /**
   * Checks if the Launcher Intake Position button is pressed.
   *
   * @return true if the Launcher Intake Position button was pressed, false if not.
   */
  public Trigger getLauncherIntakePosition() {
    Trigger theButton = button(GamePadControls.Launcher_IntakePosition.portNum);
    return theButton;
  }

  /**
   * Checks if the Launcher Score Position button is pressed.
   *
   * @return true if the Launcher Score Position button was pressed, false if not.
   */
  public Trigger getLauncherScorePosiotion() {
    Trigger theButton = button(GamePadControls.Launcher_ScorePosition.portNum);
    return theButton;
  }

  /**
   * Checks if the Launcher Intake button is pressed.
   *
   * @return true if the Launcher Intake button was pressed, false if not.
   */
  public Trigger getLauncherIntake() {
    Trigger theButton = button(GamePadControls.Launcher_Intake.portNum);
    return theButton;
  }

  /**
   * Checks if the Launcher Score button is pressed.
   *
   * @return true if the Launcher Score button was pressed, false if not.
   */
  public Trigger getLauncherScore() {
    Trigger theButton = button(GamePadControls.Launcher_Score.portNum);
    return theButton;
  }

  /**
   * Checks if the Climber Climb button is pressed.
   *
   * @return true if the Climber Climb button was pressed, false if not.
   */
  public Trigger getClimberClimb() {
    Trigger theButton = button(GamePadControls.Climber_Climb.portNum);
    return theButton;
  }

  /**
   * Checks if the Climber Assist Extend button is pressed.
   *
   * @return true if the Climber Assist Extend button was pressed, false if not.
   */
  public Trigger getClimberAssistExtend() {
    Trigger theButton = button(GamePadControls.Climber_Assist_Extend.portNum);
    return theButton;
  }
}
