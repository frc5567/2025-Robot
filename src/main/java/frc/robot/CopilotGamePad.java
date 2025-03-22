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
    Climber_Assist_Extend(1),
    Climber_Climb(2),
    FlushHeight(3),
    Elevator_L1(4),
    Elevator_L2(5),
    Elevator_L3(6),
    Elevator_L4(7),
    Elevator_Intake(8),
    Manual_Elevator_Down(9),
    Manual_Elevator_Up(10),
    Right_Reef(11),
    Launcher_Down(12);

    public final int portNum;

    GamePadControls(int newPortNum) {
      this.portNum = newPortNum;
    }
  }

  /**
   * Checks if the Manual Elevator Up button is pressed.
   *
   * @return true if the Manual Elevator Up button was pressed, false if not.
   */
  public Trigger getManualElevatorUp() {
    Trigger theButton = button(GamePadControls.Manual_Elevator_Up.portNum);
    System.out.println(
        "Manual Elevator Up Button: [" + GamePadControls.Manual_Elevator_Up.portNum + "]");

    return theButton;
  }

  /**
   * Checks if the Manual Elevator Down button is pressed.
   *
   * @return true if the Manual Elevator Down button was pressed, false if not.
   */
  public Trigger getManualElevatorDown() {
    Trigger theButton = button(GamePadControls.Manual_Elevator_Down.portNum);
    System.out.println(
        "Manual Elevator Down Button: [" + GamePadControls.Manual_Elevator_Down.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Elevator L1 Position button is pressed.
   *
   * @return true if the Elevator L1 Position button was pressed, false if not.
   */
  public Trigger getElevatorL1() {
    Trigger theButton = button(GamePadControls.Elevator_L1.portNum);
    System.out.println("Elevator L1 Button: [" + GamePadControls.Elevator_L1.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Elevator L2 Position button is pressed.
   *
   * @return true if the Elevator L2 Position button was pressed, false if not.
   */
  public Trigger getElevatorL2() {
    Trigger theButton = button(GamePadControls.Elevator_L2.portNum);
    System.out.println("Elevator L2 Button: [" + GamePadControls.Elevator_L2.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Elevator L3 Position button is pressed.
   *
   * @return true if the Elevator L3 Position button was pressed, false if not.
   */
  public Trigger getElevatorL3() {
    Trigger theButton = button(GamePadControls.Elevator_L3.portNum);
    System.out.println("Elevator L3 Button: [" + GamePadControls.Elevator_L3.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Elevator L4 Position button is pressed.
   *
   * @return true if the Elevator L4 Position button was pressed, false if not.
   */
  public Trigger getElevatorL4() {
    Trigger theButton = button(GamePadControls.Elevator_L4.portNum);
    System.out.println("Elevator L4 Button: [" + GamePadControls.Elevator_L4.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Elevator Intake button is pressed.
   *
   * @return true if the Elevator Intake button was pressed, false if not.
   */
  public Trigger getElevatorIntake() {
    Trigger theButton = button(GamePadControls.Elevator_Intake.portNum);
    System.out.println("Elevator Intake Button: [" + GamePadControls.Elevator_Intake.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Left Reef Position button is pressed.
   *
   * @return true if the Left Reef button was pressed, false if not.
   */
  public Trigger getFlushIntakeHeight() {
    Trigger theButton = button(GamePadControls.FlushHeight.portNum);
    System.out.println("Flush Height Button: [" + GamePadControls.FlushHeight.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Right Reef button is pressed.
   *
   * @return true if the Right Reef button was pressed, false if not.
   */
  public Trigger getRightReef() {
    Trigger theButton = button(GamePadControls.Right_Reef.portNum);
    System.out.println("Right Reef Button: [" + GamePadControls.Right_Reef.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Launcher Up button is pressed.
   *
   * @return true if the Launcher Up button was pressed, false if not.
   */
  public Trigger getManualLauncherUp() {
    Trigger theButton = axisGreaterThan(1, 0.5);
    System.out.println("Launcher Up Button: 1 axis less than 0");
    return theButton;
  }

  /**
   * Checks if the Launcher Down Position button is pressed.
   *
   * @return true if the Launcher Down Position button was pressed, false if not.
   */
  public Trigger getManualLauncherDown() {
    Trigger theButton = button(GamePadControls.Launcher_Down.portNum);
    System.out.println("Launcher Down Button: [" + GamePadControls.Launcher_Down.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Launcher Intake button is pressed.
   *
   * @return true if the Launcher Intake button was pressed, false if not.
   */
  public Trigger getLauncherIntake() {
    Trigger theButton = axisLessThan(0, -0.5);
    System.out.println("Launcher Intake Button: axis 0 less than 0");
    return theButton;
  }

  /**
   * Checks if the Launcher Score button is pressed.
   *
   * @return true if the Launcher Score button was pressed, false if not.
   */
  public Trigger getLauncherScore() {
    Trigger theButton = axisGreaterThan(0, 0.5);
    System.out.println("Launcher Score Button: axis 0 greater than 0");
    return theButton;
  }

  /**
   * Checks if the Climber Climb button is pressed.
   *
   * @return true if the Climber Climb button was pressed, false if not.
   */
  public Trigger getClimberClimb() {
    Trigger theButton = button(GamePadControls.Climber_Climb.portNum);
    System.out.println("Climber Climb Button: [" + GamePadControls.Climber_Climb.portNum + "]");
    return theButton;
  }

  /**
   * Checks if the Climber Assist Extend button is pressed.
   *
   * @return true if the Climber Assist Extend button was pressed, false if not.
   */
  public Trigger getClimberAssistExtend() {
    Trigger theButton = button(GamePadControls.Climber_Assist_Extend.portNum);
    System.out.println(
        "Climber Assist Extend Button: [" + GamePadControls.Climber_Assist_Extend.portNum + "]");
    return theButton;
  }
}
