package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CopilotGamePad extends CommandGenericHID {

  public CopilotGamePad(final int port) {
    super(port);
  }

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

  public Trigger getElevatorStart() {
    Trigger theButton = button(GamePadControls.Elevator_Start.portNum);
    return theButton;
  }

  public Trigger getElevatorL1() {
    Trigger theButton = button(GamePadControls.Elevator_L1.portNum);
    return theButton;
  }

  public Trigger getElevatorL2() {
    Trigger theButton = button(GamePadControls.Elevator_L2.portNum);
    return theButton;
  }

  public Trigger getElevatorL3() {
    Trigger theButton = button(GamePadControls.Elevator_L3.portNum);
    return theButton;
  }

  public Trigger getElevatorL4() {
    Trigger theButton = button(GamePadControls.Elevator_L4.portNum);
    return theButton;
  }

  public Trigger getElevatorIntake() {
    Trigger theButton = button(GamePadControls.Elevator_Intake.portNum);
    return theButton;
  }

  public Trigger getLeftReef() {
    Trigger theButton = button(GamePadControls.Left_Reef.portNum);
    return theButton;
  }

  public Trigger getRightReef() {
    Trigger theButton = button(GamePadControls.Right_Reef.portNum);
    return theButton;
  }

  public Trigger getLauncherIntakePosition() {
    Trigger theButton = button(GamePadControls.Launcher_IntakePosition.portNum);
    return theButton;
  }

  public Trigger getLauncherScorePosiotion() {
    Trigger theButton = button(GamePadControls.Launcher_ScorePosition.portNum);
    return theButton;
  }

  public Trigger getLauncherIntake() {
    Trigger theButton = button(GamePadControls.Launcher_Intake.portNum);
    return theButton;
  }

  public Trigger getLauncherScore() {
    Trigger theButton = button(GamePadControls.Launcher_Score.portNum);
    return theButton;
  }

  public Trigger getCliberClimb() {
    Trigger theButton = button(GamePadControls.Climber_Climb.portNum);
    return theButton;
  }

  public Trigger getClimberAssistExtend() {
    Trigger theButton = button(GamePadControls.Climber_Assist_Extend.portNum);
    return theButton;
  }
}
