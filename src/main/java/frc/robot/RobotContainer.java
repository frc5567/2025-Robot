// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbInitialCommand;
import frc.robot.commands.DriveToLeftBranch;
import frc.robot.commands.DriveToRightBranch;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.LaunchCoralCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.MoveLaunchAngleCommand;
import frc.robot.commands.MoveLauncherToIntakePosition;
import frc.robot.commands.MoveLauncherToLaunchL4Position;
import frc.robot.commands.MoveLauncherToLaunchPosition;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.helpers.TargetPoseHelper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberAssist;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LaunchAngle;
import frc.robot.subsystems.Launcher;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  public final Launcher m_launcher =
      new Launcher(
          RobotMap.LauncherConstants.LAUNCHER_MOTOR_CAN_ID,
          RobotMap.LauncherConstants.LAUNCHER_SENSOR_DIO_PORT);

  public final LaunchAngle m_launchAngle =
      new LaunchAngle(RobotMap.AngleMotorConstants.LAUNCH_ANGLE_MOTOR_CAN_ID);

  public final Climber m_climber = new Climber(RobotMap.ClimberConstants.CLIMBER_MOTOR_CAN_ID);

  public final ClimberAssist m_climberAssist =
      new ClimberAssist(RobotMap.ClimberAssistConstants.CLIMBER_ASSIST_MOTOR_CAN_ID);

  public final Elevator m_elevator = new Elevator(RobotMap.ElevatorConstants.ELEVATOR_MOTOR_CAN_ID);

  private final CommandXboxController m_pilotController =
      new CommandXboxController(RobotMap.PilotControllerConstants.PILOT_CONTROLLER_USB_PORT);

  private final CopilotGamePad m_copilotController =
      new CopilotGamePad(RobotMap.CopilotControllerConstants.COPILOT_CONTROLLER_USB_PORT);

  public final TargetPoseHelper m_poseHelper = new TargetPoseHelper();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  private Alliance m_allianceColor;

  public RobotContainer() {
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);
    m_allianceColor = Alliance.Red;
    configureBindings();
  }

  private void registerNamedCommands() {

    NamedCommands.registerCommand(
        "MoveLauncherIntake", new MoveLauncherToIntakePosition(m_launchAngle, m_elevator));

    NamedCommands.registerCommand(
        "MoveLauncherToL1",
        new MoveLauncherToLaunchPosition(
            m_launchAngle, m_elevator, RobotMap.ElevatorConstants.L1_SCORE_HEIGHT));

    NamedCommands.registerCommand(
        "MoveLauncherToL2",
        new MoveLauncherToLaunchPosition(
            m_launchAngle, m_elevator, RobotMap.ElevatorConstants.L2_SCORE_HEIGHT));

    NamedCommands.registerCommand(
        "MoveLauncherToL3",
        new MoveLauncherToLaunchPosition(
            m_launchAngle, m_elevator, RobotMap.ElevatorConstants.L3_SCORE_HEIGHT));

    NamedCommands.registerCommand(
        "MoveLauncherToL4", new MoveLauncherToLaunchL4Position(m_launchAngle, m_elevator));

    NamedCommands.registerCommand("DriveToRightBranch", new DriveToRightBranch(m_drivetrain));

    NamedCommands.registerCommand("ScoreCoral", new LaunchCoralCommand(m_launcher));
  }

  public void setAllianceColor(Alliance color) {
    if (color != m_allianceColor) {
      m_allianceColor = color;
    }
  }

  private void configureBindings() {

    // Set the default command for the drivetrain to be field-centric driving
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -m_pilotController.getLeftY()
                            * MaxSpeed
                            * RobotMap.DriveTrainConstants
                                .DRIVE_SCALAR) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_pilotController.getLeftX()
                            * MaxSpeed
                            * RobotMap.DriveTrainConstants
                                .DRIVE_SCALAR) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_pilotController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Set the other key bindings for the pilot controller
    m_pilotController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    m_pilotController
        .rightTrigger(0.5)
        .whileTrue(
            m_drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -(Math.copySign(
                                    (Math.pow(m_pilotController.getLeftY(), 2)),
                                    m_pilotController.getLeftY())
                                * MaxSpeed
                                / 6)) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -(Math.copySign(
                                    (Math.pow(m_pilotController.getLeftX(), 2)),
                                    m_pilotController.getLeftX())
                                * MaxSpeed
                                / 6)) // Drive left with negative X (left)
                        .withRotationalRate(
                            -m_pilotController.getRightX()
                                * MaxAngularRate
                                / 3) // Drive counterclockwise with negative X (left)
                ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_pilotController
        .back()
        .and(m_pilotController.y())
        .whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    m_pilotController
        .back()
        .and(m_pilotController.x())
        .whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    m_pilotController
        .start()
        .and(m_pilotController.y())
        .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    m_pilotController
        .start()
        .and(m_pilotController.x())
        .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    m_pilotController
        .leftBumper()
        .onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

    // Set the key bindings for the copilot controller
    m_copilotController
        .getClimberAssistExtend()
        .whileTrue(new ClimbInitialCommand(m_climberAssist));

    m_copilotController.getClimberClimb().whileTrue(new ClimbCommand(m_climber));

    m_copilotController.getLauncherIntake().whileTrue(new IntakeCoralCommand(m_launcher));
    m_copilotController.getLauncherScore().whileTrue(new LaunchCoralCommand(m_launcher));

    m_copilotController
        .getManualLauncherDown()
        .whileTrue(
            new MoveLaunchAngleCommand(
                m_launchAngle, RobotMap.AngleMotorConstants.MANUAL_ANGLE_POWER));
    m_copilotController
        .getManualLauncherUp()
        .whileTrue(
            new MoveLaunchAngleCommand(
                m_launchAngle, -RobotMap.AngleMotorConstants.MANUAL_ANGLE_POWER));

    m_copilotController
        .getElevatorIntake()
        .whileTrue(new MoveLauncherToIntakePosition(m_launchAngle, m_elevator));

    m_copilotController
        .getElevatorL1()
        .whileTrue(
            new MoveLauncherToLaunchPosition(
                m_launchAngle, m_elevator, RobotMap.ElevatorConstants.L1_SCORE_HEIGHT));
    m_copilotController
        .getElevatorL2()
        .whileTrue(
            new MoveLauncherToLaunchPosition(
                m_launchAngle, m_elevator, RobotMap.ElevatorConstants.L2_SCORE_HEIGHT));
    m_copilotController
        .getElevatorL3()
        .whileTrue(
            new MoveLauncherToLaunchPosition(
                m_launchAngle, m_elevator, RobotMap.ElevatorConstants.L3_SCORE_HEIGHT));
    m_copilotController
        .getElevatorL4()
        .whileTrue(
            new MoveLauncherToLaunchPosition(
                m_launchAngle, m_elevator, RobotMap.ElevatorConstants.L4_SCORE_HEIGHT));

    m_copilotController.getLeftReef().whileTrue(new DriveToLeftBranch(m_drivetrain));
    m_copilotController.getRightReef().whileTrue(new DriveToRightBranch(m_drivetrain));

    m_copilotController
        .getManualElevatorDown()
        .whileTrue(
            new MoveElevatorCommand(m_elevator, -RobotMap.ElevatorConstants.MANUAL_ELEVATOR_POWER));
    m_copilotController
        .getManualElevatorUp()
        .whileTrue(
            new MoveElevatorCommand(m_elevator, RobotMap.ElevatorConstants.MANUAL_ELEVATOR_POWER));

    m_drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
