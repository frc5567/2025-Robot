// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.IntakeCoralCommand;
import frc.robot.Commands.LaunchCoralCommand;
import frc.robot.Commands.MoveClimberAssistToPositionCommand;
import frc.robot.Commands.MoveClimberToPositionCommand;
import frc.robot.Commands.MoveElevatorCommand;
import frc.robot.Commands.SetLaunchAngleCommand;
import frc.robot.generated.TunerConstants;
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

  private final CommandXboxController m_copilotController =
      new CommandXboxController(RobotMap.CopilotControllerConstants.COPILOT_CONTROLLER_USB_PORT);
  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void registerNamedCommands() {}

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -m_pilotController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_pilotController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_pilotController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    m_pilotController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    m_pilotController
        .b()
        .whileTrue(
            m_drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -m_pilotController.getLeftY(), -m_pilotController.getLeftX()))));

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

    m_drivetrain.registerTelemetry(logger::telemeterize);

    m_copilotController
        .leftBumper()
        .onTrue(new MoveElevatorCommand(m_elevator, RobotMap.ElevatorConstants.STARTING_HEIGHT));
    m_copilotController
        .y()
        .onTrue(new MoveElevatorCommand(m_elevator, RobotMap.ElevatorConstants.INTAKE_HEIGHT));
    m_copilotController
        .a()
        .onTrue(new MoveElevatorCommand(m_elevator, RobotMap.ElevatorConstants.L1_SCORE_HEIGHT));
    m_copilotController
        .b()
        .onTrue(new MoveElevatorCommand(m_elevator, RobotMap.ElevatorConstants.L2_SCORE_HEIGHT));
    m_copilotController
        .x()
        .onTrue(new MoveElevatorCommand(m_elevator, RobotMap.ElevatorConstants.L4_SCORE_HEIGHT));

    m_copilotController
        .rightBumper()
        .onTrue(
            new MoveClimberToPositionCommand(
                m_climber, RobotMap.ClimberConstants.CLIMBER_TRAVEL_DISTANCE));

    m_copilotController
        .povDown()
        .onTrue(
            new MoveClimberAssistToPositionCommand(
                m_climberAssist, RobotMap.ClimberAssistConstants.CLIMBER_ASSIST_TRAVEL_DISTANCE));

    m_copilotController
        .rightTrigger()
        .onTrue(
            new SetLaunchAngleCommand(m_launchAngle, RobotMap.AngleMotorConstants.ANGLE_AT_INTAKE));
    m_copilotController
        .leftTrigger()
        .onTrue(
            new SetLaunchAngleCommand(m_launchAngle, RobotMap.AngleMotorConstants.ANGLE_AT_LAUNCH));

    m_copilotController.povLeft().whileTrue(new LaunchCoralCommand(m_launcher));

    m_copilotController.povRight().onTrue(new IntakeCoralCommand(m_launcher));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
