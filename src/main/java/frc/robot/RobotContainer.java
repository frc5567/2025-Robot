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
import frc.robot.Commands.MoveElevatorCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // TODO: Find correct motor port for elevator.
  public final Elevator m_elevator = new Elevator(29);

  // TODO: Find correct port for pilot Xbox controller.
  private final CommandXboxController m_pilotController = new CommandXboxController(0);

  // TODO: Find correct port for copilot Xbox controller.
  private final CommandXboxController m_copilotController = new CommandXboxController(1);
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
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

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
        .onTrue(new MoveElevatorCommand(m_elevator, RobotMap.ElevatorConstants.L3_SCORE_HEIGHT));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
