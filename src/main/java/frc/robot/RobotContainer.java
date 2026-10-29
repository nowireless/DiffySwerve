// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hack;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Turret;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final Battery m_battery = new Battery();
  private final Turret m_turret = new Turret();
  private final Power m_power = new Power(
    // m_turret::getBusVoltage

    // Just hard code 20volts
    () -> 20
  );
  private final Drive m_drive = new Drive(m_power);
  // private final Hack m_hack = new Hack();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    final var xFilter = new SlewRateLimiter(5);
    final var yFilter = new SlewRateLimiter(5);
    final var rotateFilter = new SlewRateLimiter(5);

    DoubleSupplier slowModeSupplier = () -> {

      // if (elevator1.getPosition() > 50) {
      //   return 0.25;
      // }

      // if (elevator1.getPosition() > 10) {
      //   return 0.5;
      // }

      // No slow mode
      return 1.0;
    };

    DoubleSupplier rotationSupplier = () -> {
      double leftTrigger = m_driverController.getHID().getLeftTriggerAxis();
      double rightTrigger = m_driverController.getHID().getRightTriggerAxis();

      SmartDashboard.putNumber("Left Trigger", leftTrigger);
      SmartDashboard.putNumber("Right Trigger", rightTrigger);

      double rotate = 0.0;
      if (leftTrigger < rightTrigger) {
        rotate = -rightTrigger;
      } else {
        rotate = leftTrigger;
      }

      rotate = Math.copySign(rotate * rotate, rotate);
      rotate = rotateFilter.calculate(rotate) * Drive.kMaxAngularSpeed;
      SmartDashboard.putNumber("Rotate", rotate);
      return rotate;
    };

    Supplier<Optional<Translation2d>> translationSupplier = () -> {
      // Read gamepad joystick state, and apply slew rate limiters

      // X Move Velocity - Forward
      double xMove = xFilter.calculate(-m_driverController.getHID().getLeftY());

      // Y Move Velocity - Strafe
      double yMove = yFilter.calculate(-m_driverController.getHID().getLeftX());

      // if (controller.getHID().getLeftBumperButton()) {
      // yMove = 0;
      // }

      if (xMove == 0 && yMove == 0) {
        return Optional.empty();
      }

      Translation2d move = new Translation2d(xMove, yMove);

      return Optional.of(new Translation2d(
          // Scale the speed of the robot by using a quadratic input curve.
          // and convert the joystick values -1.0-1.0 to Meters Per Second
          Math.pow(move.getNorm(), 2) * Drive.kMaxSpeed,
          // Get the direction the joystick is pointing
          move.getAngle()));
    };

    // SmartDashboard.putNumber("Drive Forward", 0);
    // SmartDashboard.putNumber("Drive Strafe", 0);
    // SmartDashboard.putNumber("Drive Rotate", 0);
    m_drive.setDefaultCommand(Commands.runEnd(
      () -> {
        var translation = translationSupplier.get();

        double rotate = rotationSupplier.getAsDouble();
        double xMove = 0;
        double yMove = 0;

        if (translation.isPresent()) {
          xMove = translation.get().getX();
          yMove = translation.get().getY();
        }

        m_drive.driveRobotCentric(xMove, yMove, rotate, 0.02);

        // m_drive.driveRobotCentric(
        //   SmartDashboard.getNumber("Drive Forward", 0),
        //   SmartDashboard.getNumber("Drive Strafe", 0),
        //   SmartDashboard.getNumber("Drive Rotate", 0),
        //   0.02
        // );
      },
      () -> {
        m_drive.driveRobotCentric(0, 0, 0, 0.02);
      },
      m_drive
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
