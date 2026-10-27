// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.swerve.DiffySwerveModuleV1;
import frc.lib.swerve.SwerveModule;

public class Drive extends SubsystemBase {

   public static final double kMaxSpeed = 1.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // private final Translation2d m_frontLeftLocation = new Translation2d(0.13, 0.13);
  private final Translation2d m_frontRightLocation = new Translation2d(0.13, -0.13);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.13, 0.13);
  // private final Translation2d m_backRightLocation = new Translation2d(-0.13, -0.13);

   private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // m_frontLeftLocation,
      m_frontRightLocation,
      m_backLeftLocation
      // m_backRightLocation
  );

  //
  // Hardware
  //

  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;

  private final List<SwerveModule> m_modules;

  /** Creates a new ExampleSubsystem. */
  public Drive(Power power) {

    //
    // Modules
    //

    m_frontRight = new DiffySwerveModuleV1(
      "Front Left",
      1, 
      0, 
      0, 1, 
      2, 3, 
      0, 
      Rotation2d.fromDegrees(67), 
      power::getVoltage
    );

    m_backLeft = new DiffySwerveModuleV1(
      "Back Left",
      4, 
      5, 
      4, 5, 
      6, 7, 
      1, 
      Rotation2d.fromDegrees(180), 
      power::getVoltage
    );

    m_modules = List.of(
      m_frontRight,
      m_backLeft
    );

    //
    // SysID
    //

    var flSysIDDrive = new DiffySwerveModuleV1.SysidDrive((DiffySwerveModuleV1) m_frontRight, this);
    SmartDashboard.putData("SysID Drive Dynamic Forward", flSysIDDrive.sysIdDynamic(Direction.kForward));
    SmartDashboard.putData("SysID Drive Dynamic Reverse", flSysIDDrive.sysIdDynamic(Direction.kReverse));
    SmartDashboard.putData("SysID Drive Quasistatic Forward", flSysIDDrive.sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData("SysID Drive Quasistatic Reverse", flSysIDDrive.sysIdQuasistatic(Direction.kReverse));

    var flSysIDSteer = new DiffySwerveModuleV1.SysidSteer((DiffySwerveModuleV1) m_frontRight, this);
    SmartDashboard.putData("SysID Steer Dynamic Forward", flSysIDSteer.sysIdDynamic(Direction.kForward));
    SmartDashboard.putData("SysID Steer Dynamic Reverse", flSysIDSteer.sysIdDynamic(Direction.kReverse));
    SmartDashboard.putData("SysID Steer Quasistatic Forward", flSysIDSteer.sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData("SysID Steer Quasistatic Reverse", flSysIDSteer.sysIdQuasistatic(Direction.kReverse));


    // setDefaultCommand(Commands.repeatingSequence(
    //   Commands.run(
    //     () -> m_frontRight.setDesiredState(new SwerveModuleState(
    //       0,
    //       Rotation2d.fromDegrees(0)
    //     )), 
    //     this
    //   ).withTimeout(5),
    //   Commands.run(
    //     () -> m_frontRight.setDesiredState(new SwerveModuleState(
    //       0,
    //       Rotation2d.fromDegrees(10)
    //     )), 
    //     this
    //   ).withTimeout(5),
    //   Commands.run(
    //     () -> m_frontRight.setDesiredState(new SwerveModuleState(
    //       0,
    //       Rotation2d.fromDegrees(0)
    //     )), 
    //     this
    //   ).withTimeout(5),
    //   Commands.run(
    //     () -> m_frontRight.setDesiredState(new SwerveModuleState(
    //       0,
    //       Rotation2d.fromDegrees(10)
    //     )), 
    //     this
    //   ).withTimeout(5)
    // ));
  }

  @Override
  public void periodic() {

    // m_frontRight.setDesiredState(new SwerveModuleState(
    //   0, 
    //   Rotation2d.fromDegrees(45))
    // );

    // m_frontRight.setVoltage(5, 5);

    // m_backLeft.setVoltage(0, 3);
    // m_backLeft.setDesiredState(new SwerveModuleState(
    //   0, 
    //   Rotation2d.fromDegrees(45))
    // );

  
    // This method will be called once per scheduler run
    for (SwerveModule module : m_modules) {
      module.periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void driveRobotCentric(double xSpeed, double ySpeed, double rot, double periodSeconds) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rot), periodSeconds)
    );
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[0]);
    m_backLeft.setDesiredState(swerveModuleStates[1]);
    // m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  //
  // Commands
  //

  // public Command TestFrontModule() {
  //   return Commands.run(
  //       ()-> m_frontRight.setDesiredState(new SwerveModuleState()),
  //       this
  //     );
  // }
}
