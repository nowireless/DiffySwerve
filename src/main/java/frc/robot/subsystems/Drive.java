// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.swerve.DiffySwerveModuleV1;
import frc.lib.swerve.SwerveModule;

public class Drive extends SubsystemBase {

  //
  // Constant
  //

  public static final double kMaxSpeed = 1.5; // 3 meters per second
  public static final double kMaxAngularSpeed = 2.0*Math.PI; // 1/2 rotation per second

  // private final Translation2d m_frontLeftLocation = new Translation2d(0.13, 0.13);
  private final Translation2d m_frontRightLocation = new Translation2d(0.13, -0.13);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.13, 0.13);
  // private final Translation2d m_backRightLocation = new Translation2d(-0.13, -0.13);


  //
  // Hardware
  //

  private final TalonSRX m_pigionIMUTalon = new TalonSRX(1);
  private final PigeonIMU m_imu = new PigeonIMU(m_pigionIMUTalon);

  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;

  private final List<SwerveModule> m_modules;

  //
  // Pose Estimation
  //

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // m_frontLeftLocation,
      m_frontRightLocation,
      m_backLeftLocation
      // m_backRightLocation
  );

  // private final SwerveDrivePoseEstimator m_poseEstimator;


  /** Creates a new ExampleSubsystem. */
  public Drive(Power power) {

    //
    // Modules
    //

    m_frontRight = new DiffySwerveModuleV1(
      "Front Right",
      1, 
      0, 
      0, 1, 
      2, 3, 
      0, 
      Rotation2d.fromDegrees(67+180), 
      power::getVoltage
    );

    m_backLeft = new DiffySwerveModuleV1(
      "Back Left",
      4, 
      5, 
      4, 5, 
      6, 7, 
      1, 
      Rotation2d.fromDegrees(180+180), 
      power::getVoltage
    );

    m_modules = List.of(
      m_frontRight,
      m_backLeft
    );

    //
    // Post Estimation
    //

    // m_poseEstimator = new SwerveDrivePoseEstimator(
    //   m_kinematics,
    //   getHeading(),
    //   new SwerveModulePosition[] {
    //     // m_frontLeft.getPosition(),
    //     m_frontRight.getPosition(),
    //     m_backLeft.getPosition(),
    //     // m_backRight.getPosition()
    //   },
    //   Pose2d.kZero,
    //   VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    //   VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    // );

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

    //
    // Dashboard
    //

    // TODO reorg how diag data gets pushed to NT. Try to use the more nesting

    // Show a little swerve drive robot on elastic
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> 0, null);
        builder.addDoubleProperty("Front Left Velocity", () -> 0, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_backLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> 0, null);
        builder.addDoubleProperty("Back Right Velocity", () -> 0, null);

        builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
      }
    });

    //
    // Test Commands
    //

    SmartDashboard.putNumber("Drive Motor Test Volts", 6);
    SmartDashboard.putData("Drive Motor Test", this.runEnd(
      () -> {
        var volts = SmartDashboard.getNumber("Drive Motor Test Volts", 0);
        for (var m : m_modules) {
          m.setVoltage(volts, volts);
        }
      },
      this::stop
    ));
  }

  public Rotation2d getHeading() {
    // TODO in the future this should be getting the heading from the swerve drive pose estimator
    return Rotation2d.fromDegrees(m_imu.getFusedHeading());
  }


  public void seedFieldCentricDrive() {
    // TODO this is a hack right now
    m_imu.setFusedHeading(0);
  }


  @Override
  public void periodic() {
    var heading = getHeading();
  
    // This method will be called once per scheduler run
    for (SwerveModule module : m_modules) {
      module.periodic();
    }


    SmartDashboard.putNumber("Drive Heading (Deg)", heading.getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldOriented) {
    this.drive(xSpeed, ySpeed, rot, fieldOriented, 0.02);
  }


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldOriented, double periodSeconds) {
    // TODO thing about making this a "Request" and push this into the periodic method, so we dont read heading twice
    var chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    
    if (fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getHeading());
    }

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(chassisSpeeds, periodSeconds)
    );
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    for (int i = 0; i < m_modules.size(); i++) {
      m_modules.get(i).setDesiredState(swerveModuleStates[i]);
    }
  }

  public void stop() {
    for (var m : m_modules) {
      m.setVoltage(0, 0);
    }
  }

  //
  // Commands
  //
  


}
