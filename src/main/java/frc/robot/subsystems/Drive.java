// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.swerve.DiffySwerveModuleV1;
import frc.lib.swerve.SwerveModule;

public class Drive extends SubsystemBase {

  private final SwerveModule m_frontRight;

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

    m_modules = List.of(
      m_frontRight
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
  }

  @Override
  public void periodic() {

    m_frontRight.setDesiredState(new SwerveModuleState(
      0, 
      Rotation2d.fromDegrees(45))
    );

    // m_frontRight.setVoltage(5, 5);
  
    // This method will be called once per scheduler run
    for (SwerveModule module : m_modules) {
      module.periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
