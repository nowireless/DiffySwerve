// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.DiffySwerveModuleV1;
import frc.lib.swerve.SwerveModule;

public class Drive extends SubsystemBase {

  private final SwerveModule m_frontRight = new DiffySwerveModuleV1(
    "Front Left",
    1, 
    0, 
    0, 1, 
    2, 3, 
    0, 
    Rotation2d.fromDegrees(0), 
    () -> 20
  );

  private final List<SwerveModule> m_modules = List.of(
    m_frontRight
  );

  /** Creates a new ExampleSubsystem. */
  public Drive() {}

  @Override
  public void periodic() {
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
