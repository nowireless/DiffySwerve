package frc.lib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtil {
    
  /**
   * Optimizes the desired module state to minimize steering rotation, potentially reversing wheel 
   * direction. The resulting target angle is expressed in the continuous, unwrapped coordinate 
   * system of the current module angle.
   * 
   * This is intended to take a wrapped goal angle, and then apply it to a unwrapped current angle.
   *
   * 
   * @param desiredState The desired state of the module, values in this will be potentially mutated.
   * @param currentAngle The current module angle.
   */
  public static void optimizeUnwrapped(SwerveModuleState desiredState, Rotation2d currentAngle) {
    // Optimize the the module angle 
    desiredState.optimize(currentAngle);
    
    // Now we need to unwrap the wrapped angle
    desiredState.angle = Rotation2d.fromRadians(currentAngle.getRadians() + MathUtil.angleModulus(
        desiredState.angle.getRadians() - currentAngle.getRadians()
    ));
  }
}
