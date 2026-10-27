package frc.lib.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState();

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition();

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState);

    /**
     * Sets the desired voltage for the top and bottom motors.
     * 
     * @param topMotorVolts
     * @param bottomMotorVolts
     */
    public void setVoltage(double topMotorVolts, double bottomMotorVolts);


    /**
     * This method is called periodically, and is used 
     * to update swerve module-specific state.
     */
    default void periodic() {};
}