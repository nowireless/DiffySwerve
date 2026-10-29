package frc.lib.swerve;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * SwerveDrive contains the methods someone would expect out of a CTRE style SwerveDrive
 */
public interface SwerveDrive {

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(SwerveRequest request);


    /**
     * Gets the current state of the swerve drivetrain.
     * This includes information such as the pose estimate,
     * module states, and chassis speeds.
     *
     * @return Current state of the drivetrain
     */
    public SwerveDriveState getState();


    /**
     * Gets a copy of the current state of the swerve drivetrain.
     * This includes information such as the pose estimate,
     * module states, and chassis speeds.
     * <p>
     * This can be used to get a thread-safe copy of the state object.
     *
     * @return Copy of the current state of the drivetrain
     */
    public SwerveDriveState getStateCopy();

    /**
     * Zero's this swerve drive's odometry entirely.
     * <p>
     * This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything();

    /**
     * Resets the rotation of the robot pose to 0 from the
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}
     * perspective. This makes the current orientation of the robot the
     * X forward for field-centric maneuvers.
     * <p>
     * This is equivalent to calling {@link #resetRotation} with the operator
     * perspective rotation.
     */
    default void seedFieldCentric() {
        seedFieldCentric(Rotation2d.kZero);
    }

    /**
     * Resets the rotation of the robot pose to the given value from the
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}
     * perspective. This makes the current orientation of the robot minus
     * {@code rotation} the X forward for field-centric maneuvers.
     * <p>
     * This is equivalent to calling {@link #resetRotation} with
     * {@code rotation.plus(getOperatorForwardDirection())}.
     *
     * @param rotation Rotation to make the current rotation
     */
    public void seedFieldCentric(Rotation2d rotation);

    /**
     * Resets the pose of the robot. The pose should be from the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     *
     * @param pose Pose to make the current pose
     */
    public void resetPose(Pose2d pose);

    /**
     * Resets the translation of the robot pose without affecting rotation.
     * The translation should be from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance}
     * perspective.
     *
     * @param translation Translation to make the current translation
     */
    public void resetTranslation(Translation2d translation);


    /**
     * Resets the rotation of the robot pose without affecting translation.
     * The rotation should be from the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance}
     * perspective.
     *
     * @param rotation Rotation to make the current rotation
     */
    public void resetRotation(Rotation2d rotation);

    /**
     * Takes the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perpective direction
     * and treats it as the forward direction for
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}.
     * <p>
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     * <p>
     * This does not change the robot pose, which is in the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     * As a result, the robot pose may need to be reset using {@link #resetPose}.
     *
     * @param fieldDirection Heading indicating which direction is forward from
     *                       the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective
     */
    public void setOperatorPerspectiveForward(Rotation2d fieldDirection);

    /**
     * Returns the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perpective
     * direction that is treated as the forward direction for
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}.
     * <p>
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     *
     * @return Heading indicating which direction is forward from
     *         the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective
     */
    public Rotation2d getOperatorForwardDirection();

    /*
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want.
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds. Note that you must use a timestamp with an
     *                              epoch since system startup (i.e., the epoch of this
     *                              timestamp is the same epoch as
     *                              {@link Utils#getCurrentTimeSeconds}).
     *                              This means that you should use
     *                              {@link Utils#getCurrentTimeSeconds}
     *                              as your time source or sync the epochs.
     *                              An FPGA timestamp can be converted to the correct
     *                              timebase using {@link Utils#fpgaToCurrentTime}.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want.
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds. Note that you must use a timestamp with an
     *                                 epoch since system startup (i.e., the epoch of this
     *                                 timestamp is the same epoch as
     *                                 {@link Utils#getCurrentTimeSeconds}).
     *                                 This means that you should use
     *                                 {@link Utils#getCurrentTimeSeconds}
     *                                 as your time source or sync the epochs.
     *                                 An FPGA timestamp can be converted to the correct
     *                                 timebase using {@link Utils#fpgaToCurrentTime}.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);

        /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in vision measurements after the autonomous period, or to change
     * trust as distance to a vision target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these numbers to
     *                                 trust global measurements from vision less.
     *                                 This matrix is in the form [x, y, theta]ᵀ,
     *                                 with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs);

    /**
     * Sets the pose estimator's trust in robot odometry. This might be used to change
     * trust in odometry after an impact with the wall or traversing a bump.
     *
     * @param stateStdDevs Standard deviations of the pose estimate. Increase these
     *                     numbers to trust your state estimate less. This matrix is
     *                     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public void setStateStdDevs(Matrix<N3, N1> stateStdDevs);


    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The pose's timestamp. Note that you must use a timestamp
     *                         with an epoch since system startup (i.e., the epoch of
     *                         this timestamp is the same epoch as
     *                         {@link Utils#getCurrentTimeSeconds}).
     *                         This means that you should use
     *                         {@link Utils#getCurrentTimeSeconds}
     *                         as your time source in this case.
     *                         An FPGA timestamp can be converted to the correct
     *                         timebase using {@link Utils#fpgaToCurrentTime}.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is
     *         empty).
     */
    public Optional<Pose2d> samplePoseAt(double timestampSeconds);

    /**
     * Gets the locations of the swerve modules.
     * 
     * @return Reference to the array of swerve module locations
     */
    public Translation2d[] getModuleLocations();

    /**
     * Gets the current orientation of the robot as a {@link Rotation3d} from
     * the Pigeon 2 quaternion values.
     *
     * @return The robot orientation as a {@link Rotation3d}
     */
    public Rotation3d getRotation3d();
}
