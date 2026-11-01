package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveWithController extends Command {
    private final Drive drive;

    // Inputs
    private final Supplier<Optional<Translation2d>> translationSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldOrientedSupplier;
    private final DoubleSupplier slowModeSupplier;
    private final BooleanSupplier disableXMovementSupplier;
    private final BooleanSupplier disableYMovementSupplier;
    private final Supplier<Rotation2d> externalRobotHeadingSupplier;


    // State
    private final TrapezoidProfile.Constraints headingControllerConstraints =  new TrapezoidProfile.Constraints(Drive.kMaxAngularSpeed/2.0, 4*Math.PI);
    private final ProfiledPIDController robotHeadingController = new ProfiledPIDController(10.0, 0, 0, headingControllerConstraints);

    private boolean lastHeadingControllerEnabled = false;
    private Rotation2d latchedHeading = null;

    public DriveWithController(
        Drive drive,
        Supplier<Optional<Translation2d>> translationSupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier fieldOrientedSupplier,
        DoubleSupplier slowModeSupplier,
        BooleanSupplier disableXMovementSupplier,
        BooleanSupplier disableYMovementSupplier,

        // This is used to override the robots orientation while driving.
        // The user of this command should use it to supply either an vision heading override or a heading to have the robot face
        // while driving.
        Supplier<Rotation2d> fieldHeadingOverrideSupplier
    ) {

        this.drive = drive;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        this.slowModeSupplier = slowModeSupplier;
        this.disableXMovementSupplier = disableXMovementSupplier;
        this.disableYMovementSupplier = disableYMovementSupplier;
        this.externalRobotHeadingSupplier = fieldHeadingOverrideSupplier;

        addRequirements(drive);

        robotHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        lastHeadingControllerEnabled = false;
        latchedHeading = null;
    }

    @Override
    public void execute() {
        //
        // Robot State
        // 
        var driveHeading = drive.getHeading();

        //
        // Robot Movement
        //
        var translation = translationSupplier.get();

        double rotate = rotationSupplier.getAsDouble();
        double xMove = 0;
        double yMove = 0;

        if (translation.isPresent()) {
          xMove = translation.get().getX();
          yMove = translation.get().getY();
        }

        if (disableXMovementSupplier.getAsBoolean()) {
            xMove = 0;
        }
        if (disableYMovementSupplier.getAsBoolean()) {
            yMove = 0;
        }

        // Retrieve the external robot heading override
        // This will take precedence over the latched robot heading
        Rotation2d externalRobotHeading = externalRobotHeadingSupplier.get();

        var slowModeScaler = slowModeSupplier.getAsDouble();
        xMove *= slowModeScaler;
        yMove *= slowModeScaler;
        rotate *= slowModeScaler;

        if (MathUtil.isNear(0, rotate, 0.01)) {
            // We are not commanding the robot to rotate
            if (latchedHeading == null) {
                // We just switched from unlatched to latched
                // Capture the current heading
                latchedHeading = driveHeading;
            }
        } else {
            // Robot is being commanded to be rotated so clear the latch
            latchedHeading = null;
        }

        Rotation2d robotHeadingOverride = null;
        if (externalRobotHeading != null) {
            robotHeadingOverride = externalRobotHeading;

            // Clear the latched heading if an external heading was provided
            latchedHeading = null;
        } else if (latchedHeading != null) {
            robotHeadingOverride = latchedHeading;
        }

        //
        // Heading controller
        //
        if (robotHeadingOverride != null) {
            if (!lastHeadingControllerEnabled) {
                // Reset heading controller if we are entering it for the first time
                robotHeadingController.reset(new State(driveHeading.getRadians(), 0)); // TODO need to account for angular velocity
            }

            // Calculate the rotate command in Rad/Sec so we can reuse the PID values used in auto
            robotHeadingController.setGoal(robotHeadingOverride.getRadians());
            rotate = robotHeadingController.calculate(driveHeading.getRadians());
            rotate = MathUtil.clamp(rotate, -headingControllerConstraints.maxVelocity, headingControllerConstraints.maxVelocity);

            SmartDashboard.putNumber("DriveWithController/Robot Heading goal (deg)", robotHeadingOverride.getRadians());
            SmartDashboard.putNumber("DriveWithController/Robot Heading error (deg)", Math.toDegrees(robotHeadingController.getPositionError()));
            SmartDashboard.putNumber("DriveWithController/Robot heading override output", rotate);
        } else {
            SmartDashboard.putNumber("DriveWithController/Robot Heading goal (deg)", Double.NaN);
            SmartDashboard.putNumber("DriveWithController/Robot Heading error (deg)", Double.NaN);
            SmartDashboard.putNumber("DriveWithController/Robot heading override output", Double.NaN);
        }

 
        var fieldOriented = fieldOrientedSupplier.getAsBoolean();
        drive.drive(
            xMove,
            yMove,
            rotate,
            fieldOriented
        );
        
        SmartDashboard.putNumber("DriveWithController/xMove", xMove);
        SmartDashboard.putNumber("DriveWithController/yMove", yMove);
        SmartDashboard.putNumber("DriveWithController/rotate", rotate);
        SmartDashboard.putNumber("DriveWithController/Latched Robot heading", latchedHeading == null ? Double.NaN : latchedHeading.getDegrees());
        SmartDashboard.putNumber("DriveWithController/External Robot heading", externalRobotHeading == null ? Double.NaN : externalRobotHeading.getDegrees());
        SmartDashboard.putNumber("DriveWithController/Robot Heading Override", robotHeadingOverride == null ? Double.NaN : robotHeadingOverride.getDegrees());
        SmartDashboard.putBoolean("DriveWithController/Robot Heading Override enabled", robotHeadingOverride != null);
        SmartDashboard.putBoolean("DriveWithController/Field oriented", fieldOriented);
        lastHeadingControllerEnabled = robotHeadingOverride != null;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}