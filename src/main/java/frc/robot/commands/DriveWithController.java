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
    private final Supplier<Rotation2d> robotHeadingOverrideSupplier;


    // State
    private final TrapezoidProfile.Constraints headingControllerConstraints =  new TrapezoidProfile.Constraints(Drive.kMaxAngularSpeed/2.0, 4*Math.PI);
    private final ProfiledPIDController robotHeadingController = new ProfiledPIDController(10.0, 0, 0, headingControllerConstraints);

    private boolean lastHeadingControllerEnabled = false;

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
        this.robotHeadingOverrideSupplier = fieldHeadingOverrideSupplier;

        addRequirements(drive);

        robotHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
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

        Rotation2d fieldHeadingOverride = robotHeadingOverrideSupplier.get();

        var slowModeScaler = slowModeSupplier.getAsDouble();
        xMove *= slowModeScaler;
        yMove *= slowModeScaler;
        rotate *= slowModeScaler;

        //
        // Heading controller
        //
        if (fieldHeadingOverride != null) {
            if (!lastHeadingControllerEnabled) {
                // Reset heading controller if we are entering it for the first time
                robotHeadingController.reset(new State(drive.getHeading().getRadians(), 0)); // TODO need to account for angular velocity
            }

            // Calculate the rotate command in Rad/Sec so we can reuse the PID values used in auto
            robotHeadingController.setGoal(fieldHeadingOverride.getRadians());
            rotate = robotHeadingController.calculate(drive.getHeading().getRadians());
            rotate = MathUtil.clamp(rotate, -headingControllerConstraints.maxVelocity, headingControllerConstraints.maxVelocity);

            SmartDashboard.putNumber("DriveWithController/Robot Heading goal (deg)", fieldHeadingOverride.getRadians());
            SmartDashboard.putNumber("DriveWithController/Robot Heading error (deg)", Math.toDegrees(robotHeadingController.getPositionError()));
            SmartDashboard.putNumber("DriveWithController/Robot heading override output", rotate);
        } else {
            SmartDashboard.putNumber("DriveWithController/Robot Heading goal (deg)", 0.0);
            SmartDashboard.putNumber("DriveWithController/Robot Heading error (deg)", 0);
            SmartDashboard.putNumber("DriveWithController/Robot heading override output", 0);
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
        SmartDashboard.putBoolean("DriveWithController/Robot Heading Override enabled", fieldHeadingOverride != null);
        SmartDashboard.putBoolean("DriveWithController/Field oriented", fieldOriented);
        lastHeadingControllerEnabled = fieldHeadingOverride != null;
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