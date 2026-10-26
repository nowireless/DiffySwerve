package frc.lib.swerve;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import frc.lib.hardware.SparkMini;
import frc.lib.hardware.UnwrappedAnalogEncoder;


public class DiffySwerveModuleV1 implements SwerveModule {
    public enum Mode {
        kVoltage,
        kLQR
    }

    //
    // Constants
    //

    public static final double kMotorEncoderResolution = 7; // 7 pulses * x4 decode => 28 counts
    public static final double kWheelDiameter = Units.inchesToMeters(4);

    //
    // Config
    //
    private final Rotation2d m_steerOffset;
    private final String m_name;

    //
    // Hardware
    // 
    private final SparkMini m_topMotor;
    private final SparkMini m_bottomMotor;

    private final Encoder m_bottomEncoder;
    private final Encoder m_topEncoder;

    private final UnwrappedAnalogEncoder m_steerEncoder;

    //
    // State Space
    //

    // Differential Swerve State-Space Plant
    //
    // ==========================================================
    // System Definition
    // ==========================================================
    //
    // x: States:
    // [
    //     theta_module (rad)
    //     velocity_wheel (m/s) - Wheel surface linear velocity
    //     velocity_steer (rad/s) - Module steering angular velocity
    // ]
    //
    // u: Inputs:
    // [
    //     voltage_top (V)
    //     voltage_bottom (V)
    // ]
    //
    // y: Outputs (raw sensor measurements):
    // [
    //     theta_module (rad)
    //     velocity_top_motor (rad/s)
    //     velocity_bottom_motor (rad/s)
    // ]
    //
    // ==========================================================
    // Gearing Definitions
    // ==========================================================
    //
    // Gd - Motor-to-wheel reduction during pure driving
    // Gs - Motor-to-module reduction during pure steering
    // r  - Effective wheel radius (m)
    //
    // Gd and Gs are dimensionless ratios of angular velocities.
    //
    // The differential operates in two modes:
    //
    // Pure driving:
    //     Motors rotate at equal and opposite angular velocities.
    //     The wheel rotates while the module angle stays fixed.
    //
    // Pure steering:
    //     Motors rotate at equal angular velocities.
    //     The module rotates without driving the wheel.
    //
    // ==========================================================
    // Deriving Drive Gearing Coefficient (kd)
    // ==========================================================
    //
    // Start with the differential relationship:
    //
    // velocity_wheel_angular =
    //     kd * (velocity_top_motor - velocity_bottom_motor)
    //
    // For pure driving, define a motor speed magnitude:
    //
    // velocity_top_motor    =  velocity_motor
    // velocity_bottom_motor = -velocity_motor
    //
    // Substitute:
    //
    // velocity_wheel_angular =
    //     kd * (velocity_motor - (-velocity_motor))
    //
    // velocity_wheel_angular = 2 * kd * velocity_motor
    //
    // Rearrange:
    //
    // kd = velocity_wheel_angular / (2 * velocity_motor)
    //
    // Define the conventional pure-drive gear reduction:
    //
    // Gd = velocity_motor / velocity_wheel_angular
    //
    // Substituting:
    //
    // kd = 1 / (2 * Gd)
    //
    // Now convert wheel angular velocity to linear velocity:
    //
    // velocity_wheel = r * velocity_wheel_angular
    //
    // Therefore:
    //
    // velocity_wheel =
    //     (r / (2 * Gd)) *
    //     (velocity_top_motor - velocity_bottom_motor)
    //
    // ==========================================================
    // Deriving Steering Gearing Coefficient (ks)
    // ==========================================================
    //
    // Start with the differential relationship:
    //
    // velocity_steer =
    //     ks * (velocity_top_motor + velocity_bottom_motor)
    //
    // For pure steering:
    //
    // velocity_top_motor    = velocity_motor
    // velocity_bottom_motor = velocity_motor
    //
    // Substitute:
    //
    // velocity_steer = ks * (2 * velocity_motor)
    //
    // velocity_steer = 2 * ks * velocity_motor
    //
    // Rearrange:
    //
    // ks = velocity_steer / (2 * velocity_motor)
    //
    // Define the conventional pure-steering reduction:
    //
    // Gs = velocity_motor / velocity_steer
    //
    // Substituting:
    //
    // ks = 1 / (2 * Gs)
    //
    // Therefore:
    //
    // velocity_steer =
    //     (1 / (2 * Gs)) *
    //     (velocity_top_motor + velocity_bottom_motor)
    //
    // ==========================================================
    // Deriving the Output Matrix (C)
    // ==========================================================
    //
    // We have established:
    //
    // velocity_wheel =
    //     (r / (2 * Gd)) *
    //     (velocity_top_motor - velocity_bottom_motor)
    //
    // velocity_steer =
    //     (1 / (2 * Gs)) *
    //     (velocity_top_motor + velocity_bottom_motor)
    //
    // Rearrange the drive equation:
    //
    // velocity_top_motor - velocity_bottom_motor =
    //     (2 * Gd / r) * velocity_wheel
    //
    // Rearrange the steering equation:
    //
    // velocity_top_motor + velocity_bottom_motor =
    //     2 * Gs * velocity_steer
    //
    // Add the equations and divide by two:
    //
    // velocity_top_motor =
    //     (Gd / r) * velocity_wheel
    //     + Gs * velocity_steer
    //
    // Subtract the drive equation from the steering equation,
    // then divide by two:
    //
    // velocity_bottom_motor =
    //     -(Gd / r) * velocity_wheel
    //     + Gs * velocity_steer
    //
    // Our output vector is:
    //
    // y = [
    //     theta_module
    //     velocity_top_motor
    //     velocity_bottom_motor
    // ]
    //
    // Therefore:
    //
    //          theta_module  velocity_wheel  velocity_steer
    //        |      1              0               0     |
    // C =    |      0            Gd/r              Gs    |
    //        |      0           -Gd/r              Gs    |
    //
    // Output equation:
    //
    // y = Cx + Du
    //
    // Since there is no direct voltage-to-output feedthrough:
    //
    // D = 0
    //
    // Therefore:
    //
    // y = Cx
    //
    // ==========================================================
    // System Dynamics
    // ==========================================================
    //
    // Define modal motor voltages:
    //
    // voltage_drive =
    //     (voltage_top - voltage_bottom) / 2
    //
    // voltage_steer =
    //     (voltage_top + voltage_bottom) / 2
    //
    // Characterize the two modes using feedforward:
    //
    // voltage = kV * velocity + kA * acceleration
    //
    // For driving, velocity is measured in m/s.
    // For steering, velocity is measured in rad/s.
    //
    // Static friction (kS) is ignored in this linear model.
    //
    // ----------------------------------------------------------
    // Drive Dynamics
    // ----------------------------------------------------------
    //
    // Feedforward equation:
    //
    // voltage_drive =
    //     kV_drive * velocity_wheel
    //     + kA_drive * acceleration_wheel
    //
    // Rearrange to solve for acceleration:
    //
    // kA_drive * acceleration_wheel =
    //     voltage_drive - kV_drive * velocity_wheel
    //
    // acceleration_wheel =
    //     -(kV_drive / kA_drive) * velocity_wheel
    //     + (1 / kA_drive) * voltage_drive
    //
    // Substitute the modal voltage:
    //
    // acceleration_wheel =
    //     -(kV_drive / kA_drive) * velocity_wheel
    //     + (1 / (2 * kA_drive)) *
    //       (voltage_top - voltage_bottom)
    //
    // Define:
    //
    // a = kV_drive / kA_drive
    // b = 1 / (2 * kA_drive)
    //
    // Therefore:
    //
    // acceleration_wheel =
    //     -a * velocity_wheel
    //     + b * (voltage_top - voltage_bottom)
    //
    // ----------------------------------------------------------
    // Steering Dynamics
    // ----------------------------------------------------------
    //
    // Feedforward equation:
    //
    // voltage_steer =
    //     kV_steer * velocity_steer
    //     + kA_steer * acceleration_steer
    //
    // Rearrange:
    //
    // acceleration_steer =
    //     -(kV_steer / kA_steer) * velocity_steer
    //     + (1 / kA_steer) * voltage_steer
    //
    // Substitute the modal voltage:
    //
    // acceleration_steer =
    //     -(kV_steer / kA_steer) * velocity_steer
    //     + (1 / (2 * kA_steer)) *
    //       (voltage_top + voltage_bottom)
    //
    // Define:
    //
    // d = kV_steer / kA_steer
    // c = 1 / (2 * kA_steer)
    //
    // Therefore:
    //
    // acceleration_steer =
    //     -d * velocity_steer
    //     + c * (voltage_top + voltage_bottom)
    //
    // ----------------------------------------------------------
    // Module Angle Dynamics
    // ----------------------------------------------------------
    //
    // By definition:
    //
    // d(theta_module)/dt = velocity_steer
    //
    // ==========================================================
    // State-Space Matrices
    // ==========================================================
    //
    // The continuous-time system is:
    //
    // dx/dt = Ax + Bu
    // y     = Cx + Du
    //
    // ----------------------------------------------------------
    // A: System Matrix (States x States)
    // ----------------------------------------------------------
    //
    //     | 0   0   1 |
    // A = | 0  -a   0 |
    //     | 0   0  -d |
    //
    // ----------------------------------------------------------
    // B: Input Matrix (States x Inputs)
    // ----------------------------------------------------------
    //
    //     |  0   0 |
    // B = |  b  -b |
    //     |  c   c |
    //
    // ----------------------------------------------------------
    // C: Output Matrix (Outputs x States)
    // ----------------------------------------------------------
    //
    //     | 1     0     0 |
    // C = | 0   Gd/r    Gs |
    //     | 0  -Gd/r    Gs |
    //
    // ----------------------------------------------------------
    // D: Feedthrough Matrix (Outputs x Inputs)
    // ----------------------------------------------------------
    //
    //     | 0  0 |
    // D = | 0  0 |
    //     | 0  0 |
    //
    // ==========================================================
    // Gains / Constants
    // ==========================================================
    //
    // a  - Wheel velocity decay rate (1/s)
    // b  - Wheel linear acceleration gain ((m/s^2)/V)
    // c  - Steering angular acceleration gain ((rad/s^2)/V)
    // d  - Steering velocity decay rate (1/s)
    //
    // Gd - Pure-drive motor-to-wheel reduction (dimensionless)
    // Gs - Pure-steering motor-to-module reduction (dimensionless)
    // r  - Effective wheel radius (m)
    //
    // kd - Differential drive coefficient = 1 / (2 * Gd)
    // ks - Differential steer coefficient = 1 / (2 * Gs)
    //
    // Feedforward relationships:
    //
    // a = kV_drive / kA_drive
    // b = 1 / (2 * kA_drive)
    //
    // d = kV_steer / kA_steer
    // c = 1 / (2 * kA_steer)
    //
    // Drive characterization units:
    //
    // kV_drive: V / (m/s)
    // kA_drive: V / (m/s^2)
    //
    // Steering characterization units:
    //
    // kV_steer: V / (rad/s)
    // kA_steer: V / (rad/s^2)
    //
    // ==========================================================
    // Modeling Assumptions
    // ==========================================================
    //
    // 1. Both motor-to-module paths are symmetric.
    // 2. Drive and steering dynamics are approximately decoupled.
    // 3. Encoder polarity matches the differential equations.
    // 4. The effective wheel radius is constant.
    // 5. Wheel slip is not modeled.
    // 6. Static friction and other nonlinear effects are ignored.
    // 7. Module angle is continuous (unwrapped) in the model.
    // 8. Inputs are actual applied motor voltages.
    // 9. Gd and Gs are effective pure-mode reductions and
    //    must be verified against the actual mechanism.
    //
    // ==========================================================
    // Deriving Wheel Distance 
    // ==========================================================
    //
    // Recall:
    // velocity_wheel =
    //     (r / (2 * Gd)) *
    //     (velocity_top_motor - velocity_bottom_motor)
    //
    // Integrate the above equation to get position from velocity:
    // distance_wheel = (r / (2 * Gd)) * (angle_top_motor -  angle_bottom_motor) + C
    // 
    // We can ignore the + C as we will just reset the encoders if we want a 0 distance.
    // distance_wheel = (r / (2 * Gd)) * (angle_top_motor -  angle_bottom_motor)

    // r  - Effective wheel radius (m)
    public static final double kWheelRadius = kWheelDiameter / 2.0;

    // Gd  Pure-drive motor-to-wheel reduction (dimensionless)
    private final double kGearingMotorToWheel = new Gearing()
        .with(76, 21) // UltraPlanetary 4:1 really 3.61:1
        .with(27, 23) // UltraPlanetary to intermediate shaft
        .with(68, 14) // Intermediate shaft to outer module gear
        .with(14, 42) // Inner module gear to bevel shaft
        .with(28, 14) // Bevel reduction 2:1 
        .get();

    // Gs - Pure-steering motor-to-module reduction (dimensionless)
    private final double kGearingMotorToModule = new Gearing()
        .with(76, 21) // UltraPlanetary 4:1 really 3.61:1
        .with(27, 23) // UltraPlanetary to intermediate shaft
        .with(68, 14) // Intermediate shaft to outer module gear
        .get();
    

    // a - Wheel Velocity Decay rate (1/s)
    private final double kWheelVelocityDecay = 0;

    // d - Steering Velocity Decay rate (1/s)
    private final double kSteerVelocityDecay = 0;
    
    // b - Wheel linear acceleration gain ((m/s^2)/V)
    private final double kWheelAccelerationGain = 0;
    
    // c - Steering angular acceleration gain ((rad/s^2)/V)
    private final double kSteerAccelerationGain = 0;

    private LinearSystem<N3, N2, N3> m_plant = new LinearSystem<>(
        // A: System Matrix
        // State x State
        MatBuilder.fill(
            Nat.N3(), Nat.N3(),
            0,            0,                    1,
            0, -kWheelVelocityDecay,                    0,
            0,                    0, -kSteerVelocityDecay 
        ),

        // B: Input Matrix
        // State x Input
        MatBuilder.fill(
            Nat.N3(), Nat.N2(),
            0,                                    0,
            kWheelAccelerationGain, -kWheelAccelerationGain,
            kSteerAccelerationGain,  kSteerAccelerationGain
        ),

        // C: Output Matrix
        // Output x States
        MatBuilder.fill(
            Nat.N3(), Nat.N3(),
             1,                           0,                     0,
            0,  kGearingMotorToWheel / kWheelRadius, kGearingMotorToModule,
            0, -kGearingMotorToWheel / kWheelRadius, kGearingMotorToModule
        ),

        // D: Feedthrough Matrix
        // Output x Inputs
        MatBuilder.fill(
            Nat.N3(), Nat.N2(),
             0, 0,
            0, 0,
            0, 0 
        )
    );


    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N3, N2, N3> m_observer = new KalmanFilter<>(
        Nat.N3(),
        Nat.N3(),
        m_plant,

        // How accurate we think our model is
        //
        // State model standard deviations
        // [module angle (rad),
        //  wheel velocity (m/s),
        //  steering velocity (rad/s)]
        VecBuilder.fill(
            0.1, // Module angle model uncertainty (rad)
            0.5, // Wheel velocity model uncertainty (m/s)
            0.5  // Steering velocity model uncertainty (rad/s)
        ), 

        // How accurate we think our encoder
        // data is
        //
        // Measurement standard deviations
        // [absolute module angle (rad),
        //  top motor velocity (rad/s),
        //  bottom motor velocity (rad/s)]
        VecBuilder.fill(
            0.01, // Absolute encoder angle
            1.0,  // Top motor velocity
            1.0   // Bottom motor velocity
        ),
        0.020
    );


    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N3, N2, N3> m_controller = new LinearQuadraticRegulator<N3, N2, N3>(
        m_plant,
        VecBuilder.fill(
            0.10, // Module angle (rad) ~5.7 degrees
            0.25, // Wheel velocity (m/s)
            1.00  // Steering velocity (rad/s)
        ),
        VecBuilder.fill(
             12.0, // Top motor voltage (V)
            12.0  // Bottom motor voltage (V)
        ),
        0.020
    );

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N3, N2, N3> m_loop =
        new LinearSystemLoop<>(m_plant, m_controller, m_observer, 10.0, 0.020);


    //
    // Module State
    //
    private Mode m_mode = Mode.kVoltage; 
    private double m_voltageTopMotor = 0.0;
    private double m_voltageBottomMotor = 0.0;
    private SwerveModuleState m_unfilteredModuleState = new SwerveModuleState();
    private SwerveModulePosition m_unfilteredModulePosition = new SwerveModulePosition();
    private double m_unfilteredSteerVelocityEncoders = 0.0; // Radians, we want this unwrapped 
    private double m_unfilteredSteerAngleEncoders = 0.0; // Radians/Sec, we want this unwrapped 

    public DiffySwerveModuleV1(String name, int topMotorPWM, int bottomMotorPWM, int topEncoderA, int topEncoderB, int bottomEncoderA, int bottomEncoderB, int steerEncoderAnalog, Rotation2d steerOffset, DoubleSupplier batteryVoltage) {
        m_name = name;

        m_topMotor = new SparkMini(topMotorPWM, batteryVoltage);
        m_bottomMotor = new SparkMini(bottomMotorPWM, batteryVoltage);

        m_topEncoder = new Encoder(topEncoderA, topEncoderB);
        m_topEncoder.setReverseDirection(true);
        m_topEncoder.setDistancePerPulse(1.0/kMotorEncoderResolution);

        m_bottomEncoder = new Encoder(bottomEncoderA, bottomEncoderB);
        m_bottomEncoder.setReverseDirection(true);
        m_bottomEncoder.setDistancePerPulse(1.0/kMotorEncoderResolution);

        m_steerEncoder = new UnwrappedAnalogEncoder(steerEncoderAnalog);

        m_steerOffset = steerOffset;
    }


    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public SwerveModulePosition getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // TODO Auto-generated method stub
        // m_topMotor.set(0.2);
    }

    // TODO should we use the fancy unit API for this?
    public void setVoltage(double topMotorVolts, double bottomMotorVolts) {
        m_mode = Mode.kVoltage;

        // Cap to +/- volts
        m_voltageTopMotor = MathUtil.clamp(topMotorVolts, -15, -15);
        m_voltageBottomMotor = MathUtil.clamp(bottomMotorVolts, -15, -15);
    }

    @Override
    public void periodic() {
        //
        // Read Sensors
        //
        Rotation2d unfilteredSteerAngle = Rotation2d.fromRotations(-m_steerEncoder.get())
            .minus(m_steerOffset);

        Rotation2d angleTopMotor = Rotation2d.fromRotations(m_topEncoder.getDistance());
        double topEncoderRateRPM = m_topEncoder.getRate() * 60.0;
        double velocityTopMotor = Units.rotationsPerMinuteToRadiansPerSecond(topEncoderRateRPM);

        Rotation2d angleBottomMotor = Rotation2d.fromRotations(m_bottomEncoder.getDistance());
        double bottomEncoderRateRPM = m_bottomEncoder.getRate() * 60.0;
        double velocityBottomMotor = Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoderRateRPM);

        //
        // Update unfiltered Module State
        //

        // TODO should this come from the kalman filter or just use raw sensor readings?
        // This will be fed into the swerve drive post estimator at some point, not sure
        // if that matters or not.
        double unfilteredVelocityWheel =
            (kWheelRadius / (2.0 * kGearingMotorToWheel)) *
            (velocityTopMotor - velocityBottomMotor);
        
        m_unfilteredModuleState.speedMetersPerSecond = unfilteredVelocityWheel;
        m_unfilteredModuleState.angle = unfilteredSteerAngle;
        

        double unfilteredWheelDistance = 
            (kWheelRadius / (2.0 * kGearingMotorToWheel)) * 
            (angleTopMotor.getRadians() -  angleBottomMotor.getRadians());

        m_unfilteredModulePosition.distanceMeters = unfilteredWheelDistance;
        m_unfilteredModulePosition.angle = unfilteredSteerAngle;

        m_unfilteredSteerVelocityEncoders =
            (velocityTopMotor + velocityBottomMotor) / 
            (2.0 * kGearingMotorToModule);
        m_unfilteredSteerAngleEncoders = 
            (angleTopMotor.getRadians() + angleBottomMotor.getRadians())
            / (2.0 * kGearingMotorToModule);

        //
        // Update Controller
        //

        switch (m_mode) {
        case kVoltage:
            // Nothing to do there
            break;
        case kLQR:
            // TODO
            break;
        default:
            m_voltageTopMotor = 0.0;
            m_voltageBottomMotor = 0.0;
        }
        

        //
        // Command Motors
        //
        m_topMotor.setVoltage(m_voltageTopMotor);
        m_bottomMotor.setVoltage(m_voltageBottomMotor);

        
        //
        // Publish telemetry
        //
        SmartDashboard.putNumber(m_name + ": Top Applied Voltage", m_voltageTopMotor);
        SmartDashboard.putNumber(m_name + ": Top Encoder angle (Deg)", angleTopMotor.getDegrees());
        SmartDashboard.putNumber(m_name + ": Top Encoder rate (RPM)", topEncoderRateRPM);
        SmartDashboard.putNumber(m_name + ": Bottom Applied Voltage", m_voltageBottomMotor);
        SmartDashboard.putNumber(m_name + ": Bottom Encoder angle (Deg)", angleBottomMotor.getDegrees());
        SmartDashboard.putNumber(m_name + ": Bottom Encoder rate (RPM)", bottomEncoderRateRPM);
        SmartDashboard.putNumber(m_name + ": Unfiltered Steer Angle", unfilteredSteerAngle.getDegrees());
        SmartDashboard.putNumber(m_name + ": Unfiltered Velocity (m/s)", unfilteredVelocityWheel);
        SmartDashboard.putNumber(m_name + ": Unfiltered Distance (m)", unfilteredWheelDistance);
        SmartDashboard.putString(m_name + ": Mode", m_mode.toString());
    }

    /**
     * SysidDrive is a helper class to wire up a swerve module to SysID
     * 
     * This one is used to calibrate drive motion.
     */
    public static class SysidDrive {
        
        private final MutVoltage m_appliedVolts = Volts.mutable(0);
        private final MutDistance m_distance = Meters.mutable(0);
        private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

        private final SysIdRoutine m_sysidRoutine;

        public SysidDrive(DiffySwerveModuleV1 module, Subsystem subsystem) {
            m_sysidRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    voltage -> {
                        // To make the module move the wheel the motors must oppose.
                        module.setVoltage(voltage.in(Volts), -voltage.in(Volts));
                    },
                    log -> {
                        log.motor("module")
                            // I think we are going to cheat and just grab the voltage that was set
                            // ideally this would be the applied motor output voltage from the motor 
                            // controller but I don't got that with my setup. In theory voltage 
                            // compensation is turned on so we can trust it.
                            .voltage(m_appliedVolts.mut_replace(module.m_voltageTopMotor, Volts))
                            .linearPosition(m_distance.mut_replace(module.m_unfilteredModulePosition.distanceMeters, Meters))
                            .linearVelocity(m_velocity.mut_replace(module.m_unfilteredModuleState.speedMetersPerSecond, MetersPerSecond));   
                    },
                    subsystem,
                    "DiffyServeModule_" + module.m_name + "_Drive"
                )
            );
        }

        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return m_sysidRoutine.quasistatic(direction);
        }
    
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return m_sysidRoutine.dynamic(direction);
        }
    }

    /**
     * SysidSteer is a helper class to wire up a swerve module to SysID
     * 
     * This one is used to calibrate steer motion.
     */
    public static class SysidSteer {
        
        private final MutVoltage m_appliedVolts = Volts.mutable(0);
        private final MutAngle m_distance = Radians.mutable(0);
        private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

        private final SysIdRoutine m_sysidRoutine;

        public SysidSteer(DiffySwerveModuleV1 module, Subsystem subsystem) {
            m_sysidRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    voltage -> {
                        // To make the module spin the wheel the motors must be the same.
                        module.setVoltage(voltage.in(Volts), voltage.in(Volts));
                    },
                    log -> {
                        log.motor("module")
                            // I think we are going to cheat and just grab the voltage that was set
                            // ideally this would be the applied motor output voltage from the motor 
                            // controller but I don't got that with my setup. In theory voltage 
                            // compensation is turned on so we can trust it.
                            .voltage(m_appliedVolts.mut_replace(module.m_voltageTopMotor, Volts))
                            .angularPosition(m_distance.mut_replace(module.m_unfilteredSteerAngleEncoders, Radians))
                            .angularVelocity(m_velocity.mut_replace(module.m_unfilteredSteerVelocityEncoders, RadiansPerSecond));   
                    },
                    subsystem,
                    "DiffyServeModule_" + module.m_name + "_Steer"
                )
            );
        }

        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return m_sysidRoutine.quasistatic(direction);
        }
    
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return m_sysidRoutine.dynamic(direction);
        }
    }
}
