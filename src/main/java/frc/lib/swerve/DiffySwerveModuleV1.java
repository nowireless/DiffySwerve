package frc.lib.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.hardware.SparkMini;
import edu.wpi.first.math.util.Units;

public class DiffySwerveModuleV1 implements SwerveModule {
    //
    // Constants
    //
    public static final double kWheelDiameter = Units.inchesToMeters(4);;
    public static final double kDriveGearing = new Gearing()
        .with(76, 21) // UltraPlanetary 4:1 really 3.61:1
        .with(27, 23) // UltraPlanetary to intermediate shaft
        .with(68, 14) // Intermediate shaft to outer module gear
        .with(14, 42) // Inner module gear to bevel shaft
        .with(28, 14) // Bevel reduction 2:1 
        .get();

    public static final double kMotorEncoderResolution = 7; // 7 pulses * x4 decode => 28 counts

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

    private final AnalogEncoder m_steerEncoder;

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


    // a - Wheel Velocity Decay rate (1/s)
    private final double kWheelVelocityDecay = 0;

    // d - Steering Velocity Decay rate (1/s)
    private final double kSteerVelocityDecay = 0;
    
    // b - Wheel acceleration per volt applies to one motor (rad/s^2/V)
    private final double kWheelAccelerationGain = 0;
    
    // c - Steering acceleration per volt applies to one motor (rad/s^2/V)
    private final double kSteerAccelerationGain = 0;
    
    // kd -  Drive Gearing coefficient
    private final double kDriveGearingCoefficient = 0;
    
    // ks - Steer Gearing coefficient
    private final double kSteeringGearingCoefficient = 0;


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
             1,                         0,                               0,
            0,  1.0/(2.0*kDriveGearingCoefficient), 1.0/(2.0*kSteeringGearingCoefficient),
            0, -1.0/(2.0*kDriveGearingCoefficient), 1.0/(2.0*kSteeringGearingCoefficient)
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


    // The state-space loop combines a controller, observer, feedforward and plant for easy control.    private final Linear

    public DiffySwerveModuleV1(String name, int topMotorPWM, int bottomMotorPWM, int topEncoderA, int topEncoderB, int bottomEncoderA, int bottomEncoderB, int steerEncoderAnalog, Rotation2d steerOffset, DoubleSupplier batteryVoltage) {
        m_name = name;

        m_topMotor = new SparkMini(topMotorPWM, batteryVoltage);
        m_bottomMotor = new SparkMini(bottomMotorPWM, batteryVoltage);

        m_topEncoder = new Encoder(topEncoderA, topEncoderB);
        m_topEncoder.setReverseDirection(true);
        // m_topEncoder.setDistancePerPulse(Math.PI * kWheelDiameter / (kMotorEncoderResolution * kDriveGearing));
        m_topEncoder.setDistancePerPulse(1.0/kMotorEncoderResolution);

        m_bottomEncoder = new Encoder(bottomEncoderA, bottomEncoderB);
        m_bottomEncoder.setReverseDirection(true);
        // m_bottomEncoder.setDistancePerPulse(Math.PI * kWheelDiameter / (kMotorEncoderResolution * kDriveGearing));
        m_bottomEncoder.setDistancePerPulse(1.0/kMotorEncoderResolution);


        var steerEncoderInput = new AnalogInput(steerEncoderAnalog);

        var steerEncoderTrigger = new AnalogTrigger(steerEncoderInput);

        m_steerEncoder = new AnalogEncoder(steerEncoderInput);
        m_steerEncoder.setInverted(false);

        m_steerOffset = steerOffset;
    }

    public Rotation2d getAngle() {
        // TODO need to handle rollover
        return Rotation2d.fromRotations(m_steerEncoder.get()).minus(m_steerOffset);
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

    @Override
    public void periodic() {
        //
        // Read Sensors
        //
        Rotation2d steerAngle = Rotation2d.fromRotations(m_steerEncoder.get());

        double topEncoderPosition = m_topEncoder.get();
        double topEncoderRateRPM = m_topEncoder.getRate() * 60;
        double topEncoderRateRadPS = Units.rotationsPerMinuteToRadiansPerSecond(topEncoderRateRPM);

        double bottomEncoderPosition = m_bottomEncoder.get();
        double bottomEncoderRateRPM = m_bottomEncoder.getRate() * 60;
        double bottomEncoderRateRadPS = Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoderRateRPM);

        //
        // Update Controller
        //

        // TODO

        //
        // Command Motors
        //

        // TODO


        // Temp hack to make motors spin
        if (RobotState.isEnabled()) {
            m_topMotor.setVoltage(10);
            m_bottomMotor.setVoltage(-10);
        } else {
            m_topMotor.stopMotor();
            m_bottomMotor.stopMotor();
        } 

        //
        // Publish telemetry
        //
        SmartDashboard.putNumber(m_name + ": Top Encoder rate (RPM)", topEncoderRateRPM);
        SmartDashboard.putNumber(m_name + ": Top Encoder position", topEncoderPosition);
        SmartDashboard.putNumber(m_name + ": Bottom Encoder position", bottomEncoderPosition);
        SmartDashboard.putNumber(m_name + ": Bottom Encoder rate (RPM)", bottomEncoderRateRPM);
        SmartDashboard.putNumber(m_name + ": Steer Angle", steerAngle.getDegrees());
    }
}
