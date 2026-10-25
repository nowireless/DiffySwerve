package frc.lib.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.RobotController;
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

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States (TODO): 
    // [
    //  top motor angular velocity (rad/s), 
    //  bottom motor angular velocity (rad/s), 
    //  module angle (rad)
    // ]
    // Inputs (what we can "put in"):
    // [
    //  top_voltage (Volts), 
    //  bottom_voltage (Volts)
    // ],
    // Outputs (what we can measure):
    // [
    //  top motor angular velocity (rad/s), 
    //  bottom motor angular velocity (rad/s), 
    //  module angle (rad)
    // ]
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    // private final LinearSystem<N3,N2,N3> m_plant = new LinearSystem<>(
    //     null, 
    //     null, 
    //     null, 
    //     null
    // );

    // x: State Matrix:
    // [
    //     theta_module (rad)
    //     speed_wheel (rad/s)
    //     speed_steer (rad/s)
    // ]
    // Inputs:
    // [
    //     voltage_top (V)
    //     voltage_bottom (V)
    // ]
    // y: Outputs:
    // [
    //     theta_module (rad)
    //     speed_top_motor (rad/s)
    //     speed_bottom_motor (rad/s)
    // ]
    //
    //  
    //  Relationship between motor speeds and wheel & steer speeds
    //  speed_wheel = k_d (speed_top - speed_bottom)
    //  speed_steer = k_s (speed_top + speed_bottom)
    //
    //  speed_top =     (speed_wheel / (2 k_d)) + (speed_steer / (2 k_s))
    //  speed_bottom = -(speed_wheel / (2 k_d)) + (speed_steer / (2 k_s))
    //
    // Thus our C matrix should be:
    //      theta_module speed_wheel speed steer
    //     | 1                    0            0 | theta_module
    // C = | 0             1/(2k_d)     1/(2k_s) | speed_top_motor
    //     | 0            -1/(2k_d)     1/(2k_s) | speed_bottom_motor
    //
    // The output equation is:
    // y = Cx
    //
    //  Gains/Constants for the system
    //      a - Wheel Velocity Decay rate (1/s)
    //      d - Steering Velocity Decay rate (1/s)
    //      b - Wheel acceleration per volt applies to one motor (rad/s^2/V)
    //      c - Steering acceleration per volt applies to one motor (rad/s^2/V)
    //     kd -  Drive Gearing coefficient
    //     ks - Steer Gearing coefficient
    //
    //     acceleration_wheel = -a * speed_wheel + b(voltage_top - voltage_bottom)
    //     acceleration_steer = -d * speed_steer + c(voltage_top + voltage_bottom)
    //
    //     | 0  0  1 |
    // A = | 0 -a  0 |
    //     | 0  0 -d |
    //
    //
    //     | 0  0 |
    // B = | b -b |
    //     | c  c |
    //
    //     | 0 0 |
    // D = | 0 0 |
    //     | 0 0 |
    //
    //
    // Relating this to kA and kV
    //
    // voltage_drive = (voltage_top - voltage_bottom) / 2
    // voltage_steer = (voltage_top + voltage_bottom) / 2
    //
    // Generic Feedforward
    // voltage = kV * speed + kA * acceleration
    //
    // a = kv_drive/ka_drive
    // b = 1 / (2ka_drive)
    // d = kv_steer/ka_steer
    // c = 1/(2ka_steer)

    // a - Wheel Velocity Decay rate (1/s)
    private final double kWheelVelocityDecay = 0;

    // d - Steering Velocity Decay rate (1/s)
    private final double kSteerVelocityDecay = 0;
    
    // b - Wheel acceleration per volt applies to one motor (rad/s^2/V)
    private final double kWheelAcceleration = 0;
    
    // c - Steering acceleration per volt applies to one motor (rad/s^2/V)
    private final double kSteerAcceleration = 0;
    
    // kd -  Drive Gearing coefficient
    private final double kDriveGearingCoefficient = 0;
    
    // ks - Steer Gearing coefficient
    private final double kSteeringCoefficient = 0;


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
            0,                            0,
            kWheelAcceleration, -kWheelAcceleration,
            kSteerAcceleration,  kSteerAcceleration
        ),

        // C: Output Matrix
        // Output x States
        MatBuilder.fill(
            Nat.N3(), Nat.N3(),
             1,                         0,                               0,
            0,  1.0/(2.0*kDriveGearingCoefficient), 1.0/(2.0*kSteeringCoefficient),
            0, -1.0/(2.0*kDriveGearingCoefficient), 1.0/(2.0*kSteeringCoefficient)
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
