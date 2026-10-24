package frc.lib.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.hardware.SparkEx;
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


        m_steerEncoder = new AnalogEncoder(steerEncoderAnalog);
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
        m_topMotor.set(0.2);
    }

    @Override
    public void periodic() {
        //
        // Read Sensors
        //
        double topEncoderPosition = m_topEncoder.get();
        double topEncoderRate = m_topEncoder.getRate();
        double bottomEncoderPosition = m_bottomEncoder.get();
        double bottomEncoderRate = m_bottomEncoder.getRate();
        Rotation2d steerAngle = Rotation2d.fromRotations(m_steerEncoder.get());

        // TODO Temp hack
        topEncoderRate *= 60;

        //
        // Update Controller
        //

        //
        // Command Motors
        //
        if (RobotState.isEnabled()) {
            m_topMotor.set(0.6);
            m_bottomMotor.set(-0.6);
        } else {
            m_topMotor.stopMotor();
            m_bottomMotor.stopMotor();
        } 

        //
        // Publish telemetry
        //
        SmartDashboard.putNumber(m_name + ": Top Encoder rate", topEncoderRate);
        SmartDashboard.putNumber(m_name + ": Top Encoder position", topEncoderPosition);
        SmartDashboard.putNumber(m_name + ": Bottom Encoder position", bottomEncoderPosition);
        SmartDashboard.putNumber(m_name + ": Bottom Encoder rate", bottomEncoderRate);
        SmartDashboard.putNumber(m_name + ": Steer Angle", steerAngle.getDegrees());
    }
}
