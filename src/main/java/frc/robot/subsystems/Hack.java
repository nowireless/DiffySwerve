package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.SparkMini;

public class Hack extends SubsystemBase {
    
    // private final CANcoder m_encoder = new CANcoder(0);
    private final SparkMini m_motorA = new SparkMini(4, () -> 20);
    // private final Spark m_motorB = new Spark(5);
    

    public Hack() {
        // m_encoder.getVelocity().setUpdateFrequency(10);
        // m_encoder.getSupplyVoltage().setUpdateFrequency(20);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Hack: Encoder Supply Voltage", m_encoder.getSupplyVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Hack: Encoder Velocity (RPS)", m_encoder.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Hack: Encoder Velocity scaled (RPS)", m_encoder.getVelocity().getValueAsDouble() * 27.0/23.0 * 3.61);
        // SmartDashboard.putNumber("Hack: Encoder Velocity (RPM)", m_encoder.getVelocity().getValueAsDouble() * 60.0);
        // SmartDashboard.putNumber("Hack: Encoder Velocity scaled (RPM)", m_encoder.getVelocity().getValueAsDouble() * 27.0/23.0 * 3.61 * 60.0);
        if (RobotState.isEnabled()) {
            m_motorA.set(0.5);
            // m_motorB.set(0.5);
        } else {
            m_motorA.stopMotor();
            // m_motorB.stopMotor();
        }
    }
}
