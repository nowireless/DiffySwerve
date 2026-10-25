package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Power extends SubsystemBase {
    
    private final DoubleSupplier m_batteryVoltageSupplier;
    private double m_batteryVoltage = 20.0;

    public Power(DoubleSupplier busVoltage) {
        m_batteryVoltageSupplier = busVoltage;
    }

    public double getVoltage() {
        return m_batteryVoltage;
    }

    @Override
    public void periodic() {
        m_batteryVoltage = m_batteryVoltageSupplier.getAsDouble();

        SmartDashboard.putNumber("Battery Voltage", m_batteryVoltage);
    }
}
