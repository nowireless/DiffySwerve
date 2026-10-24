package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Battery extends SubsystemBase {
    
    private final TalonSRX m_talon = new TalonSRX(1);
    private double m_batteryVoltage;

    private final AnalogInput m_voltageSensor = new AnalogInput(3);
    
    public Battery() {
        m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10);

        // m_voltageSensor.setAverageBits(10);
    }

    public double getVoltage() {
        return m_batteryVoltage;
    }

    @Override
    public void periodic() {
        m_batteryVoltage = m_voltageSensor.getAverageVoltage()/RobotController.getVoltage5V() * 25.0;

        SmartDashboard.putNumber("Battery Voltage", m_batteryVoltage);
    }
}
