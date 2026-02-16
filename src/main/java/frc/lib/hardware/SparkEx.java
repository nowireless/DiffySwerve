package frc.lib.hardware;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SparkEx extends Spark {
   
    private final DoubleSupplier m_batteryVoltage;

    public SparkEx(final int channel, DoubleSupplier batteryVoltage) {
        super(channel);

        m_batteryVoltage = batteryVoltage;
    }

    @Override
    public void setVoltage(double outputVolts) {
        this.set(outputVolts / m_batteryVoltage.getAsDouble());
    }

}