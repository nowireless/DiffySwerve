package frc.lib.hardware;

import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class SparkMini extends PWMMotorController {
   
    private final DoubleSupplier m_batteryVoltage;

    public SparkMini(final int channel) {
        this(channel, RobotController::getBatteryVoltage);
    }

    public SparkMini(final int channel, DoubleSupplier batteryVoltage) {
        super("Spark", channel);
        m_batteryVoltage = batteryVoltage;


        m_pwm.setBoundsMicroseconds(2500, 1510, 1500, 1490, 500);
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
        m_pwm.setSpeed(0.0);
        m_pwm.setZeroLatch();

        HAL.report(tResourceType.kResourceType_RevSPARK, getChannel() + 1);
    }

    @Override
    public void setVoltage(double outputVolts) {
        this.set(outputVolts / m_batteryVoltage.getAsDouble());
    }

}