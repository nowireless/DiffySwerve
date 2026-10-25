package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(13, MotorType.kBrushless);

    public Turret() {

        var motorConfig = new SparkMaxConfig()
            .smartCurrentLimit(20)
            .voltageCompensation(12)
            .smartCurrentLimit(10)
            .idleMode(IdleMode.kCoast);
        
        motorConfig.signals
            .busVoltagePeriodMs(10);

        m_motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public double getBusVoltage() {
        return m_motor.getBusVoltage();
    }

}
