package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public TalonFX lowerMotor, upperMotor, agitatorMotor;
    public double testSpeed = 0;
    private VelocityVoltage m_velocity = new VelocityVoltage(0);


    public Shooter() {
        lowerMotor = new TalonFX(Constants.Motors.ShooterLow);
        upperMotor = new TalonFX(Constants.Motors.ShooterHigh);

        var upperConfig = new Slot0Configs();
        var lowerConfig = new Slot0Configs();

        upperConfig.kP = 0.8;
        upperConfig.kV = 0.12;

        lowerConfig.kP = 0.35;
        lowerConfig.kV = 0.12;

        upperMotor.getConfigurator().apply(upperConfig, 0.05);
        lowerMotor.getConfigurator().apply(lowerConfig, 0.05);
        // agitatorMotor = new TalonFX(Constants.Motors.Agitator);
    }

    /**
     * @param power The power to set both shooter motors to. 
     *      Value should be in the range [-1, 1] where 1 is full speed forward.
     *      Invert the motors in the constructor if behavior is not as described above.
     */

   
    public void setLowerPower(double rps) {
        lowerMotor.setControl(m_velocity.withVelocity(rps));
    }

    public void stopLower() {
        lowerMotor.stopMotor();
    }

    // PID-based, better consistency
    public void setUpperPower(double rps) {
          upperMotor.setControl(m_velocity.withVelocity(rps));
    }

    public void stopUpper() {   
        upperMotor.stopMotor();
    }

    public void addTestSpeed(double increment) {
        testSpeed = MathUtil.clamp(testSpeed+increment, 0.0, 1.0);
        System.out.println("Test Speed: " + testSpeed);
    }

    /* Sets flywheel speed based on distance to target */
    public void autoPower(double distance){
        // 15.1 = one-shot sampled value, adjust as necessary
        double rps = 15.1 * distance;
        setUpperPower(rps);
    }
}