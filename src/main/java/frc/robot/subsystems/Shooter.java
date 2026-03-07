package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        upperConfig.kP = 0.55;
        upperConfig.kV = 0.12;

        lowerConfig.kP = 0.35;
        lowerConfig.kV = 0.12;

        upperMotor.getConfigurator().apply(upperConfig, 0.05);
        lowerMotor.getConfigurator().apply(lowerConfig, 0.05);
        // agitatorMotor = new TalonFX(Constants.Motors.Agitator);

        SmartDashboard.putBoolean("Shooter/Flywheel", false);
        SmartDashboard.putBoolean("Shooter/Feeder", false);
        SmartDashboard.putBoolean("Shooter/AutoPower", false);
    }

    /**
     * @param power The power to set both shooter motors to. 
     *      Value should be in the range [-1, 1] where 1 is full speed forward.
     *      Invert the motors in the constructor if behavior is not as described above.
     */

   
    public void setLowerPower(double rps) {
        lowerMotor.setControl(m_velocity.withVelocity(rps));
        SmartDashboard.putBoolean("Shooter/Feeder", true);
    }

    public void stopLower() {
        lowerMotor.stopMotor();
        SmartDashboard.putBoolean("Shooter/Feeder", false);
    }

    // PID-based, better consistency
    public void setUpperPower(double rps) {
        upperMotor.setControl(m_velocity.withVelocity(rps));
        SmartDashboard.putBoolean("Shooter/Flywheel", true);
    }

    public void stopUpper() {   
        upperMotor.stopMotor();
        SmartDashboard.putBoolean("Shooter/Flywheel", false);
    }

    public void addTestSpeed(double increment) {
        testSpeed = MathUtil.clamp(testSpeed+increment, 0.0, 1.0);
        System.out.println("Test Speed: " + testSpeed);
    }

    /* Sets flywheel speed based on distance in meters to target */
    public void autoPower(double distance){
        // 15.1 = one-shot sampled value, adjust as necessary
        double distFeet = Units.metersToFeet(distance);
        double rps = getAutoPower(distFeet);
        setUpperPower(rps);
    }

    public double getAutoPower(double distance){
        // 15.1 = one-shot sampled value, adjust as necessary
        double distFeet = Units.metersToFeet(distance);
        double rps = 39.7+ 0.0417 * distFeet + 0.137 * Math.pow(distFeet,2);
        return rps; 
    }

    public double getUpperVelocity(){
        return upperMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Flywheel Velocity (rps)", upperMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Feeder Velocity (rps)", lowerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Flywheel Power", upperMotor.get());
        SmartDashboard.putNumber("Shooter/Feeder Power", lowerMotor.get());
    }
}