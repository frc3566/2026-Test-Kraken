package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public TalonFX lowerMotor, upperMotor, agitatorMotor;
    public double testSpeed = 0;
    private VelocityVoltage m_velocity = new VelocityVoltage(0);
    private Slot0Configs upperConfig = new Slot0Configs();

    public Shooter() {
        lowerMotor = new TalonFX(Constants.Motors.ShooterLow);
        upperMotor = new TalonFX(Constants.Motors.ShooterHigh);
        upperConfig.kP = 0.1;
        upperConfig.kD = 0.05;
        upperMotor.getConfigurator().apply(upperConfig, 0.05);
        // agitatorMotor = new TalonFX(Constants.Motors.Agitator);
    }

    /**
     * @param power The power to set both shooter motors to. 
     *      Value should be in the range [-1, 1] where 1 is full speed forward.
     *      Invert the motors in the constructor if behavior is not as described above.
     */

   
    public void setLowerPower(double power) {
        lowerMotor.set(power);
        // agitatorMotor.set((power/8));
    }

    public void stopLower() {
        lowerMotor.stopMotor();
        // agitatorMotor.stopMotor();
    }

    public void setUpperPower(double power) {
        upperMotor.set(power);
    }

    public void addTestSpeed(double increment) {
        testSpeed = MathUtil.clamp(testSpeed+increment, 0.0, 1.0);
        System.out.println("Test Speed: " + testSpeed);
    }

    public void stopUpper() {   
        upperMotor.stopMotor();
    }

    // public void setAgitatorPower(double speed){
    //     agitatorMotor.set(speed);
    // }

    // public void stopAgitator(){
    //     agitatorMotor.stopMotor();
    // }

    // Used with /Vision/AutoShoot
    public void autoPower(double distance){

        // Percentage Power Calculation
        double percentPower = 31.1 + ((3.3)*(Units.metersToFeet(distance))) + (-0.0714*Math.pow(Units.metersToFeet(distance), 2));
        
        System.out.println("Auto Power: " + percentPower);
        upperMotor.set(percentPower/100);
    }

    public double getUpperDutyCyle(){
        return upperMotor.getDutyCycle().getValueAsDouble();
    }

    public double getUpperUpperVelocity(){
        return upperMotor.getVelocity().getValueAsDouble();
    }

    public void testPID(){
        upperMotor.setControl(m_velocity.withVelocity(10));
    }

    public void testOpenLoop(){
        upperMotor.set(0.1);
    }
    
}