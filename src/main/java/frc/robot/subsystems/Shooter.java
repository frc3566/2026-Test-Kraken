package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public TalonFX lowerMotor, upperMotor, agitatorMotor;
    public double testSpeed = 0;

    public Shooter() {
        lowerMotor = new TalonFX(Constants.Motors.ShooterLow);
        upperMotor = new TalonFX(Constants.Motors.ShooterHigh);
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
    
}