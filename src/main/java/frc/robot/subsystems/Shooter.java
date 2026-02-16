package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public TalonFX lowerMotor, upperMotor, agitatorMotor;
    public Shooter() {
        lowerMotor = new TalonFX(Constants.Motors.ShooterLow);
        upperMotor = new TalonFX(Constants.Motors.ShooterHigh);
        agitatorMotor = new TalonFX(Constants.Motors.Agitator);

    }

    /**
     * @param power The power to set both shooter motors to. 
     *      Value should be in the range [-1, 1] where 1 is full speed forward.
     *      Invert the motors in the constructor if behavior is not as described above.
     */

    // BOTH MOTORS NEED TO BE INVERTED; USE NEGATIVE POWER FOR NOW.
    public void setLowerPower(double power) {
        lowerMotor.set(power);
        agitatorMotor.set((power/8));
    }

    public void stopLower() {
        lowerMotor.stopMotor();
        agitatorMotor.stopMotor();
    }

    public void setUpperPower(double power) {
        upperMotor.set(power);
    }

    public void stopUpper() {   
        upperMotor.stopMotor();
    }
    
}