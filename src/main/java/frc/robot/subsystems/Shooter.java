package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public TalonFX lower, upper;
    public Shooter() {
        lower = new TalonFX(22);
        upper = new TalonFX(32);

    }

    /**
     * @param power The power to set both shooter motors to. 
     *      Value should be in the range [-1, 1] where 1 is full speed forward.
     *      Invert the motors in the constructor if behavior is not as described above.
     */

    // BOTH MOTORS NEED TO BE INVERTED; USE NEGATIVE POWER FOR NOW.
    public void setLowerPower(double power) {
        lower.set(-power);
    }

    public void stopLower() {
        lower.stopMotor();
    }

    public void setUpperPower(double power) {
        upper.set(-power);
    }

    public void stopUpper() {   
        upper.stopMotor();
    }
    
}