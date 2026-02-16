package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    public TalonFX leftMotor, rightMotor;
    public Climber() {
        // leftMotor = new TalonFX(Constants.Motors.ClimberLow);
        // rightMotor = new TalonFX(Constants.Motors.ClimberHigh);

    }

    /**
     * @param power The power to set both shooter motors to. 
     *      Value should be in the range [-1, 1] where 1 is full speed forward.
     *      Invert the motors in the constructor if behavior is not as described above.
     */

   
    public void set(double power) {
        leftMotor.set(power);
        rightMotor.set((power));
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
    
}