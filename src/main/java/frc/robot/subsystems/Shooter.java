package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX left= new TalonFX(Constants.Shooter.LEFT_MOTOR);
  private TalonFX right= new TalonFX(Constants.Shooter.RIGHT_MOTOR);
  public void setPower(double speed){
    left.set(speed);
    right.set(-speed);
  }
  public void stop(){
    left.stopMotor();
    right.stopMotor();
  }
//   public void setVoltage(double voltage){
//     left.setVoltage(voltage);
//     right.setVoltage(voltage);
//   }
}