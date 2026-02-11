package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
   
    public TalonFX intakeMotor;
    public TalonFX armMotor;
    // public double rtrigger;

    public Intake() {
        intakeMotor = new TalonFX(40);
        armMotor=new TalonFX(41);
        // intakeMotor.setInverted(false);
        
    }

    public void rollerIn(double speed){
        intakeMotor.set(speed);
    }

    public void rollerOut(double speed){
        intakeMotor.set(-speed);
    }
    public void armUp(double speed){
        armMotor.set(speed);
    }
    public void armDown(double speed){
        armMotor.set(-speed);
    }

    public void intakeStop() {
        intakeMotor.stopMotor();
    }
    public void armStop() {
        armMotor.stopMotor();
    }
}