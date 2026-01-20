package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
   
    public TalonFX intakeMotor;
    public TalonFX armMotor;
    // public double rtrigger;

    public Intake() {
        intakeMotor = new TalonFX(0);
        armMotor=new TalonFX(1);
        // intakeMotor.setInverted(false);
        
    }

    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }
    public void setArmPower(double power) {
        intakeMotor.set(power);
    }

    public void takeIn(){
        intakeMotor.set(1);
    }

    public void eject(){
        intakeMotor.set(-0.5);
    }
    public void armUp(){
        armMotor.set(0.5);
    }
    public void armDown(){
        armMotor.set(-0.5);
    }

    public void IntakeStop() {
        intakeMotor.stopMotor();
    }
    public void ArmStop() {
        armMotor.stopMotor();
    }
}

