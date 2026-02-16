package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
   
    public TalonFX rollerMotor;
    public TalonFX armMotor;
    private double armGearRatio = 4;
    // public double rtrigger;

    public Intake() {
        rollerMotor = new TalonFX(Constants.Motors.IntakeRoller);
        armMotor=new TalonFX(Constants.Motors.IntakeArm);
        // intakeMotor.setInverted(false);

        var armConfig = new TalonFXConfiguration();

        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -1;

        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -15;
        armMotor.getConfigurator().apply(armConfig);
        
    }

    // MOTOR IS INVERSED; ADD NEGATIVE
    public void rollerIn(double speed){
        rollerMotor.set(speed);
    }

    public void rollerOut(double speed){
        rollerMotor.set(-speed);
    }
    public void armUp(double speed){
        armMotor.set(speed);
    }
    public void armDown(double speed){
        armMotor.set(-speed);
    }

    public void rollerStop() {
        rollerMotor.stopMotor();
    }
    public void armStop() {
        armMotor.stopMotor();
    }

    public double getArmRotation() {
        double rotation = armMotor.getPosition().getValueAsDouble();
        System.out.println(rotation);
        return rotation;
    }
}