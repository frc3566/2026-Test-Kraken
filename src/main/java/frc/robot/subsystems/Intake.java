package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
   
    public TalonFX rollerMotor;
    public TalonFX armMotor;
    private VelocityVoltage m_velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    

    public Intake() {
        rollerMotor = new TalonFX(Constants.Motors.IntakeRoller);
        armMotor=new TalonFX(Constants.Motors.IntakeArm);

        // Need to set soft limits, so use talonfx config
        var armConfig = new TalonFXConfiguration();
        var rollerConfig = new Slot0Configs();
        rollerConfig.kP = 0.1;
        rollerConfig.kV = 0.1;


        // Remember to power the robot on while the arm is up,
        // or the arm encoders will not work as intended and can't move up
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0; // Straight up

        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.23; // All the way down

        armConfig.Feedback.RotorToSensorRatio = 1;
        armConfig.Feedback.SensorToMechanismRatio = 64;
        armMotor.getConfigurator().apply(armConfig);
        rollerMotor.getConfigurator().apply(rollerConfig);

        // // PID gains (tune later)
        // armConfig.Slot0.kP = 35.0;
        // armConfig.Slot0.kI = 0.0;
        // armConfig.Slot0.kD = 0.2;

        // // Gravity feedforward (START small, tune upward)
        // armConfig.Slot0.kG = 1.5;

        // armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // armConfig.Slot0.GravityArmPositionOffset = 0.25;

        // // Motion Magic settings
        // armConfig.MotionMagic.MotionMagicCruiseVelocity = 360;
        // armConfig.MotionMagic.MotionMagicAcceleration = 120;


        // // Apply configuration
        // armMotor.getConfigurator().apply(armConfig);
        
    }

    public void rollerIn(double rps){
        rollerMotor.setControl(m_velocity.withVelocity(rps));
    }

    public void rollerOut(double rps){
        rollerMotor.setControl(m_velocity.withVelocity(-rps));
    }
    public void armUp(double percent){
        armMotor.set(percent);
    }
    public void armDown(double percent){
        armMotor.set(-percent);
    }

    public void stopRoller() {
        rollerMotor.stopMotor();
    }
    public void stopArm() {
        armMotor.stopMotor();
    }

    public double getArmPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public void resetArmPosition(boolean isStraight){
        if(isStraight){
            armMotor.setPosition(0);
        } else{
            armMotor.setPosition(-0.25);
        }
    }


    public double getPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public boolean armAtSetpoint(double target, double tolerance) {
        return Math.abs(armMotor.getPosition().getValueAsDouble() - target) <= tolerance;
    }

}