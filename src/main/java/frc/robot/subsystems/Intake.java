package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
   
    public TalonFX rollerMotor;
    public TalonFX armMotor;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    

    public Intake() {
        rollerMotor = new TalonFX(Constants.Motors.IntakeRoller);
        armMotor=new TalonFX(Constants.Motors.IntakeArm);

        // Need to set soft limits, so use talonfx config
        var armConfig = new TalonFXConfiguration();
        var intakeConfig = new Slot0Configs();


        // Remember to power the robot on while the arm is up,
        // or the arm encoders will not work as intended and can't move up
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0; // Straight up

        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.23; // All the way down

        armConfig.Feedback.RotorToSensorRatio = 1;
        armConfig.Feedback.SensorToMechanismRatio = 64;
        armMotor.getConfigurator().apply(armConfig);

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

    public void stopRoller() {
        rollerMotor.stopMotor();
    }
    public void stopArm() {
        armMotor.stopMotor();
    }

    public double getArmPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public void setArmPosition(double rotations) {
        armMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    public double getPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public boolean armAtSetpoint(double target, double tolerance) {
        return Math.abs(armMotor.getPosition().getValueAsDouble() - target) <= tolerance;
    }

}