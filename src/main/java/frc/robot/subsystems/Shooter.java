package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public TalonFX ShooterLeft, ShooterRight, IndexerLeft, IndexerRight;
    public double testSpeed = 0;
    private VelocityVoltage m_velocity = new VelocityVoltage(0);


    public Shooter() {
        ShooterLeft = new TalonFX(Constants.Motors.ShooterLeft);
        ShooterRight = new TalonFX(Constants.Motors.ShooterRight);
        IndexerLeft = new TalonFX(Constants.Motors.IndexerLeft);
        IndexerRight = new TalonFX(Constants.Motors.IndexerRight);

        var upperConfig = new Slot0Configs();
        var lowerConfig = new Slot0Configs();

        //TODO: Tune these values
        upperConfig.kP = 0.55;
        upperConfig.kV = 0.12;

        lowerConfig.kP = 0.35;
        lowerConfig.kV = 0.12;

        ShooterLeft.getConfigurator().apply(upperConfig, 0.05);
        ShooterRight.getConfigurator().apply(upperConfig, 0.05);
        IndexerLeft.getConfigurator().apply(lowerConfig, 0.05);
        IndexerRight.getConfigurator().apply(lowerConfig, 0.05);
        // agitatorMotor = new TalonFX(Constants.Motors.Agitator);

        ShooterLeft.setControl(new Follower(ShooterRight.getDeviceID(), MotorAlignmentValue.Aligned));
        IndexerLeft.setControl(new Follower(IndexerRight.getDeviceID(), MotorAlignmentValue.Aligned));
        
        SmartDashboard.putBoolean("Shooter/Flywheel", false);
        SmartDashboard.putBoolean("Shooter/Feeder", false);
        SmartDashboard.putBoolean("AutoPower/Enabled", false);
        SmartDashboard.putNumber("AutoPower/Distance to Target (m)", 0);
        SmartDashboard.putNumber("AutoPower/Target Flywheel Speed (RPS)", 0);
    }

    /**
     * @param power The power to set both shooter motors to. 
     *      Value should be in the range [-1, 1] where 1 is full speed forward.
     *      Invert the motors in the constructor if behavior is not as described above.
     */

   
    public void setLowerPower(double rps) {
        IndexerRight.set(rps/100);
        SmartDashboard.putBoolean("Shooter/Feeder", true);
    }

    public void stopLower() {
        IndexerRight.stopMotor();
        SmartDashboard.putBoolean("Shooter/Feeder", false);
    }

    // PID-based, better consistency
    public void setUpperPower(double rps) {
        // ShooterLeft.setControl(m_velocity.withVelocity(rps));
        // ShooterRight.setControl(m_velocity.withVelocity(rps));
        ShooterRight.set(rps/100);
        SmartDashboard.putBoolean("Shooter/Flywheel", true);
    }

    public void stopUpper() {   
        ShooterRight.stopMotor();
        SmartDashboard.putBoolean("Shooter/Flywheel", false);
    }

    public void addTestSpeed(double increment) {
        testSpeed = MathUtil.clamp(testSpeed+increment, 0.0, 1.0);
        System.out.println("Test Speed: " + testSpeed);
    }

    /* Sets flywheel speed based on distance in meters to target */
    public void autoPower(double distance){
        // 15.1 = one-shot sampled value, adjust as necessary
        // double distFeet = Units.metersToFeet(distance);
        double rps = getAutoPower(distance);
        SignalLogger.writeDouble("Shooter/AutoPower/DistanceMeters", distance, "m");
        SignalLogger.writeDouble("Shooter/AutoPower/TargetRPS", rps, "rps");
        setUpperPower(rps);
    }

    public double getAutoPower(double distance){
        // 15.1 = one-shot sampled value, adjust as necessary
        // double distFeet = Units.metersToFeet(distance);
        // double rps = 39.7+ 0.0417 * distFeet + 0.137 * Math.pow(distFeet,2)-4;
        // TODO: Redo the equation
        double rps = 36.5 - 0.594*distance + 1.45 * Math.pow(distance, 2);
        return Math.min(100,rps); 
    }

    public double getUpperVelocity(){
        return ShooterLeft.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Flywheel Velocity (rps)", ShooterLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Feeder Velocity (rps)", IndexerLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Flywheel Power", ShooterLeft.get());
        SmartDashboard.putNumber("Shooter/Feeder Power", IndexerLeft.get());
        SmartDashboard.putNumber("Shooter/Flywheel Supply Current (A)", ShooterLeft.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Feeder Supply Current (A)", IndexerLeft.getSupplyCurrent().getValueAsDouble());
        SignalLogger.writeDouble("Shooter/UpperVelocityRPS", ShooterLeft.getVelocity().getValueAsDouble(), "rps");
    }
}