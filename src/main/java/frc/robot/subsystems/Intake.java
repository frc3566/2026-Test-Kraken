package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public TalonFX rollerMotor;
    public TalonFX armMotor;

    private final VelocityVoltage m_velocity = new VelocityVoltage(0);

    // Reuse a single request object — just update the position each call
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0)
        .withSlot(0);

    /**
     * Gear ratio: rotor rotations per one full rotation of the arm mechanism.
     */
    private static final double ARM_GEAR_RATIO = Constants.Arm.GearRatio;

    public Intake() {
        rollerMotor = new TalonFX(Constants.Motors.IntakeRoller);
        armMotor = new TalonFX(Constants.Motors.IntakeArm);

        // ------------------------------------------------------------------
        // Arm configuration
        // ------------------------------------------------------------------
        var armConfig = new TalonFXConfiguration();

        // Slot 0 — arm position PID
        // TODO: Tune on the real robot.
        var armSlot0 = new Slot0Configs();
        armSlot0.kP = 0.6;
        armSlot0.kI = 0.0;
        armSlot0.kD = 0.0;
        armSlot0.kV = 0.12;
        armSlot0.kG = 0.0;
        armConfig.Slot0 = armSlot0;

        // Sensor-to-mechanism ratio so getArmPosition() reports mechanism rotations
        armConfig.Feedback.SensorToMechanismRatio = ARM_GEAR_RATIO;

        // Software limits — prevent over-extension / over-retraction.
        // Remember to power the robot on while the arm is up,
        // or the arm encoders will not work as intended.
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;  // Straight up
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.23; // All the way down

        armConfig.Feedback.RotorToSensorRatio = 1;
        armConfig.Feedback.SensorToMechanismRatio = 64;
        armMotor.getConfigurator().apply(armConfig);

        // ------------------------------------------------------------------
        // Roller configuration
        // ------------------------------------------------------------------
        var rollerConfig = new Slot0Configs();
        rollerConfig.kP = 0.1;
        rollerConfig.kV = 0.1;
        rollerMotor.getConfigurator().apply(rollerConfig);

        SmartDashboard.putBoolean("Intake/Roller In", false);
        SmartDashboard.putBoolean("Intake/Roller Out", false);
        SmartDashboard.putBoolean("Intake/Arm Moving Up", false);
        SmartDashboard.putBoolean("Intake/Arm Moving Down", false);
    }

    // ------------------------------------------------------------------
    // Arm closed-loop position control
    // ------------------------------------------------------------------

    /**
     * Commands the arm to a target position using PositionDutyCycle (Slot 0).
     * Automatically updates dashboard indicators based on movement direction.
     *
     * @param rotations Target position in mechanism rotations.
     */
    public void setArmPosition(double rotations) {
        boolean movingUp = rotations > getArmPosition();
        armMotor.setControl(positionRequest.withPosition(rotations));
        SmartDashboard.putBoolean("Intake/Arm Moving Up", movingUp);
        SmartDashboard.putBoolean("Intake/Arm Moving Down", !movingUp);
    }

    /**
     * Returns the current arm mechanism position in rotations.
     */
    public double getArmPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    /**
     * Returns true if the arm is within {@code tolerance} rotations of {@code target}.
     */
    public boolean armAtSetpoint(double target, double tolerance) {
        return Math.abs(armMotor.getPosition().getValueAsDouble() - target) <= tolerance;
    }

    /**
     * Stops the arm motor immediately (coast/brake depending on NeutralMode config).
     */
    public void stopArm() {
        armMotor.stopMotor();
        SmartDashboard.putBoolean("Intake/Arm Moving Up", false);
        SmartDashboard.putBoolean("Intake/Arm Moving Down", false);
    }

    /**
     * Manual open-loop arm control — moves arm upward.
     *
     * @param percent [0, 1]
     */
    public void armUp(double percent) {
        armMotor.set(percent);
        SmartDashboard.putBoolean("Intake/Arm Moving Up", true);
        SmartDashboard.putBoolean("Intake/Arm Moving Down", false);
    }

    /**
     * Manual open-loop arm control — moves arm downward.
     *
     * @param percent [0, 1]
     */
    public void armDown(double percent) {
        armMotor.set(-percent);
        SmartDashboard.putBoolean("Intake/Arm Moving Up", false);
        SmartDashboard.putBoolean("Intake/Arm Moving Down", true);
    }

    /**
     * Resets the arm encoder to a known position.
     *
     * @param isStraight true = arm is straight up (0 rot), false = arm is all the way down (-0.25 rot)
     */
    public void resetArmPosition(boolean isStraight) {
        armMotor.setPosition(isStraight ? 0 : -0.25);
    }

    // ------------------------------------------------------------------
    // Roller control
    // ------------------------------------------------------------------

    public void rollerIn(double rps) {
        rollerMotor.setControl(m_velocity.withVelocity(rps));
        SmartDashboard.putBoolean("Intake/Roller In", true);
        SmartDashboard.putBoolean("Intake/Roller Out", false);
    }

    public void rollerOut(double rps) {
        rollerMotor.setControl(m_velocity.withVelocity(-rps));
        SmartDashboard.putBoolean("Intake/Roller In", false);
        SmartDashboard.putBoolean("Intake/Roller Out", true);
    }

    public void stopRoller() {
        rollerMotor.stopMotor();
        SmartDashboard.putBoolean("Intake/Roller In", false);
        SmartDashboard.putBoolean("Intake/Roller Out", false);
    }

    // ------------------------------------------------------------------
    // Periodic
    // ------------------------------------------------------------------

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Arm Position (rot)", armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Velocity (rps)", rollerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Arm Supply Current (A)", armMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Supply Current (A)", rollerMotor.getSupplyCurrent().getValueAsDouble());
    }
}