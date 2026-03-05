package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public final TalonFX climberMotor;

    // Reuse a single request object — just update the position each call
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0)
        .withSlot(0);

    /**
     * Gear ratio: rotor rotations per one full rotation of the output shaft (mechanism).
     * TODO: Set this to your actual climber gear ratio.
     */
    private static final double GEAR_RATIO = 1.0;

    public Climber() {
        climberMotor = new TalonFX(Constants.Motors.Climber);

        var config = new TalonFXConfiguration();

        // ------------------------------------------------------------------
        // Slot 0 — unloaded ascent (just the hook, no robot weight)
        // TODO: Tune on the real robot.
        // ------------------------------------------------------------------
        var slot0 = new Slot0Configs();
        slot0.kP = 0.6;   // Proportional
        slot0.kI = 0.0;
        slot0.kD = 0;    // Derivative — damp oscillation
        slot0.kV = 0.12;   // Velocity feedforward (V·s/rot)
        slot0.kG = 0.0;    // No gravity comp needed going up unloaded
        config.Slot0 = slot0;

        // ------------------------------------------------------------------
        // Slot 1 — loaded descent (full robot weight hanging on the climber)
        // Higher kG holds the robot against gravity. Lower cruise velocity so
        // the descent is slow and controlled rather than a free-fall.
        // TODO: Tune on the real robot — kG especially depends on robot weight.
        // ------------------------------------------------------------------
        var slot1 = new Slot1Configs();
        slot1.kP = 0.8;   // Higher P to fight the load
        slot1.kI = 0.0;
        slot1.kD = 0;
        slot1.kV = 0.12;
        slot1.kG = 0.0;    // Feedforward to hold robot weight — tune this up if it sags
        config.Slot1 = slot1;

        // ------------------------------------------------------------------
        // Current limits — critical when the motor fights the robot's full weight.
        // Without this, the TalonFX will spike current and may overheat.
        // TODO: Adjust StatorCurrentLimit based on your motor and load testing.
        // ------------------------------------------------------------------

        // Sensor-to-mechanism ratio so getPosition() reports mechanism rotations
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        // ------------------------------------------------------------------
        // Software limits — prevent over-extension / over-retraction.
        // TODO: Set real limits once you know the travel of your climber.
        // ------------------------------------------------------------------
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0; // TODO: Tune
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -180.0;

        climberMotor.getConfigurator().apply(config);

        // Zero the encoder on startup (assumes climber starts fully retracted)
        climberMotor.setPosition(0);
    }

    /**
     * Commands the climber to a target position
     * Automatically selects the gain slot based on direction:
     *   - Slot 0 (unloaded) when moving up (target > current)
     *   - Slot 1 (loaded)   when moving down (target < current) — robot is hanging
     *
     * @param rotations Target position in mechanism rotations.
     */
    public void setPosition(double rotations) {
        boolean movingDown = rotations < getPosition();
        int slot = movingDown ? 1 : 0;
        climberMotor.setControl(positionRequest.withPosition(rotations).withSlot(slot));
    }

    /**
     * Commands the climber to a target position with an explicit gain slot.
     * Use this if you need to override the automatic slot selection.
     *
     * @param rotations Target position in mechanism rotations.
     * @param slot      0 = unloaded ascent, 1 = loaded descent.
     */
    public void setPosition(double rotations, int slot) {
        climberMotor.setControl(positionRequest.withPosition(rotations).withSlot(slot));
    }

    /**
     * Returns the current mechanism position in rotations.
     */
    public double getPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    /**
     * Stops the climber motor immediately (coast/brake depending on NeutralMode config).
     */
    public void stop() {
        climberMotor.stopMotor();
    }

    /**
     * Manual open-loop control, e.g. for driver override.
     *
     * @param power [-1, 1]
     */
    public void set(double power) {
        climberMotor.set(power);
    }
}