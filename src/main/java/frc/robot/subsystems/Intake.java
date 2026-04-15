package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public TalonFX rollerMotor, armLeaderMotor;
    public CANcoder ArmCanCoder;

    private final MotionMagicVelocityVoltage m_velocity = new MotionMagicVelocityVoltage(0)
        .withAcceleration(100);

    // Reuse a single request object — just update the position each call
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0)
        .withSlot(0);

    // Separate Motion Magic limits for up vs down
    private static final double MM_UP_CRUISE_VEL = 1.5; // rotations/sec
    private static final double MM_UP_ACCEL = 1.5;       // rotations/sec^2
    private static final double MM_DOWN_CRUISE_VEL = 1.5;
    private static final double MM_DOWN_ACCEL = 1.5;

    private final MotionMagicConfigs mmUp = new MotionMagicConfigs();
    private final MotionMagicConfigs mmDown = new MotionMagicConfigs();
    private Boolean lastAppliedMovingUp = null;

    /**
     * Gear ratio: rotor rotations per one full rotation of the arm mechanism.
     */
    private static final double ARM_GEAR_RATIO = Constants.Arm.GearRatio;

    public Intake() {
        rollerMotor = new TalonFX(Constants.Motors.IntakeRoller);
        ArmCanCoder = new CANcoder(Constants.Motors.IntakeArmCanCoder);
        armLeaderMotor = new TalonFX(Constants.Motors.IntakeArmLeader);
        // armFollowerMotor = new TalonFX(Constants.Motors.IntakeArmFollower);

        // ------------------------------------------------------------------
        // Arm configuration
        // ------------------------------------------------------------------
        var armConfig = new TalonFXConfiguration();

    // Use remote CANcoder as feedback source for arm position control.
    armConfig.Feedback.FeedbackRemoteSensorID = Constants.Motors.IntakeArmCanCoder;
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // CANcoder is mounted on mechanism, so sensor->mechanism is 1:1.
    armConfig.Feedback.SensorToMechanismRatio = 1.0;
    // Rotor turns ARM_GEAR_RATIO times for one sensor turn.
    armConfig.Feedback.RotorToSensorRatio = ARM_GEAR_RATIO;

        // Software limits — prevent over-extension / over-retraction.
        // Remember to power the robot on while the arm is up,
        // or the arm encoders will not work as intended.
        // armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.ForwardSoftLimitRotations;  // Straight up
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.ReverseSoftLimitRotations; // All the way down
        armConfig.Slot0.kP = 0.5;
        armConfig.Slot0.kI = 0.0;
        armConfig.Slot0.kD = 0.0;
        armConfig.Slot0.kV = 0.12;

        // Slot 1 — slower PID for raising the arm to the top (0.0 rotations)
        var armSlot1 = new Slot1Configs();
        armSlot1.kP = armConfig.Slot0.kP;
        armSlot1.kI = armConfig.Slot0.kI;
        armSlot1.kD = armConfig.Slot0.kD;
        armSlot1.kV = armConfig.Slot0.kV; // keep feedforward the same
        armConfig.Slot1 = armSlot1;

        // Motion Magic defaults (overridden per-move in setArmPosition)
        // Units are mechanism rotations per second (because of SensorToMechanismRatio above)
        armConfig.MotionMagic.MotionMagicCruiseVelocity = MM_DOWN_CRUISE_VEL;
        armConfig.MotionMagic.MotionMagicAcceleration = MM_DOWN_ACCEL;
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Pre-build directional Motion Magic configs so we can swap quickly without touching other settings
        mmUp.MotionMagicCruiseVelocity = MM_UP_CRUISE_VEL;
        mmUp.MotionMagicAcceleration = MM_UP_ACCEL;

        mmDown.MotionMagicCruiseVelocity = MM_DOWN_CRUISE_VEL;
        mmDown.MotionMagicAcceleration = MM_DOWN_ACCEL;

        // armLeaderMotor.getConfigurator().apply(armConfig);
        // armFollowerMotor.getConfigurator().apply(armConfig);

        // armFollowerMotor.setControl(new Follower(armLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));


        // ------------------------------------------------------------------
        // Roller configuration
        // ------------------------------------------------------------------
        var rollerConfig = new Slot0Configs();
        rollerConfig.kP = 0.1;
        rollerConfig.kV = 0.1;
        // rollerMotor.getConfigurator().apply(rollerConfig);

        SmartDashboard.putBoolean("Intake/Roller In", false);
        SmartDashboard.putBoolean("Intake/Roller Out", false);
        SmartDashboard.putBoolean("Intake/Arm Moving Up", false);
        SmartDashboard.putBoolean("Intake/Arm Moving Down", false);
    }

    // ------------------------------------------------------------------
    // Arm closed-loop position control
    // ------------------------------------------------------------------

    /**
     * Commands the arm to a target position using Motion Magic (Slot 0/1).
     * Automatically updates dashboard indicators based on movement direction.
     *
     * @param rotations Target position in mechanism rotations.
     */
    public void setArmPosition(double rotations) {
        boolean movingUp = rotations > getArmPosition();
        int slotToUse = movingUp ? 0 : 1;
        SmartDashboard.putString("Intake/Arm MM Slot", movingUp ? "Up (Slot 0)" : "Down (Slot 1)");
   

        // Pick Motion Magic constraints based on direction
        // Only reapply configs when direction changes to avoid CAN spam
        // if (lastAppliedMovingUp == null || lastAppliedMovingUp.booleanValue() != movingUp) {
        //     armLeaderMotor.getConfigurator().apply(movingUp ? mmUp : mmDown);
        //     lastAppliedMovingUp = movingUp;
        // }

        // Use Motion Magic for built-in velocity/accel limiting,
        // and choose slot based on movement direction.
        armLeaderMotor.setControl(
            motionMagicRequest
                .withSlot(slotToUse)
                .withPosition(rotations)
        );
        // armFollowerMotor.setControl(new Follower(armLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        SmartDashboard.putBoolean("Intake/Arm Moving Down", !movingUp);
        SmartDashboard.putBoolean("Intake/Arm Moving Up", movingUp);
    }

    /**
     * Returns the current arm mechanism position in rotations.
     */
    public double getArmPosition() {
        return armLeaderMotor.getPosition().getValueAsDouble();
    }

    /**
     * Returns true if the arm is within {@code tolerance} rotations of {@code target}.
     */
    public boolean armAtSetpoint(double target, double tolerance) {
        return Math.abs(armLeaderMotor.getPosition().getValueAsDouble() - target) <= tolerance;
    }

    /**
     * Stops the arm motor immediately (coast/brake depending on NeutralMode config).
     */
    public void stopArm() {
        armLeaderMotor.stopMotor();
        // armFollowerMotor.stopMotor();
        SmartDashboard.putBoolean("Intake/Arm Moving Up", false);
        SmartDashboard.putBoolean("Intake/Arm Moving Down", false);
    }

    /**
     * Manual open-loop arm control — moves arm upward.
     *
     * @param percent [0, 1]
     */
    public void armUp(double percent) {
        setArmPower(percent);
    }

    /**
     * Manual open-loop arm control — moves arm downward.
     *
     * @param percent [0, 1]
     */
    public void armDown(double percent) {
        setArmPower(-percent);
    }

    /**
     * Manual open-loop arm control — sets power directly on the leader motor.
     * Positive values raise the arm; negative values lower it.
     *
     * Positive = Lower the arm, Negative = Raise the arm
     * @param power [-1, 1]
     */
    public void setArmPower(double power) {
        armLeaderMotor.setControl(new DutyCycleOut(power));
        // armFollowerMotor.setControl(new Follower(armLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        SmartDashboard.putBoolean("Intake/Arm Moving Up", power > 0);
        SmartDashboard.putBoolean("Intake/Arm Moving Down", power < 0);
    }

    /**
     * Directly sets the arm encoder position (mechanism rotations).
     * Useful for zeroing or seeding during pre-match setup.
     */
    public void setArmEncoderPosition(double rotations) {
        armLeaderMotor.setPosition(rotations);
    }

    // ------------------------------------------------------------------
    // Roller control
    // ------------------------------------------------------------------

    public void rollerIn(double rps) {
        rollerMotor.setControl(m_velocity.withVelocity(rps).withAcceleration(100));
        SmartDashboard.putBoolean("Intake/Roller In", true);
        SmartDashboard.putBoolean("Intake/Roller Out", false);
    }

    public void rollerOut(double rps) {
        rollerMotor.setControl(m_velocity.withVelocity(-rps).withAcceleration(100));
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
        SmartDashboard.putNumber("Intake/Arm Position (rot)", armLeaderMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Arm CANcoder Position (rot)", ArmCanCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Arm CANcoder Absolute Position (rot)", ArmCanCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Arm CANcoder Velocity (rps)", ArmCanCoder.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Arm Leader Velocity (rps)", armLeaderMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Power", rollerMotor.get());
        SmartDashboard.putNumber("Intake/Roller Velocity (rps)", rollerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Arm Power", armLeaderMotor.get());
        SmartDashboard.putNumber("Intake/Arm Leader Supply Current (A)", armLeaderMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Intake/Arm Follower Supply Current (A)", armFollowerMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Intake/Arm Follower Velocity (rps)", armFollowerMotor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Intake/Arm Follower Position (rot)", armFollowerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Supply Current (A)", rollerMotor.getSupplyCurrent().getValueAsDouble());
    }
}