package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Flywheel + Lift subsystem.
 *
 * Uses standard Servo in ANGULAR mode for the lift (position control).
 * Uses precise position-based movement for consistent lifting.
 *
 * Notes:
 * - Auto-shoot uses the spindexer sequence: +60°, then +120°, then +120°.
 * - Lift uses position control for reliable movement.
 */
public class ShooterLiftSubsystem {

    private final DcMotorEx shooterMotor;
    private final Servo liftServo;

    // Lift constants
    private static final double LIFT_DOWN_POSITION = 0.0;   // Servo position when down
    private static final double LIFT_UP_POSITION = 0.65;    // Servo position when up (adjust as needed)
    private static final long LIFT_MOVE_TIME_MS = 400;      // Time for servo to complete movement

    private double initialShootAngleOffset = 50;
    private double secondaryShootAngleOffset = 5;
    private double returnToIntakeOffset = 10.0;  // Adjustable offset for return positioning

    private boolean isLiftUp = false;

    // Shooter state
    private boolean isShooterOn = false;

    // PIDF defaults
    private static final double DEFAULT_P = 10.0;
    private static final double DEFAULT_I = 0.0;
    private static final double DEFAULT_D = 0.0;
    private static final double DEFAULT_F = 20.7;

    private double p = DEFAULT_P;
    private double i = DEFAULT_I;
    private double d = DEFAULT_D;
    private double f = DEFAULT_F;

    // Target velocity
    private double targetVelocity = 2000;

    public ShooterLiftSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        liftServo = hardwareMap.get(Servo.class, "liftServo");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDF();

        // Initialize lift servo to down position
        liftServo.setPosition(LIFT_DOWN_POSITION);
    }

    // --------------------
    // Shooter controls
    // --------------------

    public void toggleShooter() {
        isShooterOn = !isShooterOn;
        if (isShooterOn)
            shooterMotor.setVelocity(targetVelocity);
        else
            shooterMotor.setVelocity(0);
    }

    public void setShooterPower(double ratio) {
        if (ratio > 0) {
            isShooterOn = true;
            shooterMotor.setVelocity(targetVelocity * ratio);
        } else {
            isShooterOn = false;
            shooterMotor.setVelocity(0);
        }
    }

    // --------------------
    // PIDF tuning
    // --------------------

    public void adjustF(double delta) {
        f += delta;
        updatePIDF();
    }

    public void adjustP(double delta) {
        p += delta;
        updatePIDF();
    }

    private void updatePIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, f);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public double getP() {
        return p;
    }

    public double getF() {
        return f;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public boolean isShooterOn() {
        return isShooterOn;
    }

    // --------------------
    // Lift controls
    // --------------------

    public void toggleLift() {
        if (isLiftUp) {
            liftServo.setPosition(LIFT_DOWN_POSITION);
            isLiftUp = false;
        } else {
            liftServo.setPosition(LIFT_UP_POSITION);
            isLiftUp = true;
        }
    }

    public void toggleLift(Runnable driveAction, boolean opModeActive) {
        if (isLiftUp) {
            liftServo.setPosition(LIFT_DOWN_POSITION);
            sleepWithDrive(LIFT_MOVE_TIME_MS, driveAction, opModeActive);
            isLiftUp = false;
        } else {
            liftServo.setPosition(LIFT_UP_POSITION);
            sleepWithDrive(LIFT_MOVE_TIME_MS, driveAction, opModeActive);
            isLiftUp = true;
        }
    }

    private void moveLiftUp(Runnable driveAction, boolean opModeActive) {
        liftServo.setPosition(LIFT_UP_POSITION);
        sleepWithDrive(LIFT_MOVE_TIME_MS, driveAction, opModeActive);
        isLiftUp = true;
    }

    private void moveLiftDown(Runnable driveAction, boolean opModeActive) {
        liftServo.setPosition(LIFT_DOWN_POSITION);
        sleepWithDrive(LIFT_MOVE_TIME_MS, driveAction, opModeActive);
        isLiftUp = false;
    }

    // --------------------
    // Calibration / getters
    // --------------------

    public void adjustLiftUpPosition(double delta) {
        double newPosition = LIFT_UP_POSITION + delta;
        // Clamp to valid servo range
        if (newPosition >= 0.0 && newPosition <= 1.0) {
            liftServo.setPosition(newPosition);
        }
    }

    public double getLiftUpPosition() {
        return LIFT_UP_POSITION;
    }

    public double getCurrentLiftPosition() {
        return liftServo.getPosition();
    }

    public boolean isLiftUp() {
        return isLiftUp;
    }

    public void adjustInitialShootAngleOffset(double delta) {
        initialShootAngleOffset += delta;
    }

    public double getInitialShootAngleOffset() {
        return initialShootAngleOffset;
    }

    public void adjustReturnToIntakeOffset(double delta) {
        returnToIntakeOffset += delta;
    }

    public double getReturnToIntakeOffset() {
        return returnToIntakeOffset;
    }

    // --------------------
    // Macro: Auto shoot
    // --------------------

    public void shootMacro(SpindexerSubsystem spindexer, Runnable driveAction, boolean opModeActive) {
        if (!opModeActive)
            return;

        // How many balls to shoot (0..3). If you want always 3, change this to '3'.
        int shots = Math.min(3, spindexer.getBallsLoaded());
        if (shots <= 0)
            return;

        // Track total spindexer rotation for return to intake
        double totalSpindexerRotation = 60.0 + initialShootAngleOffset;

        // 1) Pre-align: +60 degrees (half of 120) PLUS OFFSET, using the SAME auto-turn
        // logic as auto-intake
        // initialShootAngleOffset to contour for servo spedness
        spindexer.moveSpindexerAutoDegrees(60.0 + initialShootAngleOffset, driveAction, opModeActive);

        // 2) Spin up
        setShooterPower(1.0);

        // 3) Shoot sequence; after shot 1 and 2, index +120 using auto timing
        for (int iShot = 0; iShot < shots && opModeActive; iShot++) {

            // Wait for flywheel to recover before each shot
            sleepWithDrive(350, driveAction, opModeActive);

            // Lift up to fire (position controlled servo)
            moveLiftUp(driveAction, opModeActive);

            sleepWithDrive(150, driveAction, opModeActive);

            // Lift down
            moveLiftDown(driveAction, opModeActive);

            // Index to next ball if there is one remaining:
            // shot 0 -> shot 1: +120 + full offset (works great)
            // shot 1 -> shot 2: +120 + reduced offset (3rd shot was rotating too much)
            if (iShot < shots - 1) {
                // First index uses full offset, second index uses half offset
                double indexOffset = (iShot == 0) ? secondaryShootAngleOffset : (secondaryShootAngleOffset / 2.0);
                spindexer.moveSpindexerAutoDegrees(120.0 + indexOffset, driveAction, opModeActive);
                totalSpindexerRotation += 120.0 + indexOffset;
            }
        }

        // 4) Stop shooter
        setShooterPower(0.0);
        isLiftUp = false;

        // 5) Return spindexer to intake position by reversing the total rotation
        // Use adjustable offset (you can tune this with adjustReturnToIntakeOffset method)
        spindexer.moveSpindexerAutoDegrees(-(totalSpindexerRotation + returnToIntakeOffset), driveAction, opModeActive);
    }

    // Helper method for timing with drive action
    private void sleepWithDrive(long ms, Runnable driveAction, boolean opModeActive) {
        com.qualcomm.robotcore.util.ElapsedTime t = new com.qualcomm.robotcore.util.ElapsedTime();
        while (opModeActive && t.milliseconds() < ms) {
            driveAction.run();
        }
    }
}