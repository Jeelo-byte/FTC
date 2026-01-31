package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Helper class for precise continuous rotation servo movements.
 *
 * Handles:
 * - Timed degree-based movements with configurable ms/degree ratio
 * - Blocking and non-blocking movement modes
 * - Optional multipliers for auto-tuning (e.g., compensating for under-rotation)
 * - Power ramping to reduce overshoot
 * - Integration with drive actions during movements
 */
public class ServoMovementHelper {

    private final CRServo servo;
    private double msPerDegree;
    private double currentDegrees;

    // Movement state for non-blocking operations
    private boolean isMoving = false;
    private final ElapsedTime moveTimer = new ElapsedTime();
    private long targetDuration = 0;
    private double targetPower = 0.0;

    // Power ramping to reduce overshoot
    private static final double RAMP_UP_PERCENT = 0.15;    // Ramp up for first 15% of movement
    private static final double RAMP_DOWN_PERCENT = 0.15;  // Ramp down for last 15% of movement
    private static final long MIN_STOP_TIME_MS = 50;       // Minimum time to apply brake

    /**
     * Create a new servo movement helper.
     *
     * @param servo The continuous rotation servo to control
     * @param msPerDegree Milliseconds per degree of rotation at full power
     */
    public ServoMovementHelper(CRServo servo, double msPerDegree) {
        this.servo = servo;
        this.msPerDegree = msPerDegree;
        this.currentDegrees = 0.0;
        servo.setPower(0);
    }

    /**
     * Move the servo a specified number of degrees (blocking) with power ramping.
     *
     * @param degrees Degrees to move (positive or negative)
     * @param maxPower Maximum power to apply (direction determined by sign of degrees)
     * @param driveAction Runnable to call during the movement (e.g., drive update)
     * @param opModeActive Boolean flag to check if opmode is still active
     */
    public void moveBlocking(double degrees, double maxPower, Runnable driveAction, boolean opModeActive) {
        long duration = (long) (Math.abs(degrees) * msPerDegree);
        double direction = (degrees > 0) ? 1.0 : -1.0;

        executeRampedMove(duration, maxPower * direction, driveAction, opModeActive);

        currentDegrees += degrees;
    }

    /**
     * Move the servo a specified number of degrees (blocking).
     *
     * @param degrees Degrees to move (positive or negative)
     * @param power Power to apply (direction determined by sign of degrees)
     */
    public void moveBlocking(double degrees, double power) {
        long duration = (long) (Math.abs(degrees) * msPerDegree);
        double appliedPower = (degrees > 0) ? Math.abs(power) : -Math.abs(power);

        servo.setPower(appliedPower);
        sleep(duration);
        servo.setPower(0);

        currentDegrees += degrees;
    }

    /**
     * Move the servo with a multiplier applied to timing (blocking with drive action).
     * Useful for auto-movements that need compensation.
     *
     * @param degrees Degrees to move
     * @param maxPower Maximum power to apply
     * @param multiplier Timing multiplier (e.g., 1.05 for +5% time)
     * @param driveAction Runnable to call during the movement
     * @param opModeActive Boolean flag to check if opmode is still active
     */
    public void moveBlockingWithMultiplier(double degrees, double maxPower, double multiplier,
                                           Runnable driveAction, boolean opModeActive) {
        long duration = (long) (Math.abs(degrees) * msPerDegree * multiplier);
        double direction = (degrees > 0) ? 1.0 : -1.0;

        executeRampedMove(duration, maxPower * direction, driveAction, opModeActive);

        currentDegrees += degrees;
    }

    /**
     * Execute a ramped movement to reduce overshoot.
     */
    private void executeRampedMove(long duration, double targetPower, Runnable driveAction, boolean opModeActive) {
        ElapsedTime timer = new ElapsedTime();

        long rampUpTime = (long) (duration * RAMP_UP_PERCENT);
        long rampDownTime = (long) (duration * RAMP_DOWN_PERCENT);
        long fullSpeedTime = duration - rampDownTime;

        while (opModeActive && timer.milliseconds() < duration) {
            long elapsed = (long) timer.milliseconds();
            double currentPower;

            if (elapsed < rampUpTime) {
                // Ramp up phase
                double rampPercent = (double) elapsed / rampUpTime;
                currentPower = targetPower * (0.5 + (0.5 * rampPercent));  // Start at 50% power
            } else if (elapsed < fullSpeedTime) {
                // Full speed phase
                currentPower = targetPower;
            } else {
                // Ramp down phase
                double rampPercent = (double) (duration - elapsed) / rampDownTime;
                currentPower = targetPower * (0.3 + (0.7 * rampPercent));  // End at 30% power
            }

            servo.setPower(currentPower);
            driveAction.run();
        }

        // Apply brief brake in opposite direction to stop coasting
        servo.setPower(-targetPower * 0.3);
        sleepWithDrive(MIN_STOP_TIME_MS, driveAction, opModeActive);

        servo.setPower(0);
    }

    /**
     * Start a non-blocking movement.
     * Must call update() in a loop to complete the movement.
     *
     * @param degrees Degrees to move
     * @param power Power to apply
     */
    public void startMove(double degrees, double power) {
        targetDuration = (long) (Math.abs(degrees) * msPerDegree);
        targetPower = (degrees > 0) ? Math.abs(power) : -Math.abs(power);

        servo.setPower(targetPower);
        moveTimer.reset();
        isMoving = true;
    }

    /**
     * Start a non-blocking movement with timing multiplier.
     *
     * @param degrees Degrees to move
     * @param power Power to apply
     * @param multiplier Timing multiplier
     */
    public void startMoveWithMultiplier(double degrees, double power, double multiplier) {
        targetDuration = (long) (Math.abs(degrees) * msPerDegree * multiplier);
        targetPower = (degrees > 0) ? Math.abs(power) : -Math.abs(power);

        servo.setPower(targetPower);
        moveTimer.reset();
        isMoving = true;
    }

    /**
     * Update non-blocking movement. Call this in your main loop.
     *
     * @return true if movement is complete, false if still moving
     */
    public boolean update() {
        if (!isMoving) {
            return true;
        }

        if (moveTimer.milliseconds() >= targetDuration) {
            // Apply brief brake to stop coasting
            servo.setPower(-targetPower * 0.3);
            sleep(MIN_STOP_TIME_MS);
            servo.setPower(0);
            isMoving = false;
            return true;
        }

        return false;
    }

    /**
     * Check if a non-blocking movement is in progress.
     */
    public boolean isMoving() {
        return isMoving;
    }

    /**
     * Stop any current movement immediately.
     */
    public void stop() {
        servo.setPower(0);
        isMoving = false;
    }

    /**
     * Adjust the calibration (ms per degree).
     */
    public void adjustCalibration(double delta) {
        msPerDegree += delta;
        if (msPerDegree < 0.1) {
            msPerDegree = 0.1;
        }
    }

    /**
     * Reset the degree counter to zero.
     */
    public void zeroDegrees() {
        currentDegrees = 0.0;
    }

    /**
     * Get the current degree position.
     */
    public double getCurrentDegrees() {
        return currentDegrees;
    }

    /**
     * Get the current ms per degree calibration.
     */
    public double getMsPerDegree() {
        return msPerDegree;
    }

    /**
     * Set the ms per degree directly.
     */
    public void setMsPerDegree(double msPerDegree) {
        this.msPerDegree = msPerDegree;
    }

    // Helper methods

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void sleepWithDrive(long ms, Runnable driveAction, boolean opModeActive) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive && t.milliseconds() < ms) {
            driveAction.run();
        }
    }
}