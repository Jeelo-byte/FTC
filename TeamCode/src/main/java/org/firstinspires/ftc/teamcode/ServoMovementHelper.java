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
    private double currentPower = 0.0;

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


    public CRServo getServo() {
        return servo;
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
     * Move the servo a specified number of degrees (blocking) with drive action.
     *
     * @param degrees Degrees to move (positive or negative)
     * @param power Power to apply (direction determined by sign of degrees)
     * @param driveAction Runnable to call during the movement (e.g., drive update)
     * @param opModeActive Boolean flag to check if opmode is still active
     */
    public void moveBlocking(double degrees, double power, Runnable driveAction, boolean opModeActive) {
        long duration = (long) (Math.abs(degrees) * msPerDegree);
        double appliedPower = (degrees > 0) ? Math.abs(power) : -Math.abs(power);

        servo.setPower(appliedPower);
        sleepWithDrive(duration, driveAction, opModeActive);
        servo.setPower(0);

        currentDegrees += degrees;
    }

    /**
     * Move the servo with a multiplier applied to timing (blocking with drive action).
     * Useful for auto-movements that need compensation.
     *
     * @param degrees Degrees to move
     * @param power Power to apply
     * @param multiplier Timing multiplier (e.g., 1.05 for +5% time)
     * @param driveAction Runnable to call during the movement
     * @param opModeActive Boolean flag to check if opmode is still active
     */
    public void moveBlockingWithMultiplier(double degrees, double power, double multiplier,
                                           Runnable driveAction, boolean opModeActive) {
        long duration = (long) (Math.abs(degrees) * msPerDegree * multiplier);
        double appliedPower = (degrees > 0) ? Math.abs(power) : -Math.abs(power);

        servo.setPower(appliedPower);
        sleepWithDrive(duration, driveAction, opModeActive);
        servo.setPower(0);

        currentDegrees += degrees;
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
        currentPower = (degrees > 0) ? Math.abs(power) : -Math.abs(power);

        servo.setPower(currentPower);
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
        currentPower = (degrees > 0) ? Math.abs(power) : -Math.abs(power);

        servo.setPower(currentPower);
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