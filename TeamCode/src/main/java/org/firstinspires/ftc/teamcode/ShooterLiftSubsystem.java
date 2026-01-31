package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Flywheel + Lift subsystem.
 *
 * Notes:
 * - 1st Ball: 60.0 deg (Standard)
 * - 2nd Ball: 132.0 deg (Increased to fix undershoot)
 * - 3rd Ball: 126.0 deg (Standard)
 */
public class ShooterLiftSubsystem {

    private final DcMotorEx shooterMotor;
    private final ServoMovementHelper liftHelper;

    // Lift constants
    private static final double TARGET_LIFT_DEGREES = 65.0;
    private static final double LIFT_POWER = 1.0;

    private double initialShootAngleOffset = 50;
    private double secondaryShootAngleOffset = 0;

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
        CRServo liftServo = hardwareMap.get(CRServo.class, "liftServo");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDF();

        // Initialize lift helper with 5.0 ms/degree
        liftHelper = new ServoMovementHelper(liftServo, 5.0);
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
            liftHelper.moveBlocking(-TARGET_LIFT_DEGREES, LIFT_POWER);
            isLiftUp = false;
        } else {
            liftHelper.moveBlocking(TARGET_LIFT_DEGREES, LIFT_POWER);
            isLiftUp = true;
        }
    }

    public void toggleLift(Runnable driveAction, boolean opModeActive) {
        if (isLiftUp) {
            liftHelper.moveBlocking(-TARGET_LIFT_DEGREES, LIFT_POWER, driveAction, opModeActive);
            isLiftUp = false;
        } else {
            liftHelper.moveBlocking(TARGET_LIFT_DEGREES, LIFT_POWER, driveAction, opModeActive);
            isLiftUp = true;
        }
    }

    // --------------------
    // Calibration / getters
    // --------------------

    public void adjustCalibration(double deltaMsPerDeg) {
        liftHelper.adjustCalibration(deltaMsPerDeg);
    }

    public void zeroDegrees() {
        liftHelper.zeroDegrees();
    }

    public double getCurrentDegrees() {
        return liftHelper.getCurrentDegrees();
    }

    public double getMsPerDegree() {
        return liftHelper.getMsPerDegree();
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

    // --------------------
    // Macro: Auto shoot
    // --------------------

    public void shootMacro(SpindexerSubsystem spindexer, Runnable driveAction, boolean opModeActive) {
        if (!opModeActive)
            return;

        // How many balls to shoot (0..3).
        int shots = Math.min(3, spindexer.getBallsLoaded());
        if (shots <= 0)
            return;

        // --- SETUP FOR 1st BALL ---
        // You said 1st ball works, so we keep this at 60.0
        double firstShotDegrees = 60.0 + initialShootAngleOffset;

        double totalSpindexerRotation = firstShotDegrees;

        // 1) Pre-align
        spindexer.moveSpindexerAutoDegrees(firstShotDegrees, driveAction, opModeActive);

        // 2) Spin up
        setShooterPower(1.0);

        // 3) Shoot sequence
        for (int iShot = 0; iShot < shots && opModeActive; iShot++) {

            // Wait for flywheel to recover
            sleepWithDrive(350, driveAction, opModeActive);

            // Lift up to fire
            liftHelper.moveBlocking(TARGET_LIFT_DEGREES, LIFT_POWER, driveAction, opModeActive);
            isLiftUp = true;

            sleepWithDrive(150, driveAction, opModeActive);

            // Lift down
            liftHelper.moveBlocking(-TARGET_LIFT_DEGREES, LIFT_POWER, driveAction, opModeActive);
            isLiftUp = false;

            // Index to next ball if there is one remaining
            if (iShot < shots - 1) {
                double nextIncrement;

                if (iShot == 0) {
                    // --- MOVING TO 2ND BALL ---
                    // You said this UNDER shoots.
                    // Increased from 126.0 to 132.0 to make it go further.
                    nextIncrement = 132.0 + secondaryShootAngleOffset;
                } else {
                    // --- MOVING TO 3RD BALL ---
                    // You said this works. Kept at 126.0.
                    nextIncrement = 126.0 + secondaryShootAngleOffset;
                }

                spindexer.moveSpindexerAutoDegrees(nextIncrement, driveAction, opModeActive);
                totalSpindexerRotation += nextIncrement;
            }
        }

        // 4) Stop shooter
        setShooterPower(0.0);
        isLiftUp = false;

        // 5) Return the spindexer to intake position
        spindexer.moveSpindexerAutoDegrees(-totalSpindexerRotation, driveAction, opModeActive);
    }

    // Helper method for timing with drive action
    private void sleepWithDrive(long ms, Runnable driveAction, boolean opModeActive) {
        com.qualcomm.robotcore.util.ElapsedTime t = new com.qualcomm.robotcore.util.ElapsedTime();
        while (opModeActive && t.milliseconds() < ms) {
            driveAction.run();
        }
    }
}