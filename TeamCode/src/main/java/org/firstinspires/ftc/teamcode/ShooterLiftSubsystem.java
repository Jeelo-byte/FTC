package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Flywheel + Lift subsystem.
 *
 * Notes:
 * - Auto-shoot uses the spindexer sequence: +60°, then +120°, then +120°.
 * - Spindexer moves for auto-shoot should use SpindexerSubsystem's AUTO timing
 * (+5%).
 * - Lift angles in auto-shoot are also bumped slightly (+5%) to compensate
 * under-rotation.
 */
public class ShooterLiftSubsystem {

    private final DcMotorEx shooterMotor;
    private final CRServo liftServo;

    // Lift constants
    private static final double TARGET_LIFT_DEGREES = 65.0;
    private static final double AUTO_SHOOT_ANGLE_MULTIPLIER = 1.05; // +5% for lift angles only

    private double initialShootAngleOffset = 45;

    private double msPerDegree = 5.0;
    private double currentDegrees = 0.0;
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
        liftServo = hardwareMap.get(CRServo.class, "liftServo");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDF();

        liftServo.setPower(0);
        currentDegrees = 0.0;
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
            moveLiftDegrees(-TARGET_LIFT_DEGREES);
            isLiftUp = false;
        } else {
            moveLiftDegrees(TARGET_LIFT_DEGREES);
            isLiftUp = true;
        }
    }

    public void toggleLift(Runnable driveAction, boolean opModeActive) {
        if (isLiftUp) {
            moveLiftDegrees(-TARGET_LIFT_DEGREES, driveAction, opModeActive);
            isLiftUp = false;
        } else {
            moveLiftDegrees(TARGET_LIFT_DEGREES, driveAction, opModeActive);
            isLiftUp = true;
        }
    }

    private void moveLiftDegrees(double degrees) {
        long duration = (long) (Math.abs(degrees) * msPerDegree);
        double power = (degrees > 0) ? -1.0 : 1.0;

        liftServo.setPower(power);
        sleep(duration);
        liftServo.setPower(0);

        currentDegrees += degrees;
    }

    private void moveLiftDegrees(double degrees, Runnable driveAction, boolean opModeActive) {
        long duration = (long) (Math.abs(degrees) * msPerDegree);
        double power = (degrees > 0) ? -1.0 : 1.0;

        liftServo.setPower(power);
        sleepWithDrive(duration, driveAction, opModeActive);
        liftServo.setPower(0);

        currentDegrees += degrees;
    }

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

    // --------------------
    // Calibration / getters
    // --------------------

    public void adjustCalibration(double deltaMsPerDeg) {
        msPerDegree += deltaMsPerDeg;
        if (msPerDegree < 0.1)
            msPerDegree = 0.1;
    }

    public void zeroDegrees() {
        currentDegrees = 0.0;
    }

    public double getCurrentDegrees() {
        return currentDegrees;
    }

    public double getMsPerDegree() {
        return msPerDegree;
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

        // How many balls to shoot (0..3). If you want always 3, change this to '3'.
        int shots = Math.min(3, spindexer.getBallsLoaded());
        if (shots <= 0)
            return;

        // 1) Pre-align: +60 degrees (half of 120) PLUS OFFSET, using the SAME auto-turn
        // logic as auto-intake
        // initialShootAngleOffset to contur for servo spedness
        spindexer.moveSpindexerAutoDegrees(60.0 + initialShootAngleOffset, driveAction, opModeActive);

        // 2) Spin up
        setShooterPower(1.0);

        // 3) Shoot sequence; after shot 1 and 2, index +120 using auto timing
        for (int iShot = 0; iShot < shots && opModeActive; iShot++) {

            // Wait for flywheel to recover before each shot
            sleepWithDrive(350, driveAction, opModeActive);

            // Lift up to fire (slightly boosted angle)
            moveLiftDegrees(TARGET_LIFT_DEGREES * AUTO_SHOOT_ANGLE_MULTIPLIER, driveAction, opModeActive);
            isLiftUp = true;

            sleepWithDrive(150, driveAction, opModeActive);

            // Lift down
            moveLiftDegrees(-(TARGET_LIFT_DEGREES * AUTO_SHOOT_ANGLE_MULTIPLIER), driveAction, opModeActive);
            isLiftUp = false;

            // Index to next ball if there is one remaining:
            // shot 0 -> +120
            // shot 1 -> +120
            if (iShot < shots - 1) {
                spindexer.moveSpindexerAutoDegrees(160.0 // 2nd rotation
                        , driveAction, opModeActive);
            }
        }

        // 4) Stop shooter
        setShooterPower(0.0);
        isLiftUp = false;
    }
} // k
