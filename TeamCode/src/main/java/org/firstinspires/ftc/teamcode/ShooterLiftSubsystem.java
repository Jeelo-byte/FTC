package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Flywheel + lift subsystem.
 *
 * Lift is controlled by a CRServo with timed degree estimates.
 */
public class ShooterLiftSubsystem {

    private final DcMotorEx shooterMotor;
    private final CRServo liftServo;

    // Lift config
    private static final double TARGET_LIFT_DEGREES = 65.0;
    private static final double LIFT_POWER = 1.0;
    private double msPerDegree = 5.0;
    private double currentLiftDegrees = 0.0;
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
    private double targetVelocity = 2000 / 1.677;

    // Shoot angle offset (kept for compatibility with existing teleop controls)
    private double initialShootAngleOffset = -8.2;

    public ShooterLiftSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        liftServo = getLiftServo(hardwareMap);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        updatePIDF();
        liftServo.setPower(0.0);
    }

    private CRServo getLiftServo(HardwareMap hardwareMap) {
        try {
            return hardwareMap.get(CRServo.class, "liftServo");
        } catch (RuntimeException e) {
            return hardwareMap.get(CRServo.class, "lift_servo");
        }
    }

    // --------------------
    // Shooter controls
    // --------------------

    public void toggleShooter() {
        isShooterOn = !isShooterOn;
        shooterMotor.setVelocity(isShooterOn ? targetVelocity : 0.0);
    }

    public void setShooterPower(double ratio) {
        if (ratio > 0) {
            isShooterOn = true;
            shooterMotor.setVelocity(targetVelocity * ratio);
        } else {
            isShooterOn = false;
            shooterMotor.setVelocity(0.0);
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
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
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
        moveLiftByDegrees(isLiftUp ? -TARGET_LIFT_DEGREES : TARGET_LIFT_DEGREES, null, true);
        isLiftUp = !isLiftUp;
    }

    public void toggleLift(Runnable driveAction, boolean opModeActive) {
        moveLiftByDegrees(isLiftUp ? -TARGET_LIFT_DEGREES : TARGET_LIFT_DEGREES, driveAction, opModeActive);
        isLiftUp = !isLiftUp;
    }

    public void moveLiftToDegrees(double targetDegrees) {
        moveLiftByDegrees(targetDegrees - currentLiftDegrees, null, true);
        isLiftUp = targetDegrees >= (TARGET_LIFT_DEGREES * 0.5);
    }

    // --------------------
    // Compatibility calibration methods used by current TeleOp
    // --------------------

    public void adjustCalibration(double deltaMsPerDeg) {
        msPerDegree += deltaMsPerDeg;
        if (msPerDegree < 0.1) {
            msPerDegree = 0.1;
        }
    }

    public void zeroDegrees() {
        currentLiftDegrees = 0.0;
        isLiftUp = false;
    }

    public double getCurrentDegrees() {
        return currentLiftDegrees;
    }

    // Kept for backward compatibility with TeleOp telemetry label
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
        if (!opModeActive) {
            return;
        }

        int shots = Math.min(3, spindexer.getBallsLoaded());
        if (shots <= 0) {
            return;
        }

        double firstShotDegrees = 90.0 + initialShootAngleOffset;
        double totalSpindexerRotation = firstShotDegrees;

        spindexer.moveSpindexerAutoDegrees(firstShotDegrees, driveAction, opModeActive);
        setShooterPower(1.0);

        for (int iShot = 0; iShot < shots && opModeActive; iShot++) {
            sleepWithDrive(350, driveAction, opModeActive);
            moveLiftByDegrees(TARGET_LIFT_DEGREES, driveAction, opModeActive);
            sleepWithDrive(150, driveAction, opModeActive);
            moveLiftByDegrees(-TARGET_LIFT_DEGREES, driveAction, opModeActive);

            if (iShot < shots - 1) {
                double nextIncrement = (iShot == 0) ? 148.0 : 121.5;
                spindexer.moveSpindexerAutoDegrees(nextIncrement, driveAction, opModeActive);
                totalSpindexerRotation += nextIncrement;
            }
        }

        setShooterPower(0.0);
        isLiftUp = false;
        spindexer.moveSpindexerAutoDegrees(-totalSpindexerRotation, driveAction, opModeActive);
    }

    private void moveLiftByDegrees(double deltaDegrees, Runnable driveAction, boolean opModeActive) {
        if (Math.abs(deltaDegrees) < 1e-6) {
            return;
        }
        long durationMs = (long) Math.abs(deltaDegrees * msPerDegree);
        double power = deltaDegrees > 0 ? Math.abs(LIFT_POWER) : -Math.abs(LIFT_POWER);
        liftServo.setPower(power);
        sleepWithDrive(durationMs, driveAction, opModeActive);
        liftServo.setPower(0.0);
        currentLiftDegrees += deltaDegrees;
    }

    private void sleepWithDrive(long ms, Runnable driveAction, boolean opModeActive) {
        com.qualcomm.robotcore.util.ElapsedTime t = new com.qualcomm.robotcore.util.ElapsedTime();
        while (opModeActive && t.milliseconds() < ms) {
            driveAction.run();
        }
    }
}
