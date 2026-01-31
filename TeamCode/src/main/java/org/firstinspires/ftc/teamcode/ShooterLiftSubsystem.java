package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterLiftSubsystem {

    private DcMotorEx shooterMotor;
    private CRServo liftServo;

    // Lift Constants
    private static final double TARGET_LIFT_DEGREES = 60.0;
    private double msPerDegree = 5.0;
    private double currentDegrees = 0.0;
    private boolean isLiftUp = false;

    // Shooter Constants & State
    private boolean isShooterOn = false;

    // PIDF Constants (Adjusted by User)
    private static final double DEFAULT_P = 10.0;
    private static final double DEFAULT_I = 0.0;
    private static final double DEFAULT_D = 0.0;
    private static final double DEFAULT_F = 20.7; // User requested 20.7

    private double p = DEFAULT_P;
    private double i = DEFAULT_I;
    private double d = DEFAULT_D;
    private double f = DEFAULT_F;

    // Target Velocity (User requested 2500)
    private double targetVelocity = 2500;

    public ShooterLiftSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        liftServo = hardwareMap.get(CRServo.class, "liftServo");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // PIDF Setup (Run Using Encoder matches FlywheelTuner)
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply initial PIDF
        updatePIDF();

        liftServo.setPower(0);
        currentDegrees = 0.0;
    }

    // --- Controls ---

    public void toggleShooter() {
        isShooterOn = !isShooterOn;
        if (isShooterOn) {
            shooterMotor.setVelocity(targetVelocity);
        } else {
            shooterMotor.setVelocity(0);
        }
    }

    public void setShooterPower(double ratio) {
        // Ratio 1.0 = Target Velocity
        // Ratio 0.0 = 0
        if (ratio > 0) {
            isShooterOn = true;
            shooterMotor.setVelocity(targetVelocity * ratio);
        } else {
            isShooterOn = false;
            shooterMotor.setVelocity(0);
        }
    }

    // --- PIDF Tuning ---

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

    public void toggleLift() {
        if (isLiftUp) {
            moveLiftDegrees(-TARGET_LIFT_DEGREES);
            isLiftUp = false;
        } else {
            moveLiftDegrees(TARGET_LIFT_DEGREES);
            isLiftUp = true;
        }
    }

    // Toggle with drive support
    public void toggleLift(Runnable driveAction, boolean opModeActive) {
        if (isLiftUp) {
            moveLiftDegrees(-TARGET_LIFT_DEGREES, driveAction, opModeActive);
            isLiftUp = false;
        } else {
            moveLiftDegrees(TARGET_LIFT_DEGREES, driveAction, opModeActive);
            isLiftUp = true;
        }
    }

    // --- Core Movement Logic ---

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

    // --- Calibration ---

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

    public boolean isShooterOn() {
        return isShooterOn;
    }

    // --- Macros ---

    public void shootMacro(SpindexerSubsystem spindexer, Runnable driveAction, boolean opModeActive) {
        // Spin Up
        setShooterPower(1.0);

        // Wait for Spin Up (1000ms for safety)
        sleepWithDrive(1000, driveAction, opModeActive);

        int shots = 3;
        for (int i = 0; i < shots && opModeActive; i++) {
            // Lift Up
            moveLiftDegrees(TARGET_LIFT_DEGREES, driveAction, opModeActive);
            isLiftUp = true;

            sleepWithDrive(150, driveAction, opModeActive);

            // Lift Down
            moveLiftDegrees(-TARGET_LIFT_DEGREES, driveAction, opModeActive);
            isLiftUp = false;

            sleepWithDrive(150, driveAction, opModeActive);

            // Index
            if (i < shots - 1) {
                spindexer.setSpindexerPower(SpindexerSubsystem.SPIN_POWER);
                sleepWithDrive(380, driveAction, opModeActive);
                spindexer.setSpindexerPower(0);
            }
        }

        isLiftUp = false;

        // Done
        setShooterPower(0);
    }

    private void sleepWithDrive(long ms, Runnable driveAction, boolean opModeActive) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive && t.milliseconds() < ms) {
            driveAction.run();
        }
    }
}
