package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel Tuner Fixed", group = "Tuning")
public class FlywheelTuner extends LinearOpMode {

    private DcMotorEx flywheelMotor;

    // --- CALCULATED TUNING VARIABLES ---
    // F (Feedforward): Calculated for REV HD Hex Motor (approx 11.7 for 6000 RPM)
    // P (Proportional): Set to 10 for good error correction on heavy wheels
    public double p = 10.0;
    public double i = 0.0;
    public double d = 0.0;
    public double f = 11.7;

    // Start target at 1500 Ticks Per Second (approx 3200 RPM)
    public double targetVelocity = 1500;
    public double vIncrement = 50;
    public double pIncrement = 1.0;
    public double fIncrement = 0.5;

    // --- INPUT TRACKING ---
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastY = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;

    @Override
    public void runOpMode() {
        // Config name is now just "flywheel"
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        // CHECK DIRECTION: If it spins backwards, change to FORWARD
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply the Calculated PIDF Coefficients
        PIDFCoefficients initialCoeffs = new PIDFCoefficients(p, i, d, f);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, initialCoeffs);

        telemetry.addData("Status", "Tuned with defaults (F=11.7, P=10)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- INPUTS ---
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;

            // Target Velocity
            if (currentDpadUp && !lastDpadUp) targetVelocity += vIncrement;
            if (currentDpadDown && !lastDpadDown) targetVelocity -= vIncrement;

            // P (Proportional) Adjustment
            if (currentY && !lastY) p += pIncrement;
            if (currentA && !lastA) p -= pIncrement;

            // F (Feedforward) Adjustment
            if (currentB && !lastB) f += fIncrement;
            if (currentX && !lastX) f -= fIncrement;

            // Update Input History
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastY = currentY;
            lastA = currentA;
            lastB = currentB;
            lastX = currentX;

            // --- UPDATE MOTOR PIDF ---
            // Only update if values changed (Efficiency)
            PIDFCoefficients currentCoeffs = flywheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (currentCoeffs.p != p || currentCoeffs.f != f) {
                flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
            }

            // --- RUN MOTOR ---
            if (gamepad1.right_trigger > 0.1) {
                flywheelMotor.setVelocity(targetVelocity);
            } else {
                flywheelMotor.setVelocity(0);
            }

            // --- TELEMETRY ---
            telemetry.addData("Target Vel", targetVelocity);
            telemetry.addData("Actual Vel", flywheelMotor.getVelocity());
            telemetry.addLine("----------------");
            telemetry.addData("P (Y/A)", "%.2f", p);
            telemetry.addData("F (B/X)", "%.2f", f);
            telemetry.update();
        }
    }
}
