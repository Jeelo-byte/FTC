package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Flywheel Tuner with Lift", group = "Tuning")
public class FlywheelTuner extends LinearOpMode {

    private DcMotorEx flywheelMotor;
    private Servo liftServo;

    // PID Coefficients
    public double p = 10.0;
    public double i = 0.0;
    public double d = 0.0;
    public double f = 11.7;

    public double targetVelocity = 1500;
    public double vIncrement = 50;
    public double pIncrement = 1.0;
    public double fIncrement = 0.5;

    // Lift servo positions (mutable for calibration)
    // GEAR RATIO ADJUSTMENT: 15T on Servo -> 20T on Lift. Ratio is 20/15 = 1.33x
    // travel needed.
    // Original Delta was 0.6 (0.7 - 0.1). New Delta is 0.6 * 1.33 = 0.8.
    double LIFT_DOWN_POS = 0.7; // From MotorControl2
    double LIFT_UP_POS = 0.1; // From MotorControl2

    // Debouncing for buttons
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastY = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;
    boolean lastRightBumper = false;
    boolean lastLeftBumper = false;
    boolean lastRightStickButton = false;

    // State variables
    boolean isFlywheelOn = false;
    boolean isLiftUp = false;
    boolean isEditingUp = true; // Start editing UP position by default

    @Override
    public void runOpMode() {
        // Initialize flywheel motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients initialCoeffs = new PIDFCoefficients(p, i, d, f);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, initialCoeffs);

        // Initialize lift servo
        liftServo = hardwareMap.get(Servo.class, "lift_servo");
        liftServo.setPosition(LIFT_DOWN_POS);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Lift", "L1 to toggle | R1/L1 + Stick to calibrate");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentRightBumper = gamepad1.right_bumper;
            boolean currentLeftBumper = gamepad1.left_bumper;
            boolean currentRightStickButton = gamepad1.right_stick_button;

            // =========================================================
            // LIFT CALIBRATION (R1/L1 + Right Stick Button)
            // =========================================================

            // Toggle Edit Mode (Up/Down) - Right Stick Button
            if (currentRightStickButton && !lastRightStickButton) {
                isEditingUp = !isEditingUp;
            }
            lastRightStickButton = currentRightStickButton;

            // Adjust Value with R1/L1 (only when right stick button is pressed)
            if (gamepad1.right_stick_button) {
                double delta = 0.01;

                if (currentRightBumper && !lastRightBumper) {
                    if (isEditingUp) {
                        LIFT_UP_POS += delta;
                        if (LIFT_UP_POS > 1.0)
                            LIFT_UP_POS = 1.0;
                    } else {
                        LIFT_DOWN_POS += delta;
                        if (LIFT_DOWN_POS > 1.0)
                            LIFT_DOWN_POS = 1.0;
                    }
                }

                if (currentLeftBumper && !lastLeftBumper) {
                    if (isEditingUp) {
                        LIFT_UP_POS -= delta;
                        if (LIFT_UP_POS < 0.0)
                            LIFT_UP_POS = 0.0;
                    } else {
                        LIFT_DOWN_POS -= delta;
                        if (LIFT_DOWN_POS < 0.0)
                            LIFT_DOWN_POS = 0.0;
                    }
                }
            }

            // =========================================================
            // LIFT TOGGLE (Left Bumper when NOT calibrating)
            // =========================================================

            if (!gamepad1.right_stick_button && currentLeftBumper && !lastLeftBumper) {
                isLiftUp = !isLiftUp;
                liftServo.setPosition(isLiftUp ? LIFT_UP_POS : LIFT_DOWN_POS);
            }

            // =========================================================
            // FLYWHEEL CONTROLS
            // =========================================================

            // Adjust target velocity
            if (currentDpadUp && !lastDpadUp)
                targetVelocity += vIncrement;
            if (currentDpadDown && !lastDpadDown)
                targetVelocity -= vIncrement;

            // Adjust P coefficient
            if (currentY && !lastY)
                p += pIncrement;
            if (currentA && !lastA)
                p -= pIncrement;

            // Adjust F coefficient
            if (currentB && !lastB)
                f += fIncrement;
            if (currentX && !lastX)
                f -= fIncrement;

            // Toggle flywheel on/off (only when NOT calibrating lift)
            if (!gamepad1.right_stick_button && currentRightBumper && !lastRightBumper) {
                isFlywheelOn = !isFlywheelOn;
            }

            // Update button states
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastY = currentY;
            lastA = currentA;
            lastB = currentB;
            lastX = currentX;
            lastRightBumper = currentRightBumper;
            lastLeftBumper = currentLeftBumper;

            // Update PIDF coefficients if changed
            PIDFCoefficients currentCoeffs = flywheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (currentCoeffs.p != p || currentCoeffs.f != f) {
                flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
            }

            // Set flywheel velocity
            if (isFlywheelOn) {
                flywheelMotor.setVelocity(targetVelocity);
            } else {
                flywheelMotor.setVelocity(0);
            }

            // =========================================================
            // TELEMETRY
            // =========================================================

            telemetry.addData("Flywheel State", isFlywheelOn ? "RUNNING" : "STOPPED");
            telemetry.addData("Target Vel", targetVelocity);
            telemetry.addData("Actual Vel", flywheelMotor.getVelocity());
            telemetry.addData("P", "%.2f", p);
            telemetry.addData("F", "%.2f", f);

            telemetry.addLine("\n--- Lift Calibration ---");
            if (gamepad1.right_stick_button) {
                telemetry.addData("MODE", "CALIBRATING");
                telemetry.addData("EDITING", isEditingUp ? ">> UP <<" : ">> DOWN <<");
            } else {
                telemetry.addData("MODE", "Normal (Hold Right Stick to calibrate)");
            }
            telemetry.addData("UP Position", "%.2f", LIFT_UP_POS);
            telemetry.addData("DOWN Position", "%.2f", LIFT_DOWN_POS);
            telemetry.addData("Current State", isLiftUp ? "UP" : "DOWN");

            telemetry.addLine("\n--- Controls ---");
            telemetry.addData("D-Pad Up/Down", "Adjust Target Velocity");
            telemetry.addData("Y/A", "Adjust P");
            telemetry.addData("B/X", "Adjust F");
            telemetry.addData("Right Bumper", "Toggle Flywheel");
            telemetry.addData("Left Bumper", "Toggle Lift");
            telemetry.addData("Right Stick + R1/L1", "Calibrate Lift");

            telemetry.update();
        }
    }
}