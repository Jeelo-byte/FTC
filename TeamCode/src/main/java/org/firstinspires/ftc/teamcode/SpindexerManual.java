package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Spindexer Manual Mode", group = "Linear Opmode")
public class SpindexerManual extends LinearOpMode {

    // --- Hardware --- version 3.11111
    private CRServo spindexerServo;
    private Servo liftServo;
    private DcMotor shooterMotor;
    private DcMotor intakeCoreHex;

    // Drive Motors
    private com.qualcomm.robotcore.hardware.DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // --- Constants ---
    // PID Constants (From MotorControl2)
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.00036;
    private static final double MAX_TICKS_PER_SEC = 2796;

    // PID Controllers for each wheel
    private PIDController flPID, frPID, blPID, brPID;

    // Spindexer Logic (CRServo)
    // MotorControl2: SPIN_POWER = -0.3; Left D-Pad = 0.3; Right D-Pad = -0.3
    final double SPIN_POWER = -0.3;
    final double GEAR_RATIO = 15.0 / 20.0;
    final long TIME_120 = (long) (500 * GEAR_RATIO); // Time for 120 deg
    final long TIME_60 = TIME_120 / 2; // Time for 60 deg

    // Lift Constants (Mutable for calibration)
    double LIFT_DOWN_POS = 0.58;
    double LIFT_UP_POS = 0.15; // 0.4
    final int LIFT_WAIT_TIME = 300;

    // --- State Variables ---
    // Toggles
    boolean isShooterOn = false;
    boolean lastSquare = false; // Tracks previous loop state for shooter

    boolean isIntakeOn = false;
    boolean lastBButton = false; // Tracks previous loop state for intake

    // Lift Toggle
    boolean isLiftUp = false;
    boolean lastTriangle = false;

    // Calibration State
    boolean isEditingUp = true; // Start editing UP by default
    boolean lastRightStickButton = false;
    boolean lastRightBumper = false;
    boolean lastLeftBumper = false;

    // Lift Logic
    ElapsedTime liftTimer = new ElapsedTime();
    String liftState = "IDLE";

    // D-Pad Edge Detection
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;

    @Override
    public void runOpMode() {

        // 1. Hardware Map
        spindexerServo = hardwareMap.get(CRServo.class, "spindexer_servo");
        liftServo = hardwareMap.get(Servo.class, "lift_servo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        intakeCoreHex = hardwareMap.get(DcMotor.class, "intake_motor");

        // Drive Motors Map
        leftFront = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "front_left");
        rightFront = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "front_right");
        leftBack = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "back_left");
        rightBack = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "back_right");

        // Directions (Matching MotorControl2: Only leftFront reversed)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // Others default to FORWARD

        // Motor Modes (Matching MotorControl2)
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize PIDs
        flPID = new PIDController(kP, kI, kD, kF);
        frPID = new PIDController(kP, kI, kD, kF);
        blPID = new PIDController(kP, kI, kD, kF);
        brPID = new PIDController(kP, kI, kD, kF);

        // 2. Setup
        spindexerServo.setPower(0);
        liftServo.setPosition(LIFT_DOWN_POS);

        // Shooter Setup
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Intake Setup (Core Hex)
        intakeCoreHex.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeCoreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeCoreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready - Manual Mode (PID Drive)");
        telemetry.update();

        liftServo.setPosition(LIFT_DOWN_POS);

        waitForStart();

        while (opModeIsActive()) {

            // =========================================================
            // 0. Calibration Controls (R1/L1 + Stick Button)
            // =========================================================

            // Toggle Edit Mode (Up/Down) - Right Stick Button
            if (gamepad1.right_stick_button && !lastRightStickButton) {
                isEditingUp = !isEditingUp;
            }
            lastRightStickButton = gamepad1.right_stick_button;

            // Adjust Value
            double delta = 0.01;
            boolean pressedUp = gamepad1.right_bumper && !lastRightBumper;
            boolean pressedDown = gamepad1.left_bumper && !lastLeftBumper;

            if (pressedUp || pressedDown) {
                double change = pressedUp ? delta : -delta;

                if (isEditingUp) {
                    LIFT_UP_POS += change;
                    // Clip
                    if (LIFT_UP_POS > 1.0)
                        LIFT_UP_POS = 1.0;
                    if (LIFT_UP_POS < 0.0)
                        LIFT_UP_POS = 0.0;
                } else {
                    LIFT_DOWN_POS += change;
                    // Clip
                    if (LIFT_DOWN_POS > 1.0)
                        LIFT_DOWN_POS = 1.0;
                    if (LIFT_DOWN_POS < 0.0)
                        LIFT_DOWN_POS = 0.0;
                }
            }
            lastRightBumper = gamepad1.right_bumper;
            lastLeftBumper = gamepad1.left_bumper;

            // =========================================================
            // 1. Spindexer Controls (CRServo Time-Based)
            // =========================================================

            // Left D-Pad: 0.3 Power (Positive) - Matches MotorControl2 Left D-Pad
            if (gamepad1.dpad_left && !lastDpadLeft) {
                spindexerServo.setPower(0.3);
                sleep(TIME_120);
                spindexerServo.setPower(0);
            }
            lastDpadLeft = gamepad1.dpad_left;

            // Right D-Pad: -0.3 Power (Negative) - Matches MotorControl2 Right D-Pad
            if (gamepad1.dpad_right && !lastDpadRight) {
                spindexerServo.setPower(-0.3);
                sleep(TIME_60);
                spindexerServo.setPower(0);

                if (liftState.equals("IDLE")) {
                    liftState = "GOING_UP";
                    liftTimer.reset();
                }
            }
            lastDpadRight = gamepad1.dpad_right;

            // =========================================================
            // 2. Shooter Toggle (Square - Toggle Max Power)
            // =========================================================

            // Toggle on Square press - Toggle Max Power
            if (gamepad1.square && !lastSquare) {
                isShooterOn = !isShooterOn;
            }
            lastSquare = gamepad1.square;

            // Set Power
            if (isShooterOn) {
                shooterMotor.setPower(1.0);
            } else {
                shooterMotor.setPower(0.0);
            }

            // =========================================================
            // 3. Intake Toggle (Button B)
            // =========================================================

            // Only flip the toggle if we are pressing B NOW, but weren't BEFORE
            if (gamepad1.b && !lastBButton) {
                isIntakeOn = !isIntakeOn; // Flip true <-> false
            }
            lastBButton = gamepad1.b; // Save for next loop

            // Set Power
            if (isIntakeOn) {
                intakeCoreHex.setPower(1.0);
            } else {
                intakeCoreHex.setPower(0.0);
            }

            // =========================================================
            // 4. Lift Logic (Automated) - Keeping this for D-Pad Right if needed
            // =========================================================
            switch (liftState) {
                case "GOING_UP":
                    liftServo.setPosition(LIFT_UP_POS);
                    if (liftTimer.milliseconds() > 200) {
                        liftState = "WAITING";
                        liftTimer.reset();
                    }
                    break;
                case "WAITING":
                    if (liftTimer.milliseconds() > LIFT_WAIT_TIME) {
                        liftState = "GOING_DOWN";
                        liftTimer.reset();
                    }
                    break;
                case "GOING_DOWN":
                    liftServo.setPosition(LIFT_DOWN_POS);
                    if (liftTimer.milliseconds() > 300) {
                        liftState = "IDLE";
                    }
                    break;
                case "IDLE":
                    break;
            }

            // =========================================================
            // 5. Lift Test (Triangle)
            // =========================================================

            // Lift Test - Triangle (Sweep -1.0 to 1.0)
            // Lift Test - Triangle (Toggle Up/Down)
            if (gamepad1.triangle && !lastTriangle) {
                isLiftUp = !isLiftUp; // Toggle state

                if (isLiftUp) {
                    liftServo.setPosition(LIFT_UP_POS);
                } else {
                    liftServo.setPosition(LIFT_DOWN_POS);
                }
            }
            lastTriangle = gamepad1.triangle;

            // ...

            // =========================================================
            // 6. Manual Drive (Mecanum PID)
            // =========================================================
            manualDrive();

            // =========================================================
            // Telemetry
            // =========================================================
            telemetry.addData("Shooter", isShooterOn ? "ON" : "OFF");
            telemetry.addData("Intake", isIntakeOn ? "ON" : "OFF");
            telemetry.addData("Lift State", liftState);
            telemetry.addLine("\n--- Lift Calibration ---");
            telemetry.addData("EDITING", isEditingUp ? ">> UP POS <<" : ">> DOWN POS <<");
            telemetry.addData("Current UP", "%.2f", LIFT_UP_POS);
            telemetry.addData("Current DOWN", "%.2f", LIFT_DOWN_POS);

            telemetry.addLine("\n--- Debug ---");
            telemetry.addData("Lift Servo Pos", "%.3f", liftServo.getPosition());
            telemetry.addData("Is Lift Logical Up?", isLiftUp);
            telemetry.update();
        }
    }

    // --- Mecanum Drive Logic ---

    private void manualDrive() {
        double y = gamepad1.left_trigger - gamepad1.right_trigger;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        double flTarget = fl * MAX_TICKS_PER_SEC;
        double frTarget = fr * MAX_TICKS_PER_SEC;
        double blTarget = bl * MAX_TICKS_PER_SEC;
        double brTarget = br * MAX_TICKS_PER_SEC;

        double flCurrent = leftFront.getVelocity();
        double frCurrent = rightFront.getVelocity();
        double blCurrent = leftBack.getVelocity();
        double brCurrent = rightBack.getVelocity();

        double flPower = flPID.calculate(flCurrent, flTarget);
        double frPower = frPID.calculate(frCurrent, frTarget);
        double blPower = blPID.calculate(blCurrent, blTarget);
        double brPower = brPID.calculate(brCurrent, brTarget);

        leftFront.setPower(flPower);
        rightFront.setPower(frPower);
        leftBack.setPower(blPower);
        rightBack.setPower(brPower);
    }

    // PID Controller class
    static class PIDController {
        private double kP, kI, kD, kF;
        private double integralSum = 0;
        private double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();
        private static final double INTEGRAL_MAX = 0.3;

        public PIDController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            timer.reset();
        }

        public double calculate(double current, double target) {
            double dt = timer.seconds();
            timer.reset();

            double error = target - current;
            double P = kP * error;

            integralSum += error * dt;
            integralSum = clamp(integralSum, -INTEGRAL_MAX / kI, INTEGRAL_MAX / kI);
            double I = kI * integralSum;

            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            double D = kD * derivative;

            double F = kF * target;

            lastError = error;

            double output = P + I + D + F;
            return clamp(output, -1.0, 1.0);
        }

        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }
}
