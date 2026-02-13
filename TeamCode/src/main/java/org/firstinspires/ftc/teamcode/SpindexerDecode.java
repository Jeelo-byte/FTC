package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;

@TeleOp(name = "Spindexer V12 + Mecanum PID", group = "Linear Opmode")
public class SpindexerDecode extends LinearOpMode {

    // --- SPINDEXER HARDWARE ---
    private DcMotor spinMotor;
    private DcMotorEx shooterMotor;
    private ColorSensor colorSensor;
    private Servo liftServo;

    // --- DRIVE HARDWARE ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private PIDController flPID, frPID, blPID, brPID;

    // --- SPINDEXER CONSTANTS ---
    static final double TICKS_PER_REV = 288;
    static final double GEAR_RATIO = 20.0 / 15.0;
    static final double MAX_SPEED = 0.55;
    static final double PID_kP = 0.006;
    static final double MIN_POWER = 0.2;
    static final double TOLERANCE_TICKS = 2;

    // --- DRIVE CONSTANTS ---
    static final double MAX_TICKS_PER_SEC = 2796;

    // --- TUNING ---
    static final int INTAKE_RETURN_OFFSET = 0;

    // --- SHOOTER PIDF CONSTANTS ---
    public static final double SHOOTER_P = 10.0;
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 20.7;
    public static final double TARGET_VELOCITY = 2000 / 1.677;

    // --- INVENTORY ---
    String[] ballInventory = {"Empty", "Empty", "Empty"};
    final double[] SLOT_DISTANCES = {180.0, 300.0, 60.0};
    double currentVirtualPosition = 0.0;

    // --- LIFT ---
    double liftUpTarget = 0.0;
    double liftDownTarget = 0.5;
    boolean isLiftUp = false;
    boolean isShooterOn = false;

    @Override
    public void runOpMode() {

        // --- INIT SPINDEXER ---
        spinMotor = hardwareMap.get(DcMotor.class, "spinmotor");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        liftServo = hardwareMap.get(Servo.class, "liftservo");

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setTargetPosition(0);
        spinMotor.setPower(0);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        colorSensor.enableLed(true);
        liftServo.setPosition(liftDownTarget);

        // --- INIT DRIVE ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // Directions copied from your drive code
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double drivekP = 0.0001;
        double drivekI = 0.0;
        double drivekD = 0.0;
        double drivekF = 0.00036;

        flPID = new PIDController(drivekP, drivekI, drivekD, drivekF);
        frPID = new PIDController(drivekP, drivekI, drivekD, drivekF);
        blPID = new PIDController(drivekP, drivekI, drivekD, drivekF);
        brPID = new PIDController(drivekP, drivekI, drivekD, drivekF);

        telemetry.addData("Status", "Ready: Spindexer + Mecanum PID");
        telemetry.update();

        waitForStart();

        int ballCount = 0;
        boolean automationRunning = false;
        boolean lastAState = false;
        boolean lastRBState = false;
        boolean lastLBState = false;
        boolean lastYState = false;
        boolean lastSquareState = false;

        while (opModeIsActive()) {

            // --- DRIVE LOGIC (Added) ---
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;

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

            double flCurrent = frontLeft.getVelocity();
            double frCurrent = frontRight.getVelocity();
            double blCurrent = backLeft.getVelocity();
            double brCurrent = backRight.getVelocity();

            double flPower = flPID.calculate(flCurrent, flTarget);
            double frPower = frPID.calculate(frCurrent, frTarget);
            double blPower = blPID.calculate(blCurrent, blTarget);
            double brPower = brPID.calculate(brCurrent, brTarget);

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            // --- SPINDEXER LOGIC (Existing) ---

            // SHOOTER TOGGLE
            if (gamepad1.x && !lastSquareState) {
                isShooterOn = !isShooterOn;
                shooterMotor.setVelocity(isShooterOn ? TARGET_VELOCITY : 0);
            }
            lastSquareState = gamepad1.x;

            // SMART SCORING
            if (gamepad1.right_bumper && !lastRBState) {
                // NOTE: Robot will stop driving while this sequence runs because of sleep()
                ArrayList<Integer> shotOrder = calculateShotOrder();
                performSmartScoring(shotOrder);

                moveSpindexerPID(60.0, INTAKE_RETURN_OFFSET);

                ballCount = 0;
                ballInventory[0] = "Empty"; ballInventory[1] = "Empty"; ballInventory[2] = "Empty";
                currentVirtualPosition = 0.0;
            }
            lastRBState = gamepad1.right_bumper;

            // DEBUG SET
            if (gamepad1.left_bumper && !lastLBState) {
                ballInventory[0] = "Green"; ballInventory[1] = "Purple"; ballInventory[2] = "Purple";
                ballCount = 3;
                telemetry.addData("Debug", "Set: Green-Purple-Purple");
                telemetry.update();
            }
            lastLBState = gamepad1.left_bumper;

            // INTAKE
            if (gamepad1.a && !lastAState) {
                ballCount = 0;
                ballInventory[0] = "Empty"; ballInventory[1] = "Empty"; ballInventory[2] = "Empty";
                automationRunning = true;
            }
            lastAState = gamepad1.a;

            if (automationRunning) {
                int red = colorSensor.red();
                int green = colorSensor.green();
                int blue = colorSensor.blue();
                boolean purple = (blue > green) && (blue > 45);
                boolean grn = (green > red) && (green > blue) && (green > 100);

                if (purple || grn) {
                    if (ballCount < 3) ballInventory[ballCount] = purple ? "Purple" : "Green";
                    ballCount++;

                    moveSpindexerPID(120.0, 0);

                    sleep(200);
                }
                if (ballCount >= 3) automationRunning = false;
                telemetry.addData("Detecting", purple ? "PURPLE" : (grn ? "GREEN" : "NONE"));
                telemetry.update();
            }

            // LIFT
            if (gamepad1.y && !lastYState) {
                isLiftUp = !isLiftUp;
                liftServo.setPosition(isLiftUp ? liftUpTarget : liftDownTarget);
            }
            lastYState = gamepad1.y;

            telemetry.addData("Shooter", isShooterOn ? "ON" : "OFF");
            telemetry.addData("Velocity", shooterMotor.getVelocity());
            telemetry.addData("Inv", ballInventory[0] + "|" + ballInventory[1] + "|" + ballInventory[2]);
            // Drive Telemetry
            telemetry.addData("FL Pwr", "%.2f", flPower);
            telemetry.update();
        }
    }

    // --- LOGIC ---

    private ArrayList<Integer> calculateShotOrder() {
        ArrayList<Integer> order = new ArrayList<>();
        for (int i = 0; i < 3; i++) if (ballInventory[i].equals("Green")) order.add(i);
        for (int i = 0; i < 3; i++) if (ballInventory[i].equals("Purple")) order.add(i);
        for (int i = 0; i < 3; i++) if (!ballInventory[i].equals("Empty") && !order.contains(i)) order.add(i);
        return order;
    }

    private void performSmartScoring(ArrayList<Integer> order) {
        currentVirtualPosition = 0.0;
        for (int slotIndex : order) {
            telemetry.addData("Shooting", "Slot " + slotIndex);
            telemetry.update();

            double targetRotation = SLOT_DISTANCES[slotIndex];
            double diff = targetRotation - currentVirtualPosition;

            while (diff < 0) diff += 360;
            while (diff >= 360) diff -= 360;

            moveSpindexerPID(diff, 0);

            currentVirtualPosition = targetRotation;
            liftServo.setPosition(liftUpTarget);
            sleep(400);
            liftServo.setPosition(liftDownTarget);
            sleep(300);
        }
    }

    private void moveSpindexerPID(double degrees, int extraOffset) {
        int ticksToAdd = (int)((degrees / 360.0) * TICKS_PER_REV * GEAR_RATIO);
        int targetPos = spinMotor.getCurrentPosition() + ticksToAdd + extraOffset;

        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            int currentPos = spinMotor.getCurrentPosition();
            int error = targetPos - currentPos;

            if (Math.abs(error) < TOLERANCE_TICKS) {
                break;
            }

            double power = error * PID_kP;
            power = Range.clip(power, -MAX_SPEED, MAX_SPEED);

            if (power > 0 && power < MIN_POWER) power = MIN_POWER;
            if (power < 0 && power > -MIN_POWER) power = -MIN_POWER;

            spinMotor.setPower(power);
        }

        spinMotor.setPower(0);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // --- PID CLASS (From Drive Code) ---
    public static class PIDController {
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

        public void reset() {
            integralSum = 0;
            lastError = 0;
            timer.reset();
        }

        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }

        public void setCoefficients(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }
}