package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "Motor Control v7", group = "Line ar Opmode")
public class MotorControl2 extends LinearOpMode {

    // ===== CONFIGURATION BOOLEANS =====
    private static final boolean ENABLE_MOTIF_DETECTION = false;

    // Drive motors
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // PID Controllers for drive motors
    private PIDController flPID, frPID, blPID, brPID;

    // Motor constants
    private static final double MAX_TICKS_PER_SEC = 2796;
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.00036;

    // Intake and spindexer
    private DcMotorEx intakeMotor;
    private DcMotorEx shooterMotor;
    private CRServo spindexerServo;

    // Servos
    private Servo liftServo;

    // Sensors
    private ColorSensor colorSensor;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Intake motor constants
    private double intakeSpeed = -1.0;
    private boolean isIntakeRunning = false;

    // Shooter motor constants
    private static final double SHOOTER_POWER = 1.0;
    private static final int SHOOTER_SPINUP_TIME = 1000;

    // Spindexer servo constants (CRServo with gear ratio)
    private static final double SPIN_POWER = -0.3; // Negative for clockwise rotation
    private static final double GEAR_RATIO = 15.0 / 20.0; // 15T driving 20T
    private static final long ROTATION_TIME_120 = (long) (500 * GEAR_RATIO); // Time in ms for 120° adjusted for gear
    // ratio

    // Lift servo constants with gear ratio
    // GEAR RATIO ADJUSTMENT: 15T on Servo -> 20T on Lift. Ratio is 20/15 = 1.33x
    // travel needed.
    // Original Delta was 0.6 (0.7 - 0.1). New Delta is 0.6 * 1.33 = 0.8.
    private static final double LIFT_DOWN = 0.7;
    private static final double LIFT_UP = 0.1; // Adjusted from 0.7 for gear ratio
    private static final int LIFT_DELAY = 300;

    // Color detection constants (using HSV method)
    private static final int ALPHA_THRESHOLD = 200; // Luminance/Brightness threshold
    private static final float PURPLE_HUE_MIN = 180.0f;
    private static final float PURPLE_HUE_MAX = 290.0f;
    private static final float GREEN_HUE_MIN = 90.0f;
    private static final float GREEN_HUE_MAX = 150.0f;

    // Auto-align constants
    private static final int GOAL_APRILTAG_ID_1 = 20;
    private static final int GOAL_APRILTAG_ID_2 = 24;
    private static final double TARGET_DISTANCE = 30.0;
    private static final double DISTANCE_TOLERANCE = 2.0;
    private static final double YAW_TOLERANCE = 2.0;
    private static final double X_TOLERANCE = 2.5;
    private static final double ALIGN_SPEED = 0.25;
    private static final double SLOW_SPEED = 0.12;

    // Ball storage
    private static final int SLOTS = 3;
    private BallSlot[] ballSlots = new BallSlot[SLOTS];
    private int currentSlot = 0;
    private int ballsLoaded = 0;

    // Motif
    private String[] motif = { "P", "P", "G" };
    private boolean motifDetected = !ENABLE_MOTIF_DETECTION;

    // State flags
    private boolean isAutoIntaking = false;
    private boolean isShooting = false;
    private boolean isAligning = false;
    private boolean isAligned = false;

    // Debouncing
    private boolean squarePressed = false;
    private boolean trianglePressed = false;
    private boolean circlePressed = false;
    private boolean crossPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    class BallSlot {
        String color = "";
        boolean isEmpty = true;
    }

    @Override
    public void runOpMode() {
        initHardware();

        for (int i = 0; i < SLOTS; i++) {
            ballSlots[i] = new BallSlot();
        }

        telemetry.addData("Status", "Initialized");
        if (ENABLE_MOTIF_DETECTION) {
            telemetry.addData("Motif Detection", "ENABLED - Press X to detect");
        } else {
            telemetry.addData("Motif", "%s-%s-%s (Hardcoded)", motif[0], motif[1], motif[2]);
        }
        telemetry.addData("Controls", "Circle: Intake | Square: Shoot | Triangle: Align");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- MOTIF DETECTION (X BUTTON) - Only if enabled ---
            if (ENABLE_MOTIF_DETECTION && gamepad1.cross && !crossPressed && !motifDetected) {
                detectMotif();
                crossPressed = true;
            } else if (!gamepad1.cross) {
                crossPressed = false;
            }

            // --- INTAKE SPEED ADJUSTMENT (D-PAD) ---
            if (gamepad1.dpad_down && !dpadDownPressed) {
                intakeSpeed += 0.05;
                if (intakeSpeed > 0.0)
                    intakeSpeed = 0.0;
                if (isIntakeRunning)
                    intakeMotor.setPower(intakeSpeed);
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            if (gamepad1.dpad_up && !dpadUpPressed) {
                intakeSpeed -= 0.05;
                if (intakeSpeed < -1.0)
                    intakeSpeed = -1.0;
                if (isIntakeRunning)
                    intakeMotor.setPower(intakeSpeed);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            // --- TOGGLE INTAKE (CIRCLE BUTTON) ---
            if (gamepad1.circle && !circlePressed && !isAutoIntaking && !isShooting) {
                if (ballsLoaded < 3) {
                    intakeSequence();
                }
                circlePressed = true;
            } else if (!gamepad1.circle) {
                circlePressed = false;
            }

            // --- AUTO-ALIGN (TRIANGLE BUTTON) ---
            if (gamepad1.triangle && !trianglePressed && !isAligning && !isShooting) {
                autoAlignToGoal();
                trianglePressed = true;
            } else if (!gamepad1.triangle) {
                trianglePressed = false;
            }

            // --- AUTO SHOOT (SQUARE BUTTON) ---
            if (gamepad1.square && !squarePressed && !isShooting && motifDetected) {
                shootSequence();
                squarePressed = true;
            } else if (!gamepad1.square) {
                squarePressed = false;
            }

            // --- MANUAL SPINDEXER ROTATION (D-PAD LEFT/RIGHT) ---
            if (gamepad1.dpad_left && !isAutoIntaking && !isShooting) {
                spindexerServo.setPower(0.3); // Positive for counter-clockwise
            } else if (gamepad1.dpad_right && !isAutoIntaking && !isShooting) {
                spindexerServo.setPower(-0.3); // Negative for clockwise
            } else {
                if (!isAutoIntaking && !isShooting)
                    spindexerServo.setPower(0);
            }

            // --- MANUAL DRIVE (when not aligning) ---
            if (!isAligning && !isShooting && !isAutoIntaking) {
                manualDrive();
            }

            // --- TELEMETRY ---
            telemetry.addData("Mode",
                    isShooting ? "SHOOTING" : (isAligning ? "ALIGNING" : (isAutoIntaking ? "INTAKING" : "READY")));
            telemetry.addData("Current Slot", currentSlot);
            telemetry.addData("Balls Loaded", "%d / %d", ballsLoaded, SLOTS);
            telemetry.addData("Intake Speed", "%.0f%%", Math.abs(intakeSpeed) * 100);
            telemetry.addData("Shooter", shooterMotor.getPower() > 0 ? "SPINNING" : "STOPPED");
            telemetry.addData("Aligned", isAligned ? "YES" : "NO");
            if (ENABLE_MOTIF_DETECTION) {
                telemetry.addData("Motif Detected", motifDetected ? "YES" : "NO - Press X");
            }
            telemetry.addData("Motif", "%s-%s-%s", motif[0], motif[1], motif[2]);

            // Color sensor debugging
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha();
            float[] hsvValues = new float[3];
            Color.RGBToHSV(red * 8, green * 8, blue * 8, hsvValues);
            telemetry.addData("Color", "R:%d G:%d B:%d A:%d", red, green, blue, alpha);
            telemetry.addData("Hue", "%.1f", hsvValues[0]);

            telemetry.addLine("\nLoaded Balls:");
            for (int i = 0; i < SLOTS; i++) {
                String status = ballSlots[i].isEmpty ? "[Empty]" : "[" + ballSlots[i].color + "]";
                if (i == currentSlot)
                    status += " <--";
                telemetry.addData("Slot " + i, status);
            }
            telemetry.update();
        }

        visionPortal.close();
    }

    private void initHardware() {
        // Drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right");
        leftBack = hardwareMap.get(DcMotorEx.class, "back_left");
        rightBack = hardwareMap.get(DcMotorEx.class, "back_right");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Initialize PID controllers
        flPID = new PIDController(kP, kI, kD, kF);
        frPID = new PIDController(kP, kI, kD, kF);
        blPID = new PIDController(kP, kI, kD, kF);
        brPID = new PIDController(kP, kI, kD, kF);

        // Intake motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        // Shooter motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0);

        // Spindexer servo (CRServo)
        spindexerServo = hardwareMap.get(CRServo.class, "spindexer_servo");
        spindexerServo.setPower(0);

        // Lift servo
        liftServo = hardwareMap.get(Servo.class, "lift_servo");
        liftServo.setPosition(LIFT_DOWN);

        // Color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorSensor.enableLed(true);

        // Vision
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    // Detect motif from AprilTag (21, 22, or 23)
    private void detectMotif() {
        telemetry.addData("Status", "Detecting motif from AprilTag...");
        telemetry.update();

        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.size() > 0) {
            AprilTagDetection detection = detections.get(0);
            int tagId = detection.id;

            if (tagId == 21) {
                motif[0] = "G";
                motif[1] = "P";
                motif[2] = "P";
                telemetry.addData("Motif", "G-P-P (Tag 21)");
            } else if (tagId == 22) {
                motif[0] = "P";
                motif[1] = "G";
                motif[2] = "P";
                telemetry.addData("Motif", "P-G-P (Tag 22)");
            } else if (tagId == 23) {
                motif[0] = "P";
                motif[1] = "P";
                motif[2] = "G";
                telemetry.addData("Motif", "P-P-G (Tag 23)");
            } else {
                telemetry.addData("Error", "Unknown AprilTag ID: " + tagId);
                telemetry.update();
                sleep(1000);
                return;
            }

            motifDetected = true;
            telemetry.addData("Status", "Motif detected successfully!");
            telemetry.update();
            sleep(2000);
        } else {
            telemetry.addData("Error", "No AprilTag detected");
            telemetry.update();
            sleep(1000);
        }
    }

    // Intake sequence using spindexer method
    private void intakeSequence() {
        isAutoIntaking = true;
        intakeMotor.setPower(intakeSpeed);
        isIntakeRunning = true;

        while (ballsLoaded < 3 && opModeIsActive()) {
            // Find next empty slot
            int targetSlot = -1;
            int slotsToMove = 0;

            for (int i = 0; i < 3; i++) {
                int checkIndex = (currentSlot + i) % 3;
                if (ballSlots[checkIndex].isEmpty) {
                    targetSlot = checkIndex;
                    slotsToMove = i;
                    break;
                }
            }
            if (targetSlot == -1)
                break;

            // Rotate to empty slot if needed
            if (slotsToMove > 0) {
                spindexerServo.setPower(SPIN_POWER);
                long spinTime = (long) (slotsToMove * ROTATION_TIME_120);
                sleep(spinTime);
                spindexerServo.setPower(0);
                currentSlot = targetSlot;
                sleep(200);
            }

            // Wait for ramp to clear
            if (!waitUntilClear()) {
                intakeMotor.setPower(0);
                isIntakeRunning = false;
                isAutoIntaking = false;
                return;
            }

            // Wait for ball and detect color
            if (!waitForBall(targetSlot)) {
                intakeMotor.setPower(0);
                isIntakeRunning = false;
                isAutoIntaking = false;
                return;
            }
        }

        intakeMotor.setPower(0);
        isIntakeRunning = false;
        isAutoIntaking = false;
    }

    private boolean waitUntilClear() {
        while (opModeIsActive()) {
            int alpha = colorSensor.alpha();
            if (alpha <= ALPHA_THRESHOLD) {
                return true; // Ramp is clear
            }
            if (gamepad1.square)
                return false; // Cancel on square
        }
        return true;
    }

    private boolean waitForBall(int slotIndex) {
        while (opModeIsActive()) {
            String color = checkColorSensor();

            if (!color.equals("UNKNOWN") && !color.equals("NONE")) {
                ballSlots[slotIndex].color = color;
                ballSlots[slotIndex].isEmpty = false;
                ballsLoaded++;

                // Ball detected: Return immediately so the main sequence can rotate to the next
                // slot
                return true;
            }

            if (gamepad1.square)
                return false; // Cancel on square
        }
        return false;
    }

    private String checkColorSensor() {
        // Get RGB values
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int alpha = colorSensor.alpha(); // Luminance/Brightness

        // Check if object is present
        if (alpha <= ALPHA_THRESHOLD) {
            return "NONE";
        }

        // Convert to HSV for better color distinction
        float[] hsvValues = new float[3];
        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsvValues);
        float hue = hsvValues[0];

        // Detect color based on hue and RGB ratios (Relaxed Range)
        // Purple: Hue > 80 (was 100) & Green > 0.9*Red & Blue > 0.9*Red
        // Green: Green > 0.9*Blue & Green > 80 (was 100)

        if (hue > 80 && green > (red * 0.9) && blue > (red * 0.9)) {
            return "P";
        } else if (green > (blue * 0.9) && green > 80) {
            return "G";
        }

        return "UNKNOWN";
    }

    // Shoot sequence
    private void shootSequence() {
        isShooting = true;

        // Spin up shooter
        telemetry.addData("Status", "SPINNING UP SHOOTER...");
        telemetry.update();
        shooterMotor.setPower(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_TIME);

        // Shoot all balls in motif order
        telemetry.addData("Status", "SHOOTING IN MOTIF ORDER");
        telemetry.update();

        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            String targetColor = motif[i];

            // Find nearest ball with target color
            int slotIndex = -1;
            int dist = 0;
            for (int k = 0; k < 3; k++) {
                int idx = (currentSlot + k) % 3;
                if (ballSlots[idx].color.equals(targetColor) && !ballSlots[idx].isEmpty) {
                    slotIndex = idx;
                    dist = k;
                    break;
                }
            }

            if (slotIndex != -1) {
                // Rotate to ball if needed
                if (dist > 0) {
                    spindexerServo.setPower(SPIN_POWER);
                    long spinTime = (long) (dist * ROTATION_TIME_120);
                    sleep(spinTime);
                    spindexerServo.setPower(0);
                    currentSlot = slotIndex;
                }

                // Lift and shoot
                liftServo.setPosition(LIFT_UP);
                sleep(LIFT_DELAY);
                liftServo.setPosition(LIFT_DOWN);
                sleep(LIFT_DELAY);

                // Mark slot as empty
                ballSlots[slotIndex].isEmpty = true;
                ballSlots[slotIndex].color = "";
            }
        }

        // Stop shooter
        shooterMotor.setPower(0);

        telemetry.addData("Status", "SHOOTING COMPLETE!");
        telemetry.update();
        sleep(1000);

        isShooting = false;
        ballsLoaded = 0;
    }

    // Auto-align to goal
    private void autoAlignToGoal() {
        isAligning = true;
        isAligned = false;

        telemetry.addData("Status", "AUTO-ALIGNING TO GOAL...");
        telemetry.update();

        ElapsedTime timeout = new ElapsedTime();
        int stableCount = 0;

        while (isAligning && opModeIsActive() && timeout.seconds() < 8.0) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection goalTag = null;

            for (AprilTagDetection detection : detections) {
                if (detection.id == GOAL_APRILTAG_ID_1 || detection.id == GOAL_APRILTAG_ID_2) {
                    goalTag = detection;
                    break;
                }
            }

            if (goalTag != null) {
                double yawError = -Math.toDegrees(goalTag.ftcPose.yaw);
                double xError = goalTag.ftcPose.x;
                double distanceError = goalTag.ftcPose.range - TARGET_DISTANCE;

                telemetry.addData("Yaw Error", "%.2f deg", yawError);
                telemetry.addData("X Error", "%.2f cm", xError);
                telemetry.addData("Distance Error", "%.2f cm", distanceError);
                telemetry.update();

                if (Math.abs(yawError) < YAW_TOLERANCE &&
                        Math.abs(xError) < X_TOLERANCE &&
                        Math.abs(distanceError) < DISTANCE_TOLERANCE) {

                    stableCount++;
                    if (stableCount >= 10) {
                        isAligned = true;
                        stopMotors();
                        telemetry.addData("Status", "ALIGNED!");
                        telemetry.update();
                        sleep(500);
                        isAligning = false;
                        return;
                    }
                } else {
                    stableCount = 0;
                }

                double forwardPower = 0, strafePower = 0, turnPower = 0;

                if (Math.abs(yawError) > YAW_TOLERANCE) {
                    turnPower = Math.signum(yawError) * (Math.abs(yawError) > 10 ? ALIGN_SPEED : SLOW_SPEED);
                } else if (Math.abs(xError) > X_TOLERANCE) {
                    strafePower = -Math.signum(xError) * (Math.abs(xError) > 8 ? ALIGN_SPEED : SLOW_SPEED);
                } else if (Math.abs(distanceError) > DISTANCE_TOLERANCE) {
                    forwardPower = Math.signum(distanceError)
                            * (Math.abs(distanceError) > 10 ? ALIGN_SPEED : SLOW_SPEED);
                }

                leftFront.setPower(forwardPower - strafePower - turnPower);
                rightFront.setPower(forwardPower + strafePower + turnPower);
                leftBack.setPower(forwardPower + strafePower - turnPower);
                rightBack.setPower(forwardPower - strafePower + turnPower);
            } else {
                stopMotors();
                stableCount = 0;
                telemetry.addData("Status", "Searching for goal tag...");
                telemetry.update();
            }

            sleep(50);
        }

        if (timeout.seconds() >= 8.0) {
            telemetry.addData("Status", "Alignment timeout");
            telemetry.update();
            sleep(1000);
        }

        stopMotors();
        isAligning = false;
    }

    // Manual drive with PID
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

    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
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
