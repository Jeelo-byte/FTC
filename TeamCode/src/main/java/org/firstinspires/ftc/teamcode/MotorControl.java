package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Motor Control", group="Linear Opmode")
public class MotorControl extends LinearOpMode {

    private DcMotorEx motor;
    private Servo spindexerServo;
    private ColorSensor colorSensor;

    private double targetSpeedPercent = 50.0;
    private double maxRPM = 6000.0;

    private static final double TICKS_PER_REV = 28.0;

    // Debouncing booleans
    private boolean dpadDownPressed = false;
    private boolean dpadUpPressed = false;
    private boolean squarePressed = false;

    // Spindexer constants
    private static final int SLOTS = 3; // 3 ball slots
    private static final double SLOT_ANGLE = 120.0; // 120 degrees per slot
    private static final double GEAR_RATIO = 1.0; // 15:15 gear ratio = 1:1
    private static final double DEGREES_TO_SERVO = 1.0 / 360.0; // Convert degrees to servo position
    private static final int COLOR_THRESHOLD = 50; // Threshold for color detection
    private static final long DETECTION_DELAY = 500; // 0.5 seconds delay before rotation

    // Ball storage
    private String[] ballSlots = new String[SLOTS]; // Stores "G", "P", or "EMPTY"
    private int currentSlot = 0; // Current slot position (0, 1, 2)
    private int ballCount = 0;

    // Motif - TODO: Replace with actual detection later
    private String[] motif = {"P", "G", "G"}; // Example: Purple, Green, Green

    // Auto-intake control
    private ElapsedTime detectionTimer = new ElapsedTime();
    private boolean ballDetected = false;
    private boolean isAutoIntaking = true; // Auto-intake enabled by default

    @Override
    public void runOpMode() {
        // Initialize hardware
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spindexerServo = hardwareMap.get(Servo.class, "spindexer_servo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Initialize ball slots as empty
        for (int i = 0; i < SLOTS; i++) {
            ballSlots[i] = "EMPTY";
        }

        // Set initial servo position
        rotateToSlot(0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Motif", "%s-%s-%s", motif[0], motif[1], motif[2]);
        telemetry.addData("Controls", "Square: Auto Shoot | L-Trigger: Motor");
        telemetry.update();

        waitForStart();

        detectionTimer.reset();

        while (opModeIsActive()) {
            // --- MOTOR SPEED ADJUSTMENT (DPAD) ---
            if (gamepad1.dpad_down && !dpadDownPressed) {
                targetSpeedPercent -= 5.0;
                if (targetSpeedPercent < 0) targetSpeedPercent = 0;
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            if (gamepad1.dpad_up && !dpadUpPressed) {
                targetSpeedPercent += 5.0;
                if (targetSpeedPercent > 100) targetSpeedPercent = 100;
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            // --- AUTO COLOR DETECTION AND INDEXING ---
            if (isAutoIntaking && ballCount < SLOTS) {
                int red = colorSensor.red();
                int green = colorSensor.green();
                int blue = colorSensor.blue();

                // Check if a ball is detected
                boolean isBallPresent = (red > 100 || green > 100 || blue > 100);

                if (isBallPresent && !ballDetected) {
                    // Ball just detected, start timer
                    ballDetected = true;
                    detectionTimer.reset();

                    // Determine color
                    String detectedColor = detectBallColor(red, green, blue);

                    telemetry.addData("Ball Detected!", detectedColor);
                    telemetry.addData("RGB", "R=%d G=%d B=%d", red, green, blue);
                    telemetry.update();
                }

                // After 0.5 seconds, rotate to next slot
                if (ballDetected && detectionTimer.milliseconds() >= DETECTION_DELAY) {
                    String detectedColor = detectBallColor(
                            colorSensor.red(),
                            colorSensor.green(),
                            colorSensor.blue()
                    );

                    // Save ball color in current slot
                    ballSlots[currentSlot] = detectedColor;
                    ballCount++;

                    telemetry.addData("Stored", "Slot %d: %s", currentSlot, detectedColor);
                    telemetry.update();

                    // Rotate to next slot (120 degrees clockwise)
                    currentSlot = (currentSlot + 1) % SLOTS;
                    rotateToSlot(currentSlot);

                    sleep(300); // Wait for servo to move

                    // Reset detection
                    ballDetected = false;
                }
            }

            // --- AUTO SHOOT SEQUENCE (SQUARE BUTTON) ---
            if (gamepad1.square && !squarePressed) {
                autoShootMotif();
                squarePressed = true;
            } else if (!gamepad1.square) {
                squarePressed = false;
            }

            // --- MOTOR POWER CONTROL (LEFT TRIGGER) ---
            if (gamepad1.left_trigger > 0.1) {
                double targetRPM = (targetSpeedPercent / 100.0) * maxRPM;
                double targetVelocity = (targetRPM / 60.0) * TICKS_PER_REV;
                motor.setVelocity(targetVelocity);
            } else {
                motor.setPower(0);
            }

            // Calculate actual RPM for telemetry
            double actualRPM = (motor.getVelocity() / TICKS_PER_REV) * 60.0;

            // --- TELEMETRY ---
            telemetry.addData("Mode", isAutoIntaking ? "AUTO INTAKE" : "READY TO SHOOT");
            telemetry.addData("Current Slot", currentSlot);
            telemetry.addData("Balls Loaded", "%d / %d", ballCount, SLOTS);
            telemetry.addData("Motif", "%s-%s-%s", motif[0], motif[1], motif[2]);
            telemetry.addLine("\nLoaded Balls:");
            for (int i = 0; i < SLOTS; i++) {
                telemetry.addData("Slot " + i, ballSlots[i]);
            }
            telemetry.addData("\nMotor Target Speed", "%.0f%%", targetSpeedPercent);
            telemetry.addData("Motor Actual RPM", "%.0f", actualRPM);
            telemetry.update();
        }
    }

    // Detect ball color from RGB values
    private String detectBallColor(int red, int green, int blue) {
        if (green > red + COLOR_THRESHOLD && green > blue + COLOR_THRESHOLD) {
            return "G"; // Green ball
        } else if ((red > green && blue > green) || (red + blue > green * 2)) {
            return "P"; // Purple ball
        } else {
            return "UNKNOWN";
        }
    }

    // Rotate spindexer to specific slot (0, 1, 2)
    private void rotateToSlot(int slot) {
        // Calculate servo position based on slot
        // Each slot is 120 degrees apart
        double angle = slot * SLOT_ANGLE * GEAR_RATIO;
        double servoPosition = (angle * DEGREES_TO_SERVO);

        // Ensure position is within 0-1 range
        servoPosition = servoPosition % 1.0;

        spindexerServo.setPosition(servoPosition);
    }

    // Auto shoot based on motif order
    private void autoShootMotif() {
        isAutoIntaking = false; // Stop auto-intaking during shooting

        telemetry.addData("Status", "AUTO SHOOTING BASED ON MOTIF");
        telemetry.update();

        for (int motifIndex = 0; motifIndex < motif.length; motifIndex++) {
            String targetColor = motif[motifIndex];

            telemetry.addData("Shooting Position", "%d: %s", motifIndex + 1, targetColor);
            telemetry.update();

            // Find a ball matching the target color
            int ballSlot = findBallWithColor(targetColor);

            if (ballSlot == -1) {
                // No matching ball found, shoot any available ball
                ballSlot = findAnyBall();

                if (ballSlot == -1) {
                    telemetry.addData("Warning", "No balls left to shoot");
                    telemetry.update();
                    sleep(500);
                    continue; // Skip this position
                }
            }

            // Rotate to shooting position (180 degrees from intake = opposite side)
            int shootSlot = (ballSlot + SLOTS / 2) % SLOTS; // Add 1.5 slots (180°)
            // Since we have 3 slots, opposite is actually slot + 1 or 2 depending on position
            // For 3 slots: 0->1, 1->2, 2->0 (but we need 180°)
            // Actually, for 120° slots, 180° = 1.5 slots, so we use direct calculation

            // Move to the ball's shooting position (180° from current)
            rotateToShootPosition(ballSlot);
            sleep(500); // Wait for rotation

            // TODO: Activate shooter motor here
            sleep(1000); // Simulate shooting time

            // Mark ball as used
            ballSlots[ballSlot] = "EMPTY";
            ballCount--;

            telemetry.addData("Shot", "Slot %d (%s)", ballSlot, targetColor);
            telemetry.update();
        }

        telemetry.addData("Status", "SHOOTING COMPLETE!");
        telemetry.update();
        sleep(1000);

        // Return to intake position
        currentSlot = 0;
        rotateToSlot(currentSlot);
        isAutoIntaking = true; // Resume auto-intaking
    }

    // Find a ball matching the target color
    private int findBallWithColor(String color) {
        for (int i = 0; i < SLOTS; i++) {
            if (ballSlots[i].equals(color)) {
                return i;
            }
        }
        return -1; // Not found
    }

    // Find any available ball
    private int findAnyBall() {
        for (int i = 0; i < SLOTS; i++) {
            if (!ballSlots[i].equals("EMPTY")) {
                return i;
            }
        }
        return -1; // No balls available
    }

    // Rotate to shooting position (180 degrees from intake position)
    private void rotateToShootPosition(int intakeSlot) {
        // Shooting position is 180° opposite from intake
        // For 120° slots: 180° = 1.5 slots
        // Calculate the shooting angle
        double intakeAngle = intakeSlot * SLOT_ANGLE;
        double shootAngle = (intakeAngle + 180.0) % 360.0;

        double servoPosition = (shootAngle * GEAR_RATIO * DEGREES_TO_SERVO) % 1.0;
        spindexerServo.setPosition(servoPosition);
    }
}