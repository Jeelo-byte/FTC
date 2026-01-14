package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Spindexer Control")
public class SpindexerControl extends LinearOpMode {

    private Servo spindexerServo;
    private Servo liftServo;
    private DistanceSensor intakeSensor;
    private ColorSensor rampSensor;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int SPINDEXER_SLOTS = 3;
    private static final double SLOT_ANGLE = 360.0 / SPINDEXER_SLOTS;
    private static final double GEAR_RATIO =0.2;
    private static final double LIFT_UP = 0.7;
    private static final double LIFT_DOWN = 0.1;
    private static final double DETECTION_DISTANCE = 5.0;

    private static final boolean USE_ULTRA_SPEED = false;

    private static final int ROTATION_DELAY = USE_ULTRA_SPEED ? 100 : 400;
    private static final int LIFT_DELAY = USE_ULTRA_SPEED ? 150 : 500;
    private static final int EJECT_DELAY = 300;
    private static final int LOWER_DELAY = USE_ULTRA_SPEED ? 100 : 400;

    private static final int COLOR_THRESHOLD = 50;

    private BallSlot[] ballSlots = new BallSlot[SPINDEXER_SLOTS];
    private String[] motif = new String[3];
    private int currentSlot = 0;
    private int ballsLoaded = 0;
    private boolean isIntaking = false;
    private boolean isShooting = false;
    private boolean motifDetected = false;

    class BallSlot {
        String color;
        boolean isEmpty;
        int position;

        BallSlot(int pos) {
            this.isEmpty = true;
            this.position = pos;
            this.color = "";
        }
    }

    @Override
    public void runOpMode() {
        initHardware();

        for (int i = 0; i < SPINDEXER_SLOTS; i++) {
            ballSlots[i] = new BallSlot(i);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Press X", "to detect motif");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && !motifDetected) {
                detectMotif();
            }

            if (gamepad1.a && !isIntaking && !isShooting) {
                intakeBalls();
            }

            if (gamepad1.b && !isShooting && !isIntaking && motifDetected) {
                shootAllBalls();
            }

            if (gamepad1.dpad_left && !isShooting && !isIntaking) {
                rotateToSlot((currentSlot - 1 + SPINDEXER_SLOTS) % SPINDEXER_SLOTS);
                sleep(300);
            }
            if (gamepad1.dpad_right && !isShooting && !isIntaking) {
                rotateToSlot((currentSlot + 1) % SPINDEXER_SLOTS);
                sleep(300);
            }

            updateTelemetry();
        }

        visionPortal.close();
    }

    private void initHardware() {
        spindexerServo = hardwareMap.get(Servo.class, "spindexer_servo");
        liftServo = hardwareMap.get(Servo.class, "lift_servo");

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_sensor");
        rampSensor = hardwareMap.get(ColorSensor.class, "ramp_sensor");

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        liftServo.setPosition(LIFT_DOWN);
        rotateToSlot(0);
    }

    private void detectMotif() {
        telemetry.addData("Status", "Detecting motif from AprilTag...");
        telemetry.update();

        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.size() > 0) {
            AprilTagDetection detection = detections.get(0);
            int tagId = detection.id;

            if (tagId == 21) {
                motif[0] = "G"; motif[1] = "P"; motif[2] = "P";
                telemetry.addData("Motif", "GPP (Tag 21)");
            } else if (tagId == 22) {
                motif[0] = "P"; motif[1] = "G"; motif[2] = "P";
                telemetry.addData("Motif", "PGP (Tag 22)");
            } else if (tagId == 23) {
                motif[0] = "P"; motif[1] = "P"; motif[2] = "G";
                telemetry.addData("Motif", "PPG (Tag 23)");
            } else {
                telemetry.addData("Error", "Unknown AprilTag ID: " + tagId);
                telemetry.update();
                return;
            }

            motifDetected = true;
            telemetry.addData("Status", "Motif detected successfully!");
            telemetry.update();
            sleep(2000);
        } else {
            telemetry.addData("Error", "No AprilTag detected");
            telemetry.update();
        }
    }

    private void intakeBalls() {
        isIntaking = true;
        ballsLoaded = 0;
        int spins = 0;

        telemetry.addData("Status", "Intaking balls...");
        telemetry.update();

        while (spins < 2 && opModeIsActive()) {
            for (int slot = 0; slot < SPINDEXER_SLOTS && opModeIsActive(); slot++) {
                if (ballSlots[slot].isEmpty) {
                    rotateToSlot(slot);
                    sleep(500);

                    long startTime = System.currentTimeMillis();
                    while (System.currentTimeMillis() - startTime < 2000 && opModeIsActive()) {
                        double distance = intakeSensor.getDistance(DistanceUnit.CM);

                        if (distance < DETECTION_DISTANCE) {
                            sleep(300);
                            scanBallColor(slot);
                            ballsLoaded++;
                            break;
                        }
                        sleep(50);
                    }
                }
            }
            spins++;
        }

        isIntaking = false;
        telemetry.addData("Status", "Intake complete");
        telemetry.addData("Balls loaded", ballsLoaded);
        telemetry.update();
    }

    private void scanBallColor(int slot) {
        BallSlot ball = ballSlots[slot];

        int red = rampSensor.red();
        int green = rampSensor.green();
        int blue = rampSensor.blue();

        if (green > red + COLOR_THRESHOLD && green > blue + COLOR_THRESHOLD) {
            ball.color = "G";
        } else if ((red > green && blue > green) || (red + blue > green * 2)) {
            ball.color = "P";
        } else {
            ball.color = "UNKNOWN";
        }

        ball.isEmpty = false;

        telemetry.addData("Scanned Slot", slot);
        telemetry.addData("Color", ball.color + " (R=%d G=%d B=%d)", red, green, blue);
        telemetry.update();
    }

    private void shootAllBalls() {
        if (!motifDetected) {
            telemetry.addData("Error", "Detect motif first (press X)");
            telemetry.update();
            return;
        }

        isShooting = true;
        telemetry.addData("Status", "AUTO SHOOTING IN MOTIF ORDER!");
        telemetry.update();

        for (int motifIndex = 0; motifIndex < 3 && opModeIsActive(); motifIndex++) {
            String targetColor = motif[motifIndex];

            telemetry.addData("Shooting", "Position %d: %s", motifIndex + 1, targetColor);
            telemetry.update();

            int ballSlot = findBallWithColor(targetColor);

            if (ballSlot != -1) {
                shootBallFast(ballSlot);
                sleep(200);
            } else {
                telemetry.addData("Warning", "No %s ball found, skipping", targetColor);
                telemetry.update();
                sleep(500);
            }
        }

        isShooting = false;
        telemetry.addData("Status", "SHOOTING COMPLETE!");
        telemetry.update();
        sleep(1000);
    }

    private int findBallWithColor(String color) {
        for (int i = 0; i < SPINDEXER_SLOTS; i++) {
            if (!ballSlots[i].isEmpty && ballSlots[i].color.equals(color)) {
                return i;
            }
        }
        return -1;
    }

    private void shootBallFast(int slot) {
        int shootSlot = (slot + SPINDEXER_SLOTS / 2) % SPINDEXER_SLOTS;

        rotateToSlot(shootSlot);
        sleep(ROTATION_DELAY);

        liftServo.setPosition(LIFT_UP);
        sleep(LIFT_DELAY);

        sleep(EJECT_DELAY);

        liftServo.setPosition(LIFT_DOWN);
        sleep(LOWER_DELAY);

        ballSlots[slot].isEmpty = true;
        ballSlots[slot].color = "";
        ballsLoaded--;
    }

    private void rotateToSlot(int slot) {
        currentSlot = slot;
        double angle = slot * SLOT_ANGLE * GEAR_RATIO;
        double servoPosition = angle / 360.0;
        spindexerServo.setPosition(servoPosition);
    }

    private void updateTelemetry() {
        telemetry.addData("Status", isShooting ? "SHOOTING" : (isIntaking ? "INTAKING" : "Ready"));
        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Balls Loaded", ballsLoaded);

        if (motifDetected) {
            telemetry.addData("Motif", "%s-%s-%s", motif[0], motif[1], motif[2]);
        } else {
            telemetry.addData("Motif", "Not detected (press X)");
        }

        telemetry.addData("Distance", "%.2f cm", intakeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Ramp RGB", "R=%d G=%d B=%d",
                rampSensor.red(), rampSensor.green(), rampSensor.blue());

        telemetry.addLine("\nControls:");
        telemetry.addData("X", "Detect Motif");
        telemetry.addData("A", "Intake");
        telemetry.addData("B", "Auto Shoot");
        telemetry.addData("DPad", "Manual Rotate");

        telemetry.addLine("\nLoaded Balls:");
        for (int i = 0; i < SPINDEXER_SLOTS; i++) {
            if (!ballSlots[i].isEmpty) {
                telemetry.addData("Slot " + i, ballSlots[i].color);
            }
        }

        telemetry.update();
    }
}

