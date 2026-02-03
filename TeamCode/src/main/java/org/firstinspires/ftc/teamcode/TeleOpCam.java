package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Potential Semi-Auto W/ Cam", group = "Modular")
public class TeleOpCam extends LinearOpMode {

    // Subsystems
    private MecanumDriveSubsystem drive;
    private SpindexerSubsystem spindexer;
    private ShooterLiftSubsystem shooterLift;

    // Vision / AprilTag
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean tagFound = false;
    private String patternValue = "Scanning...";

    // Debounce state
    private boolean lastSquare = false;
    private boolean lastCircle = false;
    private boolean lastTriangle = false;
    private boolean lastCross = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Calibration debounce
    private boolean lastRightStickUp = false;
    private boolean lastRightStickDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {

        drive = new MecanumDriveSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        shooterLift = new ShooterLiftSubsystem(hardwareMap);

        // --- Initialize Vision (AprilTag) ---
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the VisionPortal for "Webcam 1"
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        telemetry.addData("Status", "Ready - Potential Semi-Auto");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- AprilTag Detection Logic ---

            // Get detections EVERY loop so we can track 20/24 continuously
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {

                    // ----------------------------------------
                    // NEW: Logic for Tag 20 and 24
                    // ----------------------------------------
                    if (detection.id == 20 || detection.id == 24) {
                        double range = detection.ftcPose.range; // Distance in inches

                        // Check if distance is no higher than 100 inches
                        if (range <= 100) {
                            double ratio = range / 100.0;

                            telemetry.addData("\n--- Target Found ---", "");
                            telemetry.addData("ID Scanned", detection.id);
                            // Formatted to 2 decimal places
                            telemetry.addData("Pos (XYZ)", "%.1f, %.1f, %.1f",
                                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                            telemetry.addData("Ratio (Range/100)", "%.2f", ratio);
                        }
                    }

                    // ----------------------------------------
                    // Existing Logic for Pattern (21, 22, 23)
                    // ----------------------------------------
                    // Only update pattern if we haven't locked one in yet
                    if (!tagFound) {
                        if (detection.id == 21) {
                            patternValue = "Green Purple Purple";
                            tagFound = true;
                        } else if (detection.id == 22) {
                            patternValue = "Purple Green Purple";
                            tagFound = true;
                        } else if (detection.id == 23) {
                            patternValue = "Purple Purple Green";
                            tagFound = true;
                        }
                    }
                }
            }

            // --- Drive (always) ---
            drive.manualDrive(gamepad1);

            // Let subsystems keep drive responsive during sleeps/moves
            Runnable driveAction = () -> drive.manualDrive(gamepad1);

            // --- Auto Intake Toggle (Circle) ---
            if (gamepad1.circle && !lastCircle) {
                spindexer.toggleAutoIntake();
            }
            lastCircle = gamepad1.circle;

            // --- Auto Shoot (Cross) ---
            if (gamepad1.cross && !lastCross) {
                if (spindexer.isAutoIntakeEnabled()) {
                    spindexer.stopAutoIntake();
                }
                shooterLift.shootMacro(spindexer, driveAction, opModeIsActive());
            }
            lastCross = gamepad1.cross;

            // --- Shooter Toggle (Square) ---
            if (gamepad1.square && !lastSquare) {
                shooterLift.toggleShooter();
            }
            lastSquare = gamepad1.square;

            // --- Lift Toggle (Triangle) ---
            if (gamepad1.triangle && !lastTriangle) {
                shooterLift.toggleLift();
            }
            lastTriangle = gamepad1.triangle;

            // --- F tuning (D-pad up/down) ---
            if (gamepad1.dpad_up && !lastDpadUp) {
                shooterLift.adjustF(0.5);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                shooterLift.adjustF(-0.5);
            }
            lastDpadDown = gamepad1.dpad_down;

            // --- Manual spindexer (D-pad left/right) ---
            if (gamepad1.dpad_left && !lastDpadLeft) {
                spindexer.indexOneSlot(driveAction, opModeIsActive());
            }
            if (gamepad1.dpad_right && !lastDpadRight) {
                spindexer.rotateToShooter(driveAction,opModeIsActive());
            }
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // --- Calibration (right stick Y) ---
            boolean rightStickUp = gamepad1.right_stick_y < -0.5;
            if (rightStickUp && !lastRightStickUp) {
                shooterLift.adjustCalibration(0.5);
            }
            lastRightStickUp = rightStickUp;

            boolean rightStickDown = gamepad1.right_stick_y > 0.5;
            if (rightStickDown && !lastRightStickDown) {
                shooterLift.adjustCalibration(-0.5);
            }
            lastRightStickDown = rightStickDown;

            // --- Calibration: Shoot Angle Offset (Bumpers) ---
            if (gamepad1.left_bumper && !lastLeftBumper) {
                shooterLift.adjustInitialShootAngleOffset(-0.5);
            }
            lastLeftBumper = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !lastRightBumper) {
                shooterLift.adjustInitialShootAngleOffset(0.5);
            }
            lastRightBumper = gamepad1.right_bumper;

            // --- Telemetry (Shooter/Lift/Vision) ---
            telemetry.addData("Pattern", patternValue);
            telemetry.addData("Shooter", shooterLift.isShooterOn() ? "ON" : "OFF");
            telemetry.addData("Velocity", "%.0f / %.0f", shooterLift.getVelocity(), shooterLift.getTargetVelocity());
            telemetry.addData("PIDF", "P:%.1f F:%.1f", shooterLift.getP(), shooterLift.getF());
            telemetry.addData("Lift", shooterLift.isLiftUp() ? "UP" : "DOWN");

            telemetry.addLine("\n--- Calibration ---");
            telemetry.addData("Calib (ms/deg)", "%.2f", shooterLift.getMsPerDegree());
            telemetry.addData("Shoot Offset", "%.1f", shooterLift.getInitialShootAngleOffset());
            telemetry.addData("Cur Degrees", "%.1f", shooterLift.getCurrentDegrees());

            // --- REQUIRED: run auto-intake state machine every loop ---
            spindexer.updateAutoIntake(telemetry, driveAction, opModeIsActive());

            telemetry.update();
        }

        // Clean up vision portal when opmode stops
        if (visionPortal != null) {
            visionPortal.close();
        }

        drive.stop();
    }
}
