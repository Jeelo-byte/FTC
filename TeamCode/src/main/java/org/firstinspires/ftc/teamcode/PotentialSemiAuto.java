package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Potential Semi-Auto", group = "Modular")
public class PotentialSemiAuto extends LinearOpMode {

    // Subsystems
    private MecanumDriveSubsystem drive;
    private SpindexerSubsystem spindexer;
    private ShooterLiftSubsystem shooterLift;

    // State for Debouncing
    boolean lastSquare = false;
    boolean lastCircle = false;
    boolean lastTriangle = false;
    boolean lastCross = false;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    // Calibration Debounce
    boolean lastRightStickBtn = false;
    boolean lastRightBumper = false;
    boolean lastLeftBumper = false;
    boolean isEditingUp = true;

    @Override
    public void runOpMode() {
        // Init Subsystems
        drive = new MecanumDriveSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        shooterLift = new ShooterLiftSubsystem(hardwareMap);

        telemetry.addData("Status", "Ready - Potential Semi-Auto");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 0. Drive ---
            drive.manualDrive(gamepad1);

            // --- 1. Subsystem Actions (passed Drive capability) ---
            Runnable driveAction = () -> drive.manualDrive(gamepad1);

            // --- 2. Auto Intake (Circle) ---
            if (gamepad1.circle && !lastCircle) {
                // If not already intaking, start sequence
                if (!spindexer.isIntaking()) {
                    spindexer.startIntakeSequence(telemetry, driveAction, opModeIsActive());
                } else {
                    // If already intaking, the loop inside startIntakeSequence checks
                    // for isIntaking flag? Actually the loop blocks.
                    // So we can't 'cancel' easily unless we check inside the loop.
                    // The subsystem checks for Circle to cancel.
                }
            }
            lastCircle = gamepad1.circle;

            // --- 3. Auto Shoot (Cross) ---
            if (gamepad1.cross && !lastCross) {
                shooterLift.shootMacro(spindexer, driveAction, opModeIsActive());
            }
            lastCross = gamepad1.cross;

            // --- 4. Toggles ---

            // Shooter (Square)
            if (gamepad1.square && !lastSquare) {
                shooterLift.toggleShooter();
            }
            lastSquare = gamepad1.square;

            // Lift (Triangle)
            if (gamepad1.triangle && !lastTriangle) {
                shooterLift.toggleLift();
            }
            lastTriangle = gamepad1.triangle;

            // --- 5. Manual Spindexer (D-Pad L/R) ---
            // --- 5. Tuning (D-Pad) & Manual Spindexer (L-Stick + D-Pad) ---

            // F Tuning (Up/Down)
            if (gamepad1.dpad_up && !lastDpadUp) {
                shooterLift.adjustF(0.5);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                shooterLift.adjustF(-0.5);
            }
            lastDpadDown = gamepad1.dpad_down;

            // D-Pad Left/Right Logic
            if (gamepad1.left_stick_button) {
                // Modifier Active: Manual Spindexer
                if (gamepad1.dpad_left && !lastDpadLeft) {
                    spindexer.setSpindexerPower(0.3);
                    sleepWithDrive(380, driveAction);
                    spindexer.setSpindexerPower(0);
                }

                if (gamepad1.dpad_right && !lastDpadRight) {
                    spindexer.setSpindexerPower(-0.3);
                    sleepWithDrive(190, driveAction); // 60 deg?
                    spindexer.setSpindexerPower(0);
                }
            } else {
                // Modifier Inactive: P Tuning
                if (gamepad1.dpad_right && !lastDpadRight) {
                    shooterLift.adjustP(1.0);
                }
                if (gamepad1.dpad_left && !lastDpadLeft) {
                    shooterLift.adjustP(-1.0);
                }
            }
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // --- 6. Calibration (Right Stick + Bumpers) ---

            // Toggle Edit Mode - REMOVED (Only one duration to edit now)
            /*
             * if (gamepad1.right_stick_button && !lastRightStickBtn) {
             * isEditingUp = !isEditingUp;
             * }
             * lastRightStickBtn = gamepad1.right_stick_button;
             */

            // Adjust
            if (gamepad1.right_bumper && !lastRightBumper) {
                shooterLift.adjustCalibration(0.5); // +0.5 ms/deg
            }
            lastRightBumper = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !lastLeftBumper) {
                shooterLift.adjustCalibration(-0.5); // -0.5 ms/deg
            }
            lastLeftBumper = gamepad1.left_bumper;

            // --- Telemetry ---
            telemetry.addData("Shooter", shooterLift.isShooterOn() ? "ON" : "OFF");
            telemetry.addData("Velocity", "%.0f / %.0f", shooterLift.getVelocity(), shooterLift.getTargetVelocity());
            telemetry.addData("PIDF", "P:%.1f F:%.1f", shooterLift.getP(), shooterLift.getF());
            telemetry.addData("Lift", shooterLift.isLiftUp() ? "UP" : "DOWN");
            telemetry.addData("Balls", spindexer.getBallsLoaded() + "/3");

            telemetry.addLine("\n--- Calibration ---");
            telemetry.addData("Calib (ms/deg)", "%.2f", shooterLift.getMsPerDegree());
            telemetry.addData("Cur Degrees", "%.1f", shooterLift.getCurrentDegrees());
            // telemetry.addData("DOWN Pos", "%.2f", shooterLift.getLiftDownPos());

            telemetry.update();
        }

        drive.stop();
    }

    private void sleepWithDrive(long ms, Runnable driveAction) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start < ms)) {
            driveAction.run();
        }
    }
}
