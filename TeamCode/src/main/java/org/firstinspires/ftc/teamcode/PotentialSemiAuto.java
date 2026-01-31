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
    boolean lastRightStickUp = false;
    boolean lastRightStickDown = false;
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

            // --- 2. Auto Intake Toggle (Circle) ---
            if (gamepad1.circle && !lastCircle) {
                // Now uses toggle logic
                spindexer.toggleIntake(telemetry, driveAction, opModeIsActive());
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

            // D-Pad Left/Right: Manual Spindexer (Always Active)
            if (gamepad1.dpad_left && !lastDpadLeft) {
                // Manual Rotate + (120 deg)
                spindexer.moveSpindexer(120.0, driveAction, opModeIsActive());
            }

            if (gamepad1.dpad_right && !lastDpadRight) {
                // Manual Rotate - (60 deg)
                spindexer.moveSpindexer(60.0, driveAction, opModeIsActive());
            }

            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // --- 6. Calibration (Right Stick Y) ---

            // Stick Up (Negative Y) -> Increase ms/deg (Slower)
            boolean rightStickUp = gamepad1.right_stick_y < -0.5;
            if (rightStickUp && !lastRightStickUp) {
                shooterLift.adjustCalibration(0.5);
            }
            lastRightStickUp = rightStickUp;

            // Stick Down (Positive Y) -> Decrease ms/deg (Faster)
            boolean rightStickDown = gamepad1.right_stick_y > 0.5;
            if (rightStickDown && !lastRightStickDown) {
                shooterLift.adjustCalibration(-0.5);
            }
            lastRightStickDown = rightStickDown;

            // --- Telemetry ---
            telemetry.addData("Shooter", shooterLift.isShooterOn() ? "ON" : "OFF");
            telemetry.addData("Velocity", "%.0f / %.0f", shooterLift.getVelocity(), shooterLift.getTargetVelocity());
            telemetry.addData("PIDF", "P:%.1f F:%.1f", shooterLift.getP(), shooterLift.getF());
            telemetry.addData("Lift", shooterLift.isLiftUp() ? "UP" : "DOWN");
            // Spindexer Telemetry
            //spindexer.printTelemetry(telemetry);

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
