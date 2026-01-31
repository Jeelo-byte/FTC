package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Potential Semi-Auto", group = "Modular")
public class PotentialSemiAuto extends LinearOpMode {

    // Subsystems
    private MecanumDriveSubsystem drive;
    private SpindexerSubsystem spindexer;
    private ShooterLiftSubsystem shooterLift;

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

        telemetry.addData("Status", "Ready - Potential Semi-Auto");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Drive (always) ---
            drive.manualDrive(gamepad1);

            // Let subsystems keep drive responsive during sleeps/moves
            Runnable driveAction = () -> drive.manualDrive(gamepad1);

            // --- Auto Intake Toggle (Circle) ---
            // Press once: ON. Press again: OFF. Auto-OFF when 3 balls loaded.
            if (gamepad1.circle && !lastCircle) {
                spindexer.toggleAutoIntake();
            }
            lastCircle = gamepad1.circle;

            // --- Auto Shoot (Cross) ---
            if (gamepad1.cross && !lastCross) {
                // Stop intake so it doesn't fight auto-shoot
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
                spindexer.moveSpindexer(120.0, driveAction, opModeIsActive());
            }
            if (gamepad1.dpad_right && !lastDpadRight) {
                spindexer.moveSpindexer(60.0, driveAction, opModeIsActive());
            }
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // --- Calibration (right stick Y) ---
            boolean rightStickUp = gamepad1.right_stick_y < -0.5;
            if (rightStickUp && !lastRightStickUp) {
                spindexer.adjustCalibration(0.5);
            }
            lastRightStickUp = rightStickUp;

            boolean rightStickDown = gamepad1.right_stick_y > 0.5;
            if (rightStickDown && !lastRightStickDown) {
                spindexer.adjustCalibration(-0.5);
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

            // --- Telemetry (Shooter/Lift) ---
            telemetry.addData("Shooter", shooterLift.isShooterOn() ? "ON" : "OFF");
            telemetry.addData("Velocity", "%.0f / %.0f", shooterLift.getVelocity(), shooterLift.getTargetVelocity());
            telemetry.addData("PIDF", "P:%.1f F:%.1f", shooterLift.getP(), shooterLift.getF());
            telemetry.addData("Lift", shooterLift.isLiftUp() ? "UP" : "DOWN");

            telemetry.addLine("\n--- Calibration ---");
            telemetry.addData("Calib (ms/deg)", "%.2f", spindexer.getMsPerDegree());
            telemetry.addData("Shoot Offset", "%.1f", shooterLift.getInitialShootAngleOffset());
            telemetry.addData("Cur Degrees", "%.1f", spindexer.getCurrentDegrees());

            // --- REQUIRED: run auto-intake state machine every loop (non-blocking) ---
            spindexer.updateAutoIntake(telemetry, driveAction, opModeIsActive());

            telemetry.update();
        }

        drive.stop();
    }
}
