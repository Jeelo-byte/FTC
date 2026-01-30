/*
 * package org.firstinspires.ftc.teamcode;
 * 
 * import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 * import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 * import com.qualcomm.robotcore.hardware.CRServo;
 * import com.qualcomm.robotcore.hardware.DcMotor;
 * import com.qualcomm.robotcore.hardware.DcMotorSimple;
 * import com.qualcomm.robotcore.hardware.Servo;
 * import com.qualcomm.robotcore.util.ElapsedTime;
 * 
 * import com.qualcomm.robotcore.hardware.NormalizedRGBA;
 * import android.graphics.Color;
 * import com.qualcomm.robotcore.hardware.DistanceSensor;
 * import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
 * 
 * @TeleOp(name = "Semi_Auto Motor", group = "Linear Opmode")
 * public class MotorControl extends LinearOpMode {
 * 
 * // --- Hardware ---
 * private CRServo spindexerServo; //
 * private Servo liftServo;
 * private DcMotor shooterMotor;
 * private DcMotor intakeCoreHex;
 * private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;
 * private DistanceSensor distanceSensor;
 * 
 * // --- State Variables ---
 * // Ball Storage & Color Logic
 * private static final int SLOTS = 3;
 * private BallSlot[] ballSlots = new BallSlot[SLOTS];
 * private int currentSlot = 0;
 * private int ballsLoaded = 0;
 * 
 * // Color Constants
 * private static final int PRESENCE_THRESHOLD = 150; // From SpindexerControl
 * 
 * // Drive Motors
 * private com.qualcomm.robotcore.hardware.DcMotorEx leftFront, rightFront,
 * leftBack, rightBack;
 * 
 * // --- Constants ---
 * // PID Constants (From MotorControl2)
 * private static final double kP = 0.0001;
 * private static final double kI = 0.0;
 * private static final double kD = 0.0;
 * private static final double kF = 0.00036;
 * private static final double MAX_TICKS_PER_SEC = 2796;
 * 
 * // PID Controllers for each wheel
 * private PIDController flPID, frPID, blPID, brPID;
 * 
 * // Spindexer Logic (CRServo)
 * // From SpindexerControl
 * private static final int BASE_TIME_PER_SLOT = 380;
 * private static final int SETTLE_DELAY = 1000;
 * private static final double SPIN_POWER = -0.75; // From SpindexerControl
 * 
 * // Lift Constants (Mutable for calibration)
 * // From MotorControl2: 0.7 and 0.1 (Gear Ratio Adjusted)
 * double LIFT_DOWN_POS = 0.7;
 * double LIFT_UP_POS = 0.1;
 * final int LIFT_WAIT_TIME = 300;
 * 
 * // Spindexer Timing Constants
 * private static final int TIME_120 = 380;
 * private static final int TIME_60 = 190;
 * 
 * // --- State Variables ---
 * // Toggles
 * boolean isShooterOn = false;
 * boolean lastSquare = false; // Tracks previous loop state for shooter
 * 
 * boolean isIntakeOn = false;
 * boolean isIntaking = false; // Auto-Intake State
 * boolean lastBButton = false; // Tracks previous loop state for intake
 * 
 * // Lift Toggle
 * boolean isLiftUp = false;
 * boolean lastTriangle = false;
 * 
 * // Calibration State
 * boolean isEditingUp = true; // Start editing UP by default
 * boolean lastRightStickButton = false;
 * boolean lastRightBumper = false;
 * boolean lastLeftBumper = false;
 * 
 * // Lift Logic
 * ElapsedTime liftTimer = new ElapsedTime();
 * String liftState = "IDLE";
 * 
 * // Color Indexing Logic
 * String indexState = "IDLE";
 * ElapsedTime indexTimer = new ElapsedTime();
 * boolean lastCircle = false;
 * 
 * // Auto-Shoot Logic
 * String shootState = "IDLE";
 * ElapsedTime shootTimer = new ElapsedTime();
 * int shotsFired = 0;
 * boolean lastCross = false;
 * 
 * // D-Pad Edge Detection
 * boolean lastDpadLeft = false;
 * boolean lastDpadRight = false;
 * 
 * @Override
 * public void runOpMode() {
 * 
 * // 1. Hardware Map
 * spindexerServo = hardwareMap.get(CRServo.class, "spindexer_servo");
 * liftServo = hardwareMap.get(Servo.class, "lift_servo");
 * shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
 * intakeCoreHex = hardwareMap.get(DcMotor.class, "intake_motor");
 * colorSensor =
 * hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class,
 * "color_sensor");
 * if (colorSensor instanceof DistanceSensor) {
 * distanceSensor = (DistanceSensor) colorSensor;
 * }
 * if (colorSensor instanceof com.qualcomm.robotcore.hardware.SwitchableLight) {
 * ((com.qualcomm.robotcore.hardware.SwitchableLight)
 * colorSensor).enableLight(true);
 * }
 * 
 * // Initialize Ball Slots
 * for (int i = 0; i < SLOTS; i++) {
 * ballSlots[i] = new BallSlot();
 * }
 * 
 * // Drive Motors Map
 * leftFront = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class,
 * "front_left");
 * rightFront = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class,
 * "front_right");
 * leftBack = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class,
 * "back_left");
 * rightBack = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class,
 * "back_right");
 * 
 * // Directions (Matching MotorControl2: Only leftFront reversed)
 * leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
 * // Others default to FORWARD
 * 
 * // Motor Modes (Matching MotorControl2)
 * leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 * rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 * leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 * rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 * 
 * leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * 
 * leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 * rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 * leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 * rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 * 
 * // Initialize PIDs
 * flPID = new PIDController(kP, kI, kD, kF);
 * frPID = new PIDController(kP, kI, kD, kF);
 * blPID = new PIDController(kP, kI, kD, kF);
 * brPID = new PIDController(kP, kI, kD, kF);
 * 
 * // 2. Setup
 * spindexerServo.setPower(0);
 * liftServo.setPosition(LIFT_DOWN_POS);
 * 
 * // Shooter Setup
 * shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
 * shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * 
 * // Intake Setup (Core Hex)
 * intakeCoreHex.setDirection(DcMotorSimple.Direction.REVERSE);
 * intakeCoreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 * intakeCoreHex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 * 
 * telemetry.addData("Status", "Ready - Manual Mode (PID Drive)");
 * telemetry.update();
 * 
 * liftServo.setPosition(LIFT_DOWN_POS);
 * 
 * waitForStart();
 * 
 * while (opModeIsActive()) {
 * 
 * // =========================================================
 * // 0. Calibration Controls (R1/L1 + Stick Button)
 * // =========================================================
 * 
 * // Toggle Edit Mode (Up/Down) - Right Stick Button
 * if (gamepad1.right_stick_button && !lastRightStickButton) {
 * isEditingUp = !isEditingUp;
 * }
 * lastRightStickButton = gamepad1.right_stick_button;
 * 
 * // Adjust Value
 * double delta = 0.01;
 * boolean pressedUp = gamepad1.right_bumper && !lastRightBumper;
 * boolean pressedDown = gamepad1.left_bumper && !lastLeftBumper;
 * 
 * if (pressedUp || pressedDown) {
 * double change = pressedUp ? delta : -delta;
 * 
 * if (isEditingUp) {
 * LIFT_UP_POS += change;
 * // Clip
 * if (LIFT_UP_POS > 1.0)
 * LIFT_UP_POS = 1.0;
 * if (LIFT_UP_POS < 0.0)
 * LIFT_UP_POS = 0.0;
 * } else {
 * LIFT_DOWN_POS += change;
 * // Clip
 * if (LIFT_DOWN_POS > 1.0)
 * LIFT_DOWN_POS = 1.0;
 * if (LIFT_DOWN_POS < 0.0)
 * LIFT_DOWN_POS = 0.0;
 * }
 * }
 * lastRightBumper = gamepad1.right_bumper;
 * lastLeftBumper = gamepad1.left_bumper;
 * 
 * // =========================================================
 * // 1. Spindexer Controls (CRServo Time-Based)
 * // =========================================================
 * 
 * // Only allow manual control if not auto-shooting
 * if (shootState.equals("IDLE")) {
 * // Left D-Pad: 0.3 Power (Positive) - Matches MotorControl2 Left D-Pad
 * if (gamepad1.dpad_left && !lastDpadLeft) {
 * spindexerServo.setPower(0.3);
 * sleep(TIME_120);
 * spindexerServo.setPower(0);
 * }
 * lastDpadLeft = gamepad1.dpad_left;
 * 
 * // Right D-Pad: -0.3 Power (Negative) - Matches MotorControl2 Right D-Pad
 * if (gamepad1.dpad_right && !lastDpadRight) {
 * spindexerServo.setPower(-0.3);
 * sleep(TIME_60);
 * spindexerServo.setPower(0);
 * 
 * if (liftState.equals("IDLE")) {
 * liftState = "GOING_UP";
 * liftTimer.reset();
 * }
 * }
 * lastDpadRight = gamepad1.dpad_right;
 * }
 * 
 * // =========================================================
 * // 2. Shooter Toggle (Square - Toggle Max Power)
 * // =========================================================
 * if (shootState.equals("IDLE")) {
 * if (gamepad1.square && !lastSquare) {
 * isShooterOn = !isShooterOn;
 * }
 * lastSquare = gamepad1.square;
 * shooterMotor.setPower(isShooterOn ? 1.0 : 0.0);
 * }
 * 
 * // =========================================================
 * // 3. Intake Sequence (Circle/B)
 * // =========================================================
 * if (shootState.equals("IDLE") && !isIntaking) {
 * if (gamepad1.circle && !lastCircle) {
 * // Reset if full
 * if (ballsLoaded >= 3) {
 * ballsLoaded = 0;
 * for (BallSlot s : ballSlots)
 * s.isEmpty = true;
 * }
 * intakeSequence();
 * }
 * lastCircle = gamepad1.circle;
 * }
 * 
 * // =========================================================
 * // 4. Lift Logic (Triangle Toggle + Calibration)
 * // =========================================================
 * if (shootState.equals("IDLE")) {
 * if (gamepad1.triangle && !lastTriangle) {
 * isLiftUp = !isLiftUp;
 * liftServo.setPosition(isLiftUp ? LIFT_UP_POS : LIFT_DOWN_POS);
 * }
 * lastTriangle = gamepad1.triangle;
 * }
 * 
 * // =========================================================
 * // 4b. Auto-Shoot Macro (Cross/A)
 * // =========================================================
 * 
 * // Trigger
 * if (gamepad1.cross && !lastCross && shootState.equals("IDLE")) {
 * shootState = "SPINUP";
 * shootTimer.reset();
 * shotsFired = 0;
 * 
 * // Override subsystems
 * isShooterOn = true;
 * shooterMotor.setPower(1.0);
 * isIntakeOn = false;
 * intakeCoreHex.setPower(0);
 * 
 * // Ensure Lift is Down initially?
 * isLiftUp = false;
 * liftServo.setPosition(LIFT_DOWN_POS);
 * }
 * lastCross = gamepad1.cross;
 * 
 * // State Machine
 * if (shootState.equals("SPINUP")) {
 * // Wait 0.75s leeway
 * if (shootTimer.milliseconds() > 750) {
 * // Start shooting cycle
 * shootState = "LIFT_UP";
 * shootTimer.reset();
 * }
 * } else if (shootState.equals("LIFT_UP")) {
 * liftServo.setPosition(LIFT_UP_POS);
 * if (shootTimer.milliseconds() > LIFT_WAIT_TIME) {
 * shootState = "LIFT_DOWN";
 * shootTimer.reset();
 * }
 * } else if (shootState.equals("LIFT_DOWN")) {
 * liftServo.setPosition(LIFT_DOWN_POS);
 * if (shootTimer.milliseconds() > LIFT_WAIT_TIME) {
 * // Lift cycle done, now index
 * shootState = "INDEX";
 * shootTimer.reset();
 * spindexerServo.setPower(SPIN_POWER); // -0.3
 * }
 * } else if (shootState.equals("INDEX")) {
 * if (shootTimer.milliseconds() > TIME_120) {
 * spindexerServo.setPower(0);
 * shotsFired++;
 * 
 * if (shotsFired < 3) {
 * // Next shot
 * shootState = "LIFT_UP";
 * shootTimer.reset();
 * } else {
 * // Done
 * shootState = "IDLE";
 * isShooterOn = false;
 * shooterMotor.setPower(0);
 * }
 * }
 * }
 * 
 * // =========================================================
 * // 5. Color Sensor Indexing Logic (State Machine)
 * // =========================================================
 * 
 * // Trigger: Color (P/G) AND Distance < 4.0 inches
 * // GUARD: Only if NOT shooting
 * if (shootState.equals("IDLE") && indexState.equals("IDLE")) {
 * String detected = checkColorSensor();
 * boolean validColor = detected.equals("P") || detected.equals("G");
 * 
 * double dist = 100.0; // Default high
 * if (distanceSensor != null) {
 * dist = distanceSensor.getDistance(DistanceUnit.INCH);
 * }
 * 
 * if (validColor && dist < 4.0) {
 * indexState = "WAITING"; // Wait FIRST
 * indexTimer.reset();
 * telemetry.addData("Triggered By", detected + " @ " + String.format("%.1f in",
 * dist));
 * }
 * } else if (indexState.equals("WAITING")) {
 * // Wait 0.75s for ball to settle
 * if (indexTimer.milliseconds() > 750) {
 * indexState = "READING";
 * }
 * } else if (indexState.equals("READING")) {
 * // Read Color Now
 * String detColor = checkColorSensor();
 * // If we lost it, maybe still index? Or check empty?
 * // User said "keeps the current order", implying we store it now.
 * // If it's NONE provided we triggered earlier, let's trust the trigger or
 * // re-verify?
 * // Re-verifying is safer.
 * if (!detColor.equals("NONE") && !detColor.equals("UNKNOWN")) {
 * ballSlots[currentSlot].color = detColor;
 * ballSlots[currentSlot].isEmpty = false;
 * currentSlot = (currentSlot + 1) % 3; // Advance slot
 * ballsLoaded++;
 * 
 * // Start Spinning
 * indexState = "SPINNING";
 * spindexerServo.setPower(SPIN_POWER); // -0.3 (Clockwise)
 * indexTimer.reset();
 * } else {
 * // Start Spinning anyway? Or abort?
 * // Tricky. If it rolled away, spinning might be bad.
 * // But if it settled well, we shd see it.
 * // Let's abort to IDLE if lost.
 * indexState = "IDLE";
 * }
 * } else if (indexState.equals("SPINNING")) {
 * // Spin for 120 degrees
 * if (indexTimer.milliseconds() > TIME_120) {
 * spindexerServo.setPower(0);
 * indexState = "IDLE";
 * }
 * }
 * 
 * // =========================================================
 * // 6. Manual Drive
 * // =========================================================
 * manualDrive();
 * 
 * // =========================================================
 * // Telemetry
 * // =========================================================
 * telemetry.addData("Status", !shootState.equals("IDLE") ? "AUTO-SHOOT: " +
 * shootState
 * : (indexState.equals("IDLE") ? "Ready" : "Indexing: " + indexState));
 * telemetry.addData("Shooter", isShooterOn ? "ON" : "OFF");
 * telemetry.addData("Intake", isIntakeOn ? "ON" : "OFF");
 * 
 * telemetry.addLine("\n--- Lift Calibration ---");
 * telemetry.addData("EDITING", isEditingUp ? ">> UP <<" : ">> DOWN <<");
 * telemetry.addData("UP", "%.2f", LIFT_UP_POS);
 * telemetry.addData("DOWN", "%.2f", LIFT_DOWN_POS);
 * 
 * telemetry.addLine("\n--- Slots ---");
 * for (int i = 0; i < 3; i++) {
 * String s = (i == currentSlot) ? "> " : "  ";
 * s += ballSlots[i].isEmpty ? "Empty" : ballSlots[i].color;
 * telemetry.addData("Slot " + i, s);
 * }
 * telemetry.update();
 * }
 * }
 * 
 * // --- Spindexer Intake Logic ---
 * 
 * // Intake Sequence (Blocking with Manual Drive)
 * private void intakeSequence() {
 * isIntaking = true;
 * // Turn on intake
 * isIntakeOn = true;
 * intakeCoreHex.setPower(1.0);
 * 
 * while (ballsLoaded < 3 && opModeIsActive()) {
 * 
 * // Allow Driving
 * manualDrive();
 * 
 * // Find next empty slot
 * int targetSlot = -1;
 * int slotsToMove = 0;
 * 
 * for (int i = 0; i < 3; i++) {
 * // SpindexerControl Logic: (currentSlot - i + 3) % 3 <-- Checks BACKWARDS?
 * // Let's stick to SpindexerControl logic exactly as requested
 * int checkIndex = (currentSlot - i + 3) % 3;
 * if (ballSlots[checkIndex].isEmpty) {
 * targetSlot = checkIndex;
 * slotsToMove = i;
 * break;
 * }
 * }
 * 
 * if (targetSlot == -1)
 * break; // formatting
 * 
 * // Rotate if needed
 * if (slotsToMove > 0) {
 * spindexerServo.setPower(SPIN_POWER);
 * long spinTime = (long) (slotsToMove * BASE_TIME_PER_SLOT);
 * 
 * // Sleep with drive support is tricky.
 * // Standard sleep blocks everything. We should loop-wait.
 * ElapsedTime sleepTimer = new ElapsedTime();
 * while (opModeIsActive() && sleepTimer.milliseconds() < spinTime) {
 * manualDrive();
 * }
 * 
 * spindexerServo.setPower(0);
 * currentSlot = targetSlot;
 * 
 * // Sleep 200ms
 * sleepTimer.reset();
 * while (opModeIsActive() && sleepTimer.milliseconds() < 200) {
 * manualDrive();
 * }
 * }
 * 
 * // Wait until clear
 * if (!waitUntilClear()) {
 * isIntakeOn = false;
 * intakeCoreHex.setPower(0);
 * isIntaking = false;
 * return;
 * }
 * 
 * // Wait for ball
 * if (!waitForBall(targetSlot)) {
 * isIntakeOn = false;
 * intakeCoreHex.setPower(0);
 * isIntaking = false;
 * return;
 * }
 * }
 * 
 * isIntakeOn = false;
 * intakeCoreHex.setPower(0);
 * isIntaking = false;
 * }
 * 
 * private boolean waitUntilClear() {
 * ElapsedTime safetyTimer = new ElapsedTime();
 * // Check Alpha > Threshold means BLOCKED/PRESENT?
 * // SpindexerControl says: `while (alpha > PRESENCE_THRESHOLD)` -> Wait until
 * // CLEARED (< Threshold)
 * // Wait, normally High Alpha = Object Close. So this waits for object to
 * LEAVE?
 * // Yes, likely waiting for the ramp to be empty before accepting next ball.
 * while (opModeIsActive() && colorSensor.alpha() > PRESENCE_THRESHOLD) {
 * manualDrive();
 * // Cancel with Circle (toggle button)
 * if (gamepad1.circle || safetyTimer.seconds() > 3.0)
 * return false;
 * 
 * telemetry.addData("Status", "Waiting for Clear...");
 * telemetry.update();
 * }
 * return true;
 * }
 * 
 * private boolean waitForBall(int slotIndex) {
 * ElapsedTime safetyTimer = new ElapsedTime();
 * 
 * while (opModeIsActive()) {
 * manualDrive();
 * 
 * String color = checkColorSensor();
 * 
 * if (!color.equals("UNKNOWN") && !color.equals("NONE")) {
 * ballSlots[slotIndex].color = color;
 * ballSlots[slotIndex].isEmpty = false;
 * ballsLoaded++;
 * 
 * // Settle Delay (1000ms) with Drive
 * ElapsedTime delay = new ElapsedTime();
 * while (opModeIsActive() && delay.milliseconds() < SETTLE_DELAY) {
 * manualDrive();
 * }
 * 
 * // Index One Spot
 * spindexerServo.setPower(SPIN_POWER);
 * delay.reset();
 * while (opModeIsActive() && delay.milliseconds() < BASE_TIME_PER_SLOT) {
 * manualDrive();
 * }
 * spindexerServo.setPower(0);
 * 
 * // Update Slot Logic (Rotate Indices?)
 * // SpindexerControl: currentSlot = (currentSlot - 1 + 3) % 3;
 * currentSlot = (currentSlot - 1 + 3) % 3;
 * 
 * return true;
 * }
 * 
 * // Timeout or Cancel
 * if (gamepad1.circle || safetyTimer.seconds() > 5.0)
 * return false;
 * 
 * telemetry.addData("Status", "Waiting for Ball...");
 * telemetry.update();
 * }
 * return false;
 * }
 * 
 * // --- New: Color Sensor Logic (SpindexerControl Version) ---
 * private String checkColorSensor() {
 * int r = colorSensor.red();
 * int g = colorSensor.green();
 * int b = colorSensor.blue();
 * int alpha = colorSensor.alpha();
 * 
 * if (alpha < PRESENCE_THRESHOLD)
 * return "NONE";
 * 
 * if (g > r && g > b)
 * return "G";
 * if (r > g && b > g)
 * return "P";
 * 
 * return "UNKNOWN";
 * }
 * 
 * // --- New: BallSlot Class ---
 * private class BallSlot {
 * public String color = "Empty"; // "Empty", "RED", "BLUE", "UNKNOWN"
 * public boolean isEmpty = true;
 * 
 * public BallSlot() {
 * this.color = "Empty";
 * this.isEmpty = true;
 * }
 * }
 * 
 * // --- Mecanum Drive Logic ---
 * 
 * private void manualDrive() {
 * double y = gamepad1.left_trigger - gamepad1.right_trigger;
 * double x = gamepad1.left_stick_x;
 * double rx = gamepad1.right_stick_x;
 * 
 * double fl = y + x + rx;
 * double fr = y - x - rx;
 * double bl = y - x + rx;
 * double br = y + x - rx;
 * 
 * double max = Math.max(1.0,
 * Math.max(Math.abs(fl),
 * Math.max(Math.abs(fr),
 * Math.max(Math.abs(bl), Math.abs(br)))));
 * 
 * fl /= max;
 * fr /= max;
 * bl /= max;
 * br /= max;
 * 
 * double flTarget = fl * MAX_TICKS_PER_SEC;
 * double frTarget = fr * MAX_TICKS_PER_SEC;
 * double blTarget = bl * MAX_TICKS_PER_SEC;
 * double brTarget = br * MAX_TICKS_PER_SEC;
 * 
 * double flCurrent = leftFront.getVelocity();
 * double frCurrent = rightFront.getVelocity();
 * double blCurrent = leftBack.getVelocity();
 * double brCurrent = rightBack.getVelocity();
 * 
 * double flPower = flPID.calculate(flCurrent, flTarget);
 * double frPower = frPID.calculate(frCurrent, frTarget);
 * double blPower = blPID.calculate(blCurrent, blTarget);
 * double brPower = brPID.calculate(brCurrent, brTarget);
 * 
 * leftFront.setPower(flPower);
 * rightFront.setPower(frPower);
 * leftBack.setPower(blPower);
 * rightBack.setPower(brPower);
 * }
 * 
 * // PID Controller class
 * static class PIDController {
 * private double kP, kI, kD, kF;
 * private double integralSum = 0;
 * private double lastError = 0;
 * private ElapsedTime timer = new ElapsedTime();
 * private static final double INTEGRAL_MAX = 0.3;
 * 
 * public PIDController(double kP, double kI, double kD, double kF) {
 * this.kP = kP;
 * this.kI = kI;
 * this.kD = kD;
 * this.kF = kF;
 * timer.reset();
 * }
 * 
 * public double calculate(double current, double target) {
 * double dt = timer.seconds();
 * timer.reset();
 * 
 * double error = target - current;
 * double P = kP * error;
 * 
 * integralSum += error * dt;
 * integralSum = clamp(integralSum, -INTEGRAL_MAX / kI, INTEGRAL_MAX / kI);
 * double I = kI * integralSum;
 * 
 * double derivative = (dt > 0) ? (error - lastError) / dt : 0;
 * double D = kD * derivative;
 * 
 * double F = kF * target;
 * 
 * lastError = error;
 * 
 * double output = P + I + D + F;
 * return clamp(output, -1.0, 1.0);
 * }
 * 
 * private double clamp(double value, double min, double max) {
 * return Math.max(min, Math.min(max, value));
 * }
 * }
 * }
 */
