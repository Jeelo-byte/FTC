package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Spindexer + Intake subsystem.
 *
 * Includes:
 * - Manual spindexer moves (your tuned timings)
 * - Auto-intake as a NON-BLOCKING state machine (call updateAutoIntake each loop)
 * - Color detection logic copied from ColorSortREVLogic (GREEN/PURPLE)
 * - SEPARATE TUNING:
 * 1. SHOOTING uses the original 0.55 multiplier (perfect).
 * 2. INTAKE uses a reduced 0.45 multiplier to fix overshoot.
 */
public class SpindexerSubsystem {

    private final ServoMovementHelper spindexerHelper;
    private final DcMotor intakeMotor;
    private final ColorSensor colorSensor;

    // -----------------------------
    // Movement (your tuned values)
    // -----------------------------
    // User tuned: 120 degrees ~= 330.5ms at your configured power
    private static final double MS_PER_DEGREE = 619.69 / 120.0;
    public static final double SPIN_POWER = 0.375; // Positive power (helper handles direction)

    // -----------------------------
    // Multipliers (Separated to fix overshoot without breaking shooting)
    // -----------------------------

    // 1. SHOOTING MULTIPLIER (Old AUTO_TURN_MULTIPLIER)
    // KEPT EXACTLY THE SAME so shooting remains perfect.
    private static final double SHOOT_TURN_MULTIPLIER = 0.55;

    // 2. INTAKE MULTIPLIER (NEW)
    // Reduced from 0.55 to 0.45 to fix the intake overshoot.
    // If it still overshoots, LOWER this. If it undershoots, RAISE it.
    private static final double INTAKE_TURN_MULTIPLIER = 0.45;

    // -----------------------------
    // Color logic (from your OpMode)
    // -----------------------------
    private static final int MIN_BRIGHTNESS = 100;
    private static final int PRESENCE_THRESHOLD = 150; // alpha gate

    // -----------------------------
    // Auto timings
    // -----------------------------
    private static final long BALL_SETTLE_MS = 250;
    private static final long POST_INDEX_PAUSE_MS = 200;

    private static final double CLEAR_TIMEOUT_S = 3.0;
    private static final double BALL_TIMEOUT_S = 5.0;

    // -----------------------------
    // Slot state
    // -----------------------------
    private final BallSlot[] ballSlots = new BallSlot[3];
    private int currentSlot = 0;
    private int ballsLoaded = 0;

    // -----------------------------
    // Auto-intake state machine
    // -----------------------------
    private boolean autoIntakeEnabled = false;
    private AutoState autoState = AutoState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private int targetSlot = -1;
    private int slotsToMove = 0;
    private long pauseMs = 0;

    // Sensor cache for telemetry
    private int r, g, b, a;
    private String lastDetectedColor = "NONE";

    private enum AutoState {
        IDLE,
        FIND_TARGET,
        ROTATE_TO_TARGET_START,
        ROTATE_TO_TARGET_WAIT,
        WAIT_CLEAR,
        WAIT_BALL,
        SETTLE,
        INDEX_START,
        INDEX_WAIT,
        WAIT_COLOR_CLEAR,
        POST_INDEX_PAUSE,
        DONE
    }

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        CRServo spindexerServo = hardwareMap.get(CRServo.class, "spindexer_servo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Initialize spindexer helper with tuned ms/degree
        spindexerHelper = new ServoMovementHelper(spindexerServo, MS_PER_DEGREE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            colorSensor.enableLed(true);
        } catch (Exception ignored) {
        }

        for (int i = 0; i < 3; i++)
            ballSlots[i] = new BallSlot();
    }

    // ---------------------------------------------------------------------
    // Auto-intake controls (toggle)
    // ---------------------------------------------------------------------

    public void toggleAutoIntake() {
        if (autoIntakeEnabled)
            stopAutoIntake();
        else
            startAutoIntake();
    }

    public void startAutoIntake() {
        if (ballsLoaded >= 3)
            resetSlots();

        autoIntakeEnabled = true;
        intakeMotor.setPower(1.0);

        autoState = AutoState.FIND_TARGET;
        stateTimer.reset();
    }

    public void stopAutoIntake() {
        autoIntakeEnabled = false;
        intakeMotor.setPower(0.0);
        spindexerHelper.stop();

        autoState = AutoState.IDLE;
        targetSlot = -1;
        slotsToMove = 0;
        pauseMs = 0;
    }

    public boolean isAutoIntakeEnabled() {
        return autoIntakeEnabled;
    }

    /**
     * Call ONCE per TeleOp loop. Non-blocking.
     */
    public void updateAutoIntake(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
        // Keep sensor cache fresh for both logic + telemetry
        lastDetectedColor = checkColorSensor();

        if (!opModeActive) {
            stopAutoIntake();
            return;
        }

        // Auto stop when full
        if (autoIntakeEnabled && ballsLoaded >= 3) {
            autoState = AutoState.DONE;
        }

        if (!autoIntakeEnabled) {
            addTelemetry(telemetry);
            return;
        }

        switch (autoState) {

            case FIND_TARGET: {
                targetSlot = -1;
                slotsToMove = 0;

                // Find nearest empty slot relative to currentSlot
                for (int i = 0; i < 3; i++) {
                    int checkIndex = (currentSlot - i + 3) % 3;
                    if (ballSlots[checkIndex].isEmpty) {
                        targetSlot = checkIndex;
                        slotsToMove = i;
                        break;
                    }
                }

                if (targetSlot == -1) {
                    autoState = AutoState.DONE;
                    break;
                }

                autoState = (slotsToMove > 0) ? AutoState.ROTATE_TO_TARGET_START : AutoState.WAIT_CLEAR;
                stateTimer.reset();
                break;
            }

            case ROTATE_TO_TARGET_START: {
                double deg = slotsToMove * 120.0;
                // --- FIX: USE INTAKE MULTIPLIER (0.45) TO PREVENT OVERSHOOT ---
                spindexerHelper.startMoveWithMultiplier(-deg, SPIN_POWER, INTAKE_TURN_MULTIPLIER);
                stateTimer.reset();
                autoState = AutoState.ROTATE_TO_TARGET_WAIT;
                break;
            }

            case ROTATE_TO_TARGET_WAIT: {
                driveAction.run();

                if (spindexerHelper.update()) {
                    currentSlot = targetSlot;

                    pauseMs = 200;
                    stateTimer.reset();
                    autoState = AutoState.POST_INDEX_PAUSE;
                }
                break;
            }

            case WAIT_CLEAR: {
                driveAction.run();

                if (colorSensor.alpha() <= PRESENCE_THRESHOLD) {
                    autoState = AutoState.WAIT_BALL;
                    stateTimer.reset();
                    break;
                }

                if (stateTimer.seconds() > CLEAR_TIMEOUT_S) {
                    autoState = AutoState.FIND_TARGET;
                    stateTimer.reset();
                }
                break;
            }

            case WAIT_BALL: {
                driveAction.run();

                if (lastDetectedColor.equals("GREEN") || lastDetectedColor.equals("PURPLE")) {
                    ballSlots[targetSlot].isEmpty = false;
                    ballSlots[targetSlot].color = lastDetectedColor;
                    ballsLoaded++;

                    pauseMs = BALL_SETTLE_MS;
                    stateTimer.reset();
                    autoState = AutoState.SETTLE;
                    break;
                }

                if (stateTimer.seconds() > BALL_TIMEOUT_S) {
                    autoState = AutoState.FIND_TARGET;
                    stateTimer.reset();
                }
                break;
            }

            case SETTLE: {
                driveAction.run();
                if (stateTimer.milliseconds() >= pauseMs) {
                    autoState = AutoState.INDEX_START;
                }
                break;
            }

            case INDEX_START: {
                // --- FIX: USE INTAKE MULTIPLIER (0.45) TO PREVENT OVERSHOOT ---
                spindexerHelper.startMoveWithMultiplier(-120.0, SPIN_POWER, INTAKE_TURN_MULTIPLIER);
                stateTimer.reset();
                autoState = AutoState.INDEX_WAIT;
                break;
            }

            case INDEX_WAIT: {
                driveAction.run();
                if (spindexerHelper.update()) {
                    autoState = AutoState.WAIT_COLOR_CLEAR;
                    stateTimer.reset();
                }
                break;
            }

            case WAIT_COLOR_CLEAR: {
                driveAction.run();

                // Mirrors your OpMode: wait while G/B are still bright
                if (!(colorSensor.green() > MIN_BRIGHTNESS || colorSensor.blue() > MIN_BRIGHTNESS)) {
                    currentSlot = (currentSlot + 1) % 3;

                    pauseMs = POST_INDEX_PAUSE_MS;
                    stateTimer.reset();
                    autoState = AutoState.POST_INDEX_PAUSE;
                    break;
                }

                if (stateTimer.seconds() > CLEAR_TIMEOUT_S) {
                    currentSlot = (currentSlot + 1) % 3;
                    pauseMs = POST_INDEX_PAUSE_MS;
                    stateTimer.reset();
                    autoState = AutoState.POST_INDEX_PAUSE;
                }
                break;
            }

            case POST_INDEX_PAUSE: {
                driveAction.run();
                if (stateTimer.milliseconds() >= pauseMs) {
                    autoState = AutoState.FIND_TARGET;
                    stateTimer.reset();
                }
                break;
            }

            case DONE: {
                stopAutoIntake();
                break;
            }

            case IDLE:
            default:
                break;
        }

        addTelemetry(telemetry);
    }

    // ---------------------------------------------------------------------
    // Backward-compatible methods
    // ---------------------------------------------------------------------

    public void toggleIntake(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
        toggleAutoIntake();
        addTelemetry(telemetry);
    }

    public void startIntakeSequence(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
        startAutoIntake();
        while (opModeActive && isAutoIntakeEnabled()) {
            updateAutoIntake(telemetry, driveAction, opModeActive);
            if (ballsLoaded >= 3)
                break;
        }
    }

    // ---------------------------------------------------------------------
    // Manual movement (using helper for consistency)
    // ---------------------------------------------------------------------

    public void moveSpindexer(double spindexerDegrees, Runnable driveAction, boolean opModeActive) {
        // Negative degrees for clockwise rotation
        spindexerHelper.moveBlocking(-spindexerDegrees, SPIN_POWER, driveAction, opModeActive);
    }

    /**
     * Auto move used by SHOOTING.
     * Uses SHOOT_TURN_MULTIPLIER (0.55) to keep shooting "perfect".
     */
    public void moveSpindexerAutoDegrees(double spindexerDegrees, Runnable driveAction, boolean opModeActive) {
        // Negative degrees for clockwise rotation
        spindexerHelper.moveBlockingWithMultiplier(-spindexerDegrees, SPIN_POWER,
                SHOOT_TURN_MULTIPLIER, driveAction, opModeActive);
    }

    public void rotateToShooter(Runnable driveAction, boolean opModeActive) {
        moveSpindexer(65.0, driveAction, opModeActive);
    }

    public void indexOneSlot(Runnable driveAction, boolean opModeActive) {
        moveSpindexer(60.0, driveAction, opModeActive);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        moveSpindexer(60.0, driveAction, opModeActive);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public boolean isIntaking() {
        return intakeMotor.getPower() != 0.0;
    }

    public int getBallsLoaded() {
        return ballsLoaded;
    }

    public BallSlot[] getBallSlots() {
        return ballSlots;
    }

    public void resetSlots() {
        ballsLoaded = 0;
        currentSlot = 0;
        targetSlot = -1;
        for (BallSlot s : ballSlots) {
            s.isEmpty = true;
            s.color = "Empty";
        }
    }

    // ---------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------

    private String checkColorSensor() {
        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();
        a = colorSensor.alpha();

        if (a < PRESENCE_THRESHOLD)
            return "NONE";

        if (g > MIN_BRIGHTNESS || b > MIN_BRIGHTNESS) {
            if (g > r && g > b)
                return "GREEN";
            if (b > r && b > g)
                return "PURPLE";
        }
        return "UNKNOWN";
    }

    private void addTelemetry(Telemetry telemetry) {
        if (telemetry == null)
            return;

        telemetry.addData("AutoIntake", autoIntakeEnabled ? "ON" : "OFF");
        telemetry.addData("AutoState", autoState);
        telemetry.addData("BallsLoaded", ballsLoaded);
        telemetry.addData("CurrentSlot", currentSlot);
        telemetry.addData("TargetSlot", targetSlot);
        telemetry.addData("LastDetected", lastDetectedColor);

        telemetry.addLine("Slots (0..2):");
        for (int i = 0; i < 3; i++) {
            telemetry.addData("  slot[" + i + "]", ballSlots[i].isEmpty ? "Empty" : ballSlots[i].color);
        }

        telemetry.addData("Sensor RGBA", "R:%d G:%d B:%d A:%d", r, g, b, a);
    }

    public static class BallSlot {
        public String color = "Empty";
        public boolean isEmpty = true;
    }
}