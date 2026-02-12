package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Spindexer + Intake subsystem with encoder-based spindexer rotation.
 *
 * Hardware:
 * - Spindexer motor: core_hex/coreHex (encoder required)
 * - Intake motor: intake_motor
 * - Color sensor: sensor_color
 */
public class SpindexerSubsystem {

    private final DcMotorEx spindexerMotor;
    private final DcMotor intakeMotor;
    private final ColorSensor colorSensor;
    private final boolean intakeEnabled;

    // Encoder/geometry tuning
    public static double SPINDEXER_TICKS_PER_REV = 288.0;
    public static double SPINDEXER_GEAR_RATIO = 1.0;
    public static double SPINDEXER_DIRECTION_SIGN = -1.0; // +1 or -1 for direction tuning
    public static double SPINDEXER_POWER = 0.55;
    public static int SPINDEXER_TOLERANCE_TICKS = 8;
    public static long SPINDEXER_BASE_TIMEOUT_MS_PER_120_DEG = 450;

    // Color logic
    private static final int MIN_BRIGHTNESS = 100;
    private static final int PRESENCE_THRESHOLD = 150;

    // Auto timings
    private static final long BALL_SETTLE_MS = 250;
    private static final long POST_INDEX_PAUSE_MS = 200;
    private static final double CLEAR_TIMEOUT_S = 3.0;
    private static final double BALL_TIMEOUT_S = 5.0;

    // Slot state
    private final BallSlot[] ballSlots = new BallSlot[3];
    private int currentSlot = 0;
    private int ballsLoaded = 0;

    // Auto-intake state machine
    private boolean autoIntakeEnabled = false;
    private AutoState autoState = AutoState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int targetSlot = -1;
    private int slotsToMove = 0;
    private long pauseMs = 0;

    // Spindexer move state (non-blocking)
    private boolean spindexerMoveActive = false;
    private int spindexerTargetTicks = 0;
    private long spindexerMoveTimeoutMs = 0;
    private final ElapsedTime spindexerMoveTimer = new ElapsedTime();

    // Telemetry cache
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
        this(hardwareMap, true);
    }

    public SpindexerSubsystem(HardwareMap hardwareMap, boolean intakeEnabled) {
        this.intakeEnabled = intakeEnabled;
        spindexerMotor = getSpindexerMotor(hardwareMap);
        intakeMotor = intakeEnabled ? hardwareMap.get(DcMotor.class, "intake_motor") : null;
        colorSensor = intakeEnabled ? hardwareMap.get(ColorSensor.class, "sensor_color") : null;

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setTargetPosition(0);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(0.0);

        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (colorSensor != null) {
            try {
                colorSensor.enableLed(true);
            } catch (Exception ignored) {
            }
        }

        for (int i = 0; i < 3; i++) {
            ballSlots[i] = new BallSlot();
        }
    }

    private DcMotorEx getSpindexerMotor(HardwareMap hardwareMap) {
        try {
            return hardwareMap.get(DcMotorEx.class, "spinmotor");
        } catch (RuntimeException e) {
            return hardwareMap.get(DcMotorEx.class, "coreHex");
        }
    }

    // ---------------------------------------------------------------------
    // Auto-intake controls (toggle)
    // ---------------------------------------------------------------------

    public void toggleAutoIntake() {
        if (!intakeEnabled) {
            return;
        }
        if (autoIntakeEnabled) {
            stopAutoIntake();
        } else {
            startAutoIntake();
        }
    }

    public void startAutoIntake() {
        if (!intakeEnabled || intakeMotor == null || colorSensor == null) {
            return;
        }
        if (ballsLoaded >= 3) {
            resetSlots();
        }
        autoIntakeEnabled = true;
        intakeMotor.setPower(1.0);
        autoState = AutoState.FIND_TARGET;
        stateTimer.reset();
    }

    public void stopAutoIntake() {
        autoIntakeEnabled = false;
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        stopSpindexerMove();
        autoState = AutoState.IDLE;
        targetSlot = -1;
        slotsToMove = 0;
        pauseMs = 0;
    }

    public boolean isAutoIntakeEnabled() {
        return autoIntakeEnabled;
    }

    public void updateAutoIntake(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
        if (!intakeEnabled || intakeMotor == null || colorSensor == null) {
            autoIntakeEnabled = false;
            autoState = AutoState.IDLE;
            addTelemetry(telemetry);
            return;
        }

        lastDetectedColor = checkColorSensor();

        if (!opModeActive) {
            stopAutoIntake();
            return;
        }

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
                startSpindexerMoveDegrees(-deg);
                autoState = AutoState.ROTATE_TO_TARGET_WAIT;
                break;
            }

            case ROTATE_TO_TARGET_WAIT: {
                driveAction.run();
                if (updateSpindexerMove()) {
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
                startSpindexerMoveDegrees(-120.0);
                autoState = AutoState.INDEX_WAIT;
                break;
            }

            case INDEX_WAIT: {
                driveAction.run();
                if (updateSpindexerMove()) {
                    autoState = AutoState.WAIT_COLOR_CLEAR;
                    stateTimer.reset();
                }
                break;
            }

            case WAIT_COLOR_CLEAR: {
                driveAction.run();
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
            if (ballsLoaded >= 3) {
                break;
            }
        }
    }

    // ---------------------------------------------------------------------
    // Spindexer movement API (degrees)
    // ---------------------------------------------------------------------

    /**
     * Positive degrees mean clockwise from caller perspective (legacy behavior).
     */
    public void moveSpindexer(double spindexerDegrees, Runnable driveAction, boolean opModeActive) {
        moveSpindexerBlocking(-spindexerDegrees, driveAction, opModeActive);
    }

    /**
     * Auto move for shooting. Keeps same signature as before.
     */
    public void moveSpindexerAutoDegrees(double spindexerDegrees, Runnable driveAction, boolean opModeActive) {
        moveSpindexerBlocking(-spindexerDegrees, driveAction, opModeActive);
    }

    public void rotateToShooter(Runnable driveAction, boolean opModeActive) {
        moveSpindexer(62.0 / 1.8, driveAction, opModeActive);
    }

    public void indexOneSlot(Runnable driveAction, boolean opModeActive) {
        moveSpindexer(58.0 / 1.8, driveAction, opModeActive);
        safeSleep(25);
        moveSpindexer(58.20 / 1.8, driveAction, opModeActive);
    }

    public void setIntakePower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    public boolean isIntaking() {
        return intakeMotor != null && intakeMotor.getPower() != 0.0;
    }

    public boolean isIntakeEnabled() {
        return intakeEnabled;
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

    private void moveSpindexerBlocking(double internalDegrees, Runnable driveAction, boolean opModeActive) {
        startSpindexerMoveDegrees(internalDegrees);
        while (opModeActive && !updateSpindexerMove()) {
            driveAction.run();
        }
    }

    private void startSpindexerMoveDegrees(double internalDegrees) {
        int deltaTicks = degreesToTicks(internalDegrees * SPINDEXER_DIRECTION_SIGN);
        spindexerTargetTicks = spindexerMotor.getCurrentPosition() + deltaTicks;

        spindexerMotor.setTargetPosition(spindexerTargetTicks);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(Math.abs(SPINDEXER_POWER));

        spindexerMoveTimeoutMs = moveTimeoutMsForDegrees(Math.abs(internalDegrees));
        spindexerMoveTimer.reset();
        spindexerMoveActive = true;
    }

    private boolean updateSpindexerMove() {
        if (!spindexerMoveActive) {
            return true;
        }

        int err = Math.abs(spindexerTargetTicks - spindexerMotor.getCurrentPosition());
        boolean finished = err <= SPINDEXER_TOLERANCE_TICKS || !spindexerMotor.isBusy();
        boolean timedOut = spindexerMoveTimer.milliseconds() >= spindexerMoveTimeoutMs;

        if (finished || timedOut) {
            stopSpindexerMove();
            return true;
        }
        return false;
    }

    private void stopSpindexerMove() {
        spindexerMotor.setPower(0.0);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMoveActive = false;
    }

    private int degreesToTicks(double degrees) {
        double ticks = (degrees / 360.0) * SPINDEXER_TICKS_PER_REV * SPINDEXER_GEAR_RATIO;
        return (int) Math.round(ticks);
    }

    private long moveTimeoutMsForDegrees(double degrees) {
        return 150 + Math.round((degrees / 120.0) * SPINDEXER_BASE_TIMEOUT_MS_PER_120_DEG);
    }

    private String checkColorSensor() {
        if (colorSensor == null) {
            return "NONE";
        }

        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();
        a = colorSensor.alpha();

        if (a < PRESENCE_THRESHOLD) {
            return "NONE";
        }
        if (g > MIN_BRIGHTNESS || b > MIN_BRIGHTNESS) {
            if (g > r && g > b) {
                return "GREEN";
            }
            if (b > r && b > g) {
                return "PURPLE";
            }
        }
        return "UNKNOWN";
    }

    private void addTelemetry(Telemetry telemetry) {
        if (telemetry == null) {
            return;
        }

        telemetry.addData("AutoIntake", autoIntakeEnabled ? "ON" : "OFF");
        telemetry.addData("Intake Enabled", intakeEnabled ? "YES" : "NO");
        telemetry.addData("AutoState", autoState);
        telemetry.addData("BallsLoaded", ballsLoaded);
        telemetry.addData("CurrentSlot", currentSlot);
        telemetry.addData("TargetSlot", targetSlot);
        telemetry.addData("LastDetected", lastDetectedColor);
        telemetry.addData("SpindexerTicks", spindexerMotor.getCurrentPosition());
        telemetry.addData("SpindexerBusy", spindexerMoveActive);

        telemetry.addLine("Slots (0..2):");
        for (int i = 0; i < 3; i++) {
            telemetry.addData("  slot[" + i + "]", ballSlots[i].isEmpty ? "Empty" : ballSlots[i].color);
        }

        telemetry.addData("Sensor RGBA", "R:%d G:%d B:%d A:%d", r, g, b, a);
    }

    private void safeSleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static class BallSlot {
        public String color = "Empty";
        public boolean isEmpty = true;
    }
}
