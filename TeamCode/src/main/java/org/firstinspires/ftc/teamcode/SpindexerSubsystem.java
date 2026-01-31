package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpindexerSubsystem {

    private CRServo spindexerServo;
    private DcMotor intakeMotor;
    private ColorSensor colorSensor;

    // Constants (Matched to SpindexerControl)
    private static final int BASE_TIME_PER_SLOT = 380;
    private static final int SETTLE_DELAY = 1000;
    public static final double SPIN_POWER = -0.75;
    private static final int PRESENCE_THRESHOLD = 150;

    // User Requested Delay
    private static final int DETECTION_DELAY = 750; // 0.75 seconds

    // State
    private BallSlot[] ballSlots = new BallSlot[3];
    private int currentSlot = 0;
    private int ballsLoaded = 0;
    private boolean isIntaking = false;

    // Telemetry Data
    private String lastPattern = "";
    private int r, g, b, alpha;

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, "spindexer_servo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Init Hardware
        spindexerServo.setPower(0);
        //intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ENABLE LED (Fix for color detection)
        colorSensor.enableLed(true);

        for (int i = 0; i < 3; i++)
            ballSlots[i] = new BallSlot();
    }

    // --- Actions ---

    public void toggleIntake(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
        if (isIntaking) {
            isIntaking = false; // Flag to stop
            intakeMotor.setPower(0);
        } else {
            startIntakeSequence(telemetry, driveAction, opModeActive);
        }
    }

    // Blocking Sequence (run in loop in main opmode or separate thread)
    // NOTE: For 'toggle' to work in LinearOpMode, we usually can't block here
    // forever
    // unless we break out.
    // BUT user said "use my implementation... one button to toggle...
    // SpindexerControl.java"
    // SpindexerControl BLOCKS inside intakeSequence.
    // So we will BLOCK here, but check `isIntaking` to exit.

    public void startIntakeSequence(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
        if (ballsLoaded >= 3) {
            resetSlots();
        }

        isIntaking = true;
        intakeMotor.setPower(1.0);

        while (ballsLoaded < 3 && opModeActive && isIntaking) {

            // 1. Find Target Slot
            int targetSlot = -1;
            int slotsToMove = 0;

            for (int i = 0; i < 3; i++) {
                int checkIndex = (currentSlot - i + 3) % 3;
                if (ballSlots[checkIndex].isEmpty) {
                    targetSlot = checkIndex;
                    slotsToMove = i;
                    break;
                }
            }

            if (targetSlot == -1)
                break; // Full?

            // 2. Rotate to Target
            if (slotsToMove > 0) {
                spindexerServo.setPower(SPIN_POWER);
                long spinTime = (long) (slotsToMove * BASE_TIME_PER_SLOT);
                sleepWithDrive(spinTime, driveAction, opModeActive);
                spindexerServo.setPower(0);

                currentSlot = targetSlot;
                sleepWithDrive(200, driveAction, opModeActive);
            }

            // 3. Wait for Clear (if something stuck)
            if (!waitUntilClear(driveAction, opModeActive, telemetry)) {
                break; // isIntaking set false or timeout
            }

            // 4. Wait for Ball (Detection)
            if (!waitForBall(targetSlot, driveAction, opModeActive, telemetry)) {
                break; // isIntaking set false or timeout
            }
        }

        intakeMotor.setPower(0);
        isIntaking = false;
    }

    // --- Helpers ---

    private boolean waitUntilClear(Runnable driveAction, boolean opModeActive, Telemetry telemetry) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive && isIntaking && colorSensor.alpha() > PRESENCE_THRESHOLD) {
            driveAction.run();
            // User SpindexerControl uses 3.0s timeout and 'B' button check.
            // We use isIntaking flag.
            if (t.seconds() > 3.0)
                return false;

            updateTelemetryData();
            if (telemetry != null) {
                telemetry.addData("Intake", "Waiting for Clear...");
                telemetry.update();
            }
        }
        return isIntaking;
    }

    private boolean waitForBall(int slotIndex, Runnable driveAction, boolean opModeActive, Telemetry telemetry) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive && isIntaking) {
            driveAction.run();

            String color = checkColorSensor();
            if (!color.equals("UNKNOWN") && !color.equals("NONE")) {

                // Found Ball!

                // 1. User Requested Delay (750ms) to confirm/separate
                sleepWithDrive(DETECTION_DELAY, driveAction, opModeActive);

                // Re-Verify Color? (Optional but good practice after delay)
                // For now, accept the first detection as valid

                ballSlots[slotIndex].color = color;
                ballSlots[slotIndex].isEmpty = false;
                ballsLoaded++;

                updateTelemetryData();

                // 2. Settle Delay (1000ms) - From SpindexerControl
                sleepWithDrive(SETTLE_DELAY, driveAction, opModeActive);

                // 3. Index ONE spot (Rotate)
                spindexerServo.setPower(SPIN_POWER);
                sleepWithDrive(BASE_TIME_PER_SLOT, driveAction, opModeActive);
                spindexerServo.setPower(0);

                currentSlot = (currentSlot - 1 + 3) % 3;
                return true;
            }

            if (t.seconds() > 5.0)
                return false; // Timeout

            updateTelemetryData();
            if (telemetry != null) {
                telemetry.addData("Intake", "Waiting for Ball...");
                telemetry.update();
            }
        }
        return false;
    }

    private void sleepWithDrive(long ms, Runnable driveAction, boolean opModeActive) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive && t.milliseconds() < ms) {
            driveAction.run();
        }
    }

    private String checkColorSensor() {
        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();
        alpha = colorSensor.alpha();

        if (alpha < PRESENCE_THRESHOLD)
            return "NONE";

        // Logic from SpindexerControl
        if (g > r && g > b)
            return "G";
        if (r > g && b > g)
            return "P";

        return "UNKNOWN";
    }

    // --- Telemetry Helpers ---

    private void updateTelemetryData() {
        StringBuilder pattern = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if (ballSlots[i].isEmpty) {
                pattern.append("_");
            } else {
                pattern.append(ballSlots[i].color);
            }
            if (i < 2)
                pattern.append(" ");
        }
        lastPattern = pattern.toString();

        // Refresh sensor values
        checkColorSensor();
    }

    public void printTelemetry(Telemetry telemetry) {
        updateTelemetryData();
        telemetry.addData("Spindexer", isIntaking ? "INTAKING" : "IDLE");
        telemetry.addData("Count", ballsLoaded + "/3");
        telemetry.addData("Pattern", lastPattern); // G _ P
        telemetry.addData("Sensor", "R:" + r + " G:" + g + " B:" + b + " A:" + alpha);
    }

    // --- Accessors ---

    public void setSpindexerPower(double power) {
        spindexerServo.setPower(power);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public boolean isIntaking() {
        return isIntaking;
    }

    public int getBallsLoaded() {
        return ballsLoaded;
    }

    public void resetSlots() {
        ballsLoaded = 0;
        for (BallSlot s : ballSlots)
            s.isEmpty = true;
    }

    public static class BallSlot {
        public String color = "Empty";
        public boolean isEmpty = true;
    }
}
