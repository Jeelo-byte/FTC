package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.CRServo; // Reverted to CRServo
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.*;

public class SpindexerSubsystem {

    private CRServo spindexerServo; // Reverted to CRServo
    private DcMotor intakeMotor;
    private ColorSensor colorSensor;

    // Constants
    // User tuned: 120 degrees ~= 300ms at Power 0.75 -> 600ms at Power 0.375
    // 600 ms / 120 deg = 5.0 ms per degree
    private static final double MS_PER_DEGREE = 330.5 / 120.0;
    public static final double SPIN_POWER = -0.375; // Slower Clockwise

    private static final int SETTLE_DELAY = 1000;
    private static final int PRESENCE_THRESHOLD = 150;
    private static final int DETECTION_DELAY = 750;

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
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Init Hardware
        spindexerServo.setPower(0);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ENABLE LED
        colorSensor.enableLed(true);

        for (int i = 0; i < 3; i++)
            ballSlots[i] = new BallSlot();
    }

    // --- Actions ---

    public void toggleIntake(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
        if (isIntaking) {
            isIntaking = false;
            intakeMotor.setPower(0);
        } else {
            startIntakeSequence(telemetry, driveAction, opModeActive);
        }
    }

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
                    slotsToMove = i; // 0, 1, or 2 slots away
                    break;
                }
            }

            if (targetSlot == -1)
                break;

            // 2. Rotate to Target
            if (slotsToMove > 0) {
                // Rotate Spindexer: slots * 120 degrees
                double deg = slotsToMove * 120.0;
                moveSpindexer(deg, driveAction, opModeActive);

                currentSlot = targetSlot;
                sleepWithDrive(200, driveAction, opModeActive);
            }

            // 3. Wait for Clear
            if (!waitUntilClear(driveAction, opModeActive, telemetry)) {
                break;
            }

            // 4. Wait for Ball
            if (!waitForBall(targetSlot, driveAction, opModeActive, telemetry)) {
                break;
            }
        }

        intakeMotor.setPower(0);
        isIntaking = false;
    }

    // Core Movement Logic
    public void moveSpindexer(double spindexerDegrees, Runnable driveAction, boolean opModeActive) {
        // Time Conversion
        long duration = (long) (spindexerDegrees * MS_PER_DEGREE);

        spindexerServo.setPower(SPIN_POWER);
        sleepWithDrive(duration, driveAction, opModeActive);
        spindexerServo.setPower(0);
    }

    public void rotateToShooter(Runnable driveAction, boolean opModeActive) {
        moveSpindexer(60.0, driveAction, opModeActive);
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

    // --- Helpers ---

    private boolean waitUntilClear(Runnable driveAction, boolean opModeActive, Telemetry telemetry) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive && isIntaking && colorSensor.alpha() > PRESENCE_THRESHOLD) {
            driveAction.run();
            if (t.seconds() > 3.0)
                return false;
            if (telemetry != null)
                telemetry.update();
        }
        return isIntaking;
    }

    private boolean waitForBall(int slotIndex, Runnable driveAction, boolean opModeActive, Telemetry telemetry) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive && isIntaking) {
            driveAction.run();

            String color = checkColorSensor();
            if (!color.equals("NONE")) {
                // USER REQUEST: Turn off detection moving spindexer for now.
                // Just log it but don't act.
                if (telemetry != null) {
                    telemetry.addData("Intake", "Ball Detected (Auto-Move Disabled)");
                    telemetry.update();
                }

                // return true; // Disabled
            }

            if (t.seconds() > 5.0)
                return false;
            if (telemetry != null) {
                telemetry.addData("Intake", "Waiting...");
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

        if (isColorMatch(r, g, b, 87, 300, 225))
            return "G";
        if (isColorMatch(r, g, b, 160, 198, 290))
            return "P";

        return "UNKNOWN";
    }

    private boolean isColorMatch(int r, int g, int b, int targetR, int targetG, int targetB) {
        double tolerance = 60.0;
        double dist = Math.sqrt(Math.pow(r - targetR, 2) + Math.pow(g - targetG, 2) + Math.pow(b - targetB, 2));
        return dist < tolerance;
    }

    // --- Accessors ---

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    // resetServo removed (incompatible with CRServo)

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
