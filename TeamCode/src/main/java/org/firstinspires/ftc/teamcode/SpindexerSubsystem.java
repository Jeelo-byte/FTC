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

    // Constants
    private static final int BASE_TIME_PER_SLOT = 380;
    private static final int SETTLE_DELAY = 1000;
    public static final double SPIN_POWER = -0.75;
    private static final int PRESENCE_THRESHOLD = 150;

    // State
    private BallSlot[] ballSlots = new BallSlot[3];
    private int currentSlot = 0;
    private int ballsLoaded = 0;
    private boolean isIntaking = false;

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, "spindexer_servo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Init Hardware
        spindexerServo.setPower(0);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (int i = 0; i < 3; i++)
            ballSlots[i] = new BallSlot();
    }

    // --- Actions ---

    public void startIntakeSequence(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
        if (ballsLoaded >= 3) {
            resetSlots(); // Auto-reset if full when started
        }

        isIntaking = true;
        intakeMotor.setPower(1.0);

        while (ballsLoaded < 3 && opModeActive) {
            driveAction.run();

            // Find next empty
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
                break;

            // Rotate if needed
            if (slotsToMove > 0) {
                spindexerServo.setPower(SPIN_POWER);
                long spinTime = (long) (slotsToMove * BASE_TIME_PER_SLOT);
                sleepWithDrive(spinTime, driveAction, opModeActive);
                spindexerServo.setPower(0);
                currentSlot = targetSlot;
                sleepWithDrive(200, driveAction, opModeActive);
            }

            if (!waitUntilClear(driveAction, opModeActive, telemetry)) {
                isIntaking = false;
                break;
            }

            if (!waitForBall(targetSlot, driveAction, opModeActive, telemetry)) {
                isIntaking = false;
                break;
            }
        }

        intakeMotor.setPower(0);
        isIntaking = false;
    }

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

    public BallSlot[] getSlots() {
        return ballSlots;
    }

    public void resetSlots() {
        ballsLoaded = 0;
        for (BallSlot s : ballSlots)
            s.isEmpty = true;
    }

    // --- Helpers ---

    private boolean waitUntilClear(Runnable driveAction, boolean opModeActive, Telemetry telemetry) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive && colorSensor.alpha() > PRESENCE_THRESHOLD) {
            driveAction.run();
            if (t.seconds() > 3.0)
                return false;
            telemetry.addData("Intake", "Waiting for Clear...");
            telemetry.update();
        }
        return true;
    }

    private boolean waitForBall(int slotIndex, Runnable driveAction, boolean opModeActive, Telemetry telemetry) {
        ElapsedTime t = new ElapsedTime();
        while (opModeActive) {
            driveAction.run();

            String color = checkColorSensor();
            if (!color.equals("UNKNOWN") && !color.equals("NONE")) {
                ballSlots[slotIndex].color = color;
                ballSlots[slotIndex].isEmpty = false;
                ballsLoaded++;

                sleepWithDrive(SETTLE_DELAY, driveAction, opModeActive);

                // Index one spot
                spindexerServo.setPower(SPIN_POWER);
                sleepWithDrive(BASE_TIME_PER_SLOT, driveAction, opModeActive);
                spindexerServo.setPower(0);

                currentSlot = (currentSlot - 1 + 3) % 3;
                return true;
            }

            if (t.seconds() > 5.0)
                return false;
            telemetry.addData("Intake", "Waiting for Ball...");
            telemetry.update();
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
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();

        if (alpha < PRESENCE_THRESHOLD)
            return "NONE";
        if (g > r && g > b)
            return "G";
        if (r > g && b > g)
            return "P";

        return "UNKNOWN";
    }

    public static class BallSlot {
        public String color = "Empty";
        public boolean isEmpty = true;
    }
}
