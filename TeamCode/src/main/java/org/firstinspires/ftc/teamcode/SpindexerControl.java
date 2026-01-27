package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Spindexer Motor Control")
public class SpindexerControl extends LinearOpMode {

    private CRServo spindexerServo;
    private ColorSensor rampSensor;

    private static final int BASE_TIME_PER_SLOT = 380;
    private static final double SPIN_POWER = 1.0;

    private static final int COLOR_THRESHOLD = 40;
    private static final int PRESENCE_THRESHOLD = 150;

    private BallSlot[] ballSlots = new BallSlot[3];
    private int currentSlot = 0;
    private int ballsLoaded = 0;
    private boolean isIntaking = false;
    private boolean aButtonPreviouslyPressed = false;

    class BallSlot {
        String color = "";
        boolean isEmpty = true;
    }

    @Override
    public void runOpMode() {
        initHardware();

        for (int i = 0; i < 3; i++) ballSlots[i] = new BallSlot();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && !aButtonPreviouslyPressed && !isIntaking) {
                if (ballsLoaded < 3) {
                    intakeSequence();
                }
            }
            aButtonPreviouslyPressed = gamepad1.a;

            updateTelemetry();
        }
    }

    private void intakeSequence() {
        isIntaking = true;

        while (ballsLoaded < 3 && opModeIsActive()) {

            int targetSlot = -1;
            int slotsToMove = 0;

            for (int i = 0; i < 3; i++) {
                int checkIndex = (currentSlot + i) % 3;
                if (ballSlots[checkIndex].isEmpty) {
                    targetSlot = checkIndex;
                    slotsToMove = i;
                    break;
                }
            }

            if (targetSlot == -1) break;

            if (slotsToMove > 0) {
                spindexerServo.setPower(SPIN_POWER);
                long spinTime = (long)(slotsToMove * BASE_TIME_PER_SLOT);
                sleep(spinTime);
                spindexerServo.setPower(0);

                currentSlot = targetSlot;
                sleep(200);
            }

            if (!waitUntilClear()) { isIntaking = false; return; }

            if (!waitForBall(targetSlot)) { isIntaking = false; return; }
        }

        isIntaking = false;
    }

    private boolean waitUntilClear() {
        ElapsedTime safetyTimer = new ElapsedTime();
        while (opModeIsActive() && rampSensor.alpha() > PRESENCE_THRESHOLD) {
            if (gamepad1.b || safetyTimer.seconds() > 3.0) return false;
        }
        return true;
    }

    private boolean waitForBall(int slotIndex) {
        ElapsedTime safetyTimer = new ElapsedTime();
        while (opModeIsActive()) {
            String color = checkColorSensor();

            if (!color.equals("UNKNOWN") && !color.equals("NONE")) {
                ballSlots[slotIndex].color = color;
                ballSlots[slotIndex].isEmpty = false;
                ballsLoaded++;

                spindexerServo.setPower(SPIN_POWER);
                sleep(BASE_TIME_PER_SLOT);
                spindexerServo.setPower(0);

                currentSlot = (currentSlot + 1) % 3;

                return true;
            }

            if (gamepad1.b || safetyTimer.seconds() > 5.0) return false;
        }
        return false;
    }

    private String checkColorSensor() {
        int r = rampSensor.red();
        int g = rampSensor.green();
        int b = rampSensor.blue();
        int alpha = rampSensor.alpha();

        if (alpha < PRESENCE_THRESHOLD) return "NONE";

        if (g > r && g > b) return "G";
        if (r > g && b > g) return "P";

        return "UNKNOWN";
    }

    private void updateTelemetry() {
        telemetry.addData("State", isIntaking ? "INTAKING": "IDLE");
        telemetry.addData("Balls Loaded", ballsLoaded + "/3");
        telemetry.addData("Color R/G/B", rampSensor.red() + " " + rampSensor.green() + " " + rampSensor.blue());
        telemetry.update();
    }

    private void initHardware() {
        spindexerServo = hardwareMap.get(CRServo.class, "spindexer");
        rampSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        rampSensor.enableLed(true);
        spindexerServo.setPower(0);
    }
}