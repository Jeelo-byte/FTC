package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Arrays;

@TeleOp(name = "SpindexerV5", group = "Competition")
public class SpindexerV5 extends LinearOpMode {

    SpindexerSubsystem spindexer;

    String[] targetMotif = {"PURPLE", "GREEN", "PURPLE"};

    @Override
    public void runOpMode() {
        spindexer = new SpindexerSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized (Spindexer V5)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.triangle) {
                if (spindexer.getBallsLoaded() == 0) {
                    telemetry.addData("SCAN", "Scanning Motif...");
                    telemetry.update();
                    targetMotif = new String[]{"GREEN", "PURPLE", "GREEN"};
                    sleep(500);
                } else {
                    spindexer.toggleAutoIntake();
                    sleep(300);
                }
            }

            if (gamepad1.circle) {
                spindexer.shootAllSequence(targetMotif, telemetry, this::idle, opModeIsActive());
            }

            if (gamepad1.square) {
                spindexer.smartShoot("PURPLE", telemetry, this::idle, opModeIsActive());
            }
            if (gamepad1.cross) {
                spindexer.smartShoot("GREEN", telemetry, this::idle, opModeIsActive());
            }

            spindexer.updateAutoIntake(telemetry, this::idle, opModeIsActive());

            if (gamepad1.dpad_down) {
                spindexer.hardReset();
            }

            telemetry.addData("Motif", Arrays.toString(targetMotif));
            telemetry.update();
        }
    }

    public static class SpindexerSubsystem {

        private final Servo spindexerServo;
        private final DcMotor intakeMotor;
        private final ColorSensor colorSensor;
        private final Servo liftServo;

        private static final double POS_PER_DEGREE = 1.0 / 1800.0;
        private static final double DEG_PER_SLOT = 120.0;
        private static final double DEG_TO_SHOOTER = 180.0;

        private static final int MIN_BRIGHTNESS = 100;
        private static final int PRESENCE_THRESHOLD = 150;
        private static final long BALL_SETTLE_MS = 250;
        private static final double CLEAR_TIMEOUT_S = 3.0;

        private final BallSlot[] ballSlots = new BallSlot[3];
        private int currentIntakeSlot = 0;
        private int ballsLoaded = 0;

        private double currentServoDeg = 0.0;

        private boolean autoIntakeEnabled = false;
        private AutoState autoState = AutoState.IDLE;
        private final ElapsedTime stateTimer = new ElapsedTime();
        private int targetSlot = -1;
        private String lastDetectedColor = "NONE";

        private enum AutoState {
            IDLE, FIND_TARGET, MOVE_TO_TARGET, WAIT_CLEAR, WAIT_BALL, SETTLE, DONE
        }

        public SpindexerSubsystem(HardwareMap hardwareMap) {
            spindexerServo = hardwareMap.get(Servo.class, "spindexer_servo");
            intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
            colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
            liftServo = hardwareMap.get(Servo.class, "lift_servo");

            liftServo.setPosition(0.0);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            try { colorSensor.enableLed(true); } catch (Exception ignored) {}

            for (int i = 0; i < 3; i++) ballSlots[i] = new BallSlot();

            spindexerServo.setPosition(0.0);
        }

        private void moveServoToDegree(double targetDeg) {
            if (targetDeg > 1750) {
                spindexerServo.setPosition(0.0);
                waitServoMove(1800);
                currentServoDeg = 0.0;
                targetDeg = targetDeg % 360.0;
            }

            double diff = Math.abs(targetDeg - currentServoDeg);
            spindexerServo.setPosition(targetDeg * POS_PER_DEGREE);
            waitServoMove(diff);
            currentServoDeg = targetDeg;
        }

        private void waitServoMove(double degreesToMove) {
            long ms = (long)(degreesToMove * (0.05 / 60.0) * 1000);
            try { Thread.sleep(ms + 50); } catch (InterruptedException e) {}
        }

        public boolean smartShoot(String targetColor, Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
            stopAutoIntake();

            int foundSlotIndex = -1;
            for (int i = 0; i < 3; i++) {
                if (ballSlots[i].color.equals(targetColor)) {
                    foundSlotIndex = i;
                    break;
                }
            }
            if (foundSlotIndex == -1) return false;

            int slotDiff = foundSlotIndex - currentIntakeSlot;
            if (slotDiff < 0) slotDiff += 3;

            double degreesToShooter = currentServoDeg + (slotDiff * 120.0) + DEG_TO_SHOOTER;
            moveServoToDegree(degreesToShooter);

            liftServo.setPosition(0.5);
            try { Thread.sleep(300); } catch (InterruptedException e) {}
            liftServo.setPosition(0.0);
            try { Thread.sleep(200); } catch (InterruptedException e) {}

            ballSlots[foundSlotIndex].isEmpty = true;
            ballSlots[foundSlotIndex].color = "Empty";
            ballsLoaded--;

            double nextIntakeAlign = degreesToShooter + 180.0;
            moveServoToDegree(nextIntakeAlign);

            currentIntakeSlot = foundSlotIndex;
            return true;
        }

        public void shootAllSequence(String[] pattern, Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
            stopAutoIntake();
            for (String colorNeeded : pattern) {
                if (!opModeActive) break;
                if (smartShoot(colorNeeded, telemetry, driveAction, opModeActive)) {
                    try { Thread.sleep(100); } catch (InterruptedException e) {}
                }
            }
        }

        public void updateAutoIntake(Telemetry telemetry, Runnable driveAction, boolean opModeActive) {
            lastDetectedColor = checkColorSensor();
            if (!opModeActive) { stopAutoIntake(); return; }
            if (autoIntakeEnabled && ballsLoaded >= 3) autoState = AutoState.DONE;
            if (!autoIntakeEnabled) { addTelemetry(telemetry); return; }

            switch (autoState) {
                case FIND_TARGET:
                    targetSlot = -1;
                    for (int i = 0; i < 3; i++) {
                        int checkIndex = (currentIntakeSlot + i) % 3;
                        if (ballSlots[checkIndex].isEmpty) {
                            targetSlot = checkIndex;
                            break;
                        }
                    }
                    if (targetSlot == -1) { autoState = AutoState.DONE; break; }

                    if (targetSlot != currentIntakeSlot) {
                        autoState = AutoState.MOVE_TO_TARGET;
                    } else {
                        autoState = AutoState.WAIT_CLEAR;
                    }
                    break;

                case MOVE_TO_TARGET:
                    int diff = targetSlot - currentIntakeSlot;
                    if (diff < 0) diff += 3;
                    double targetPos = currentServoDeg + (diff * 120.0);

                    moveServoToDegree(targetPos);
                    currentIntakeSlot = targetSlot;

                    autoState = AutoState.WAIT_CLEAR;
                    break;

                case WAIT_CLEAR:
                    driveAction.run();
                    if (colorSensor.alpha() <= PRESENCE_THRESHOLD) {
                        autoState = AutoState.WAIT_BALL;
                        stateTimer.reset();
                    }
                    break;

                case WAIT_BALL:
                    driveAction.run();
                    if (lastDetectedColor.equals("GREEN") || lastDetectedColor.equals("PURPLE")) {
                        ballSlots[targetSlot].isEmpty = false;
                        ballSlots[targetSlot].color = lastDetectedColor;
                        ballsLoaded++;
                        stateTimer.reset();
                        autoState = AutoState.SETTLE;
                    } else if (stateTimer.seconds() > 5.0) {
                        autoState = AutoState.FIND_TARGET;
                    }
                    break;

                case SETTLE:
                    driveAction.run();
                    if (stateTimer.milliseconds() >= BALL_SETTLE_MS) {
                        int nextSlot = (currentIntakeSlot + 1) % 3;
                        double nextPos = currentServoDeg + 120.0;
                        moveServoToDegree(nextPos);
                        currentIntakeSlot = nextSlot;

                        autoState = AutoState.FIND_TARGET;
                    }
                    break;

                case DONE: stopAutoIntake(); break;
            }
            addTelemetry(telemetry);
        }

        public void hardReset() {
            ballsLoaded = 0;
            currentIntakeSlot = 0;
            for (BallSlot s : ballSlots) { s.isEmpty = true; s.color = "Empty"; }
            spindexerServo.setPosition(0.0);
            currentServoDeg = 0.0;
            try { Thread.sleep(1500); } catch (InterruptedException e) {}
        }

        public void toggleAutoIntake() { if (autoIntakeEnabled) stopAutoIntake(); else startAutoIntake(); }
        public void startAutoIntake() {
            if (ballsLoaded >= 3) hardReset();
            autoIntakeEnabled = true;
            intakeMotor.setPower(1.0);
            autoState = AutoState.FIND_TARGET;
        }
        public void stopAutoIntake() {
            autoIntakeEnabled = false;
            intakeMotor.setPower(0.0);
            autoState = AutoState.IDLE;
        }
        public int getBallsLoaded() { return ballsLoaded; }

        private String checkColorSensor() {
            int r = colorSensor.red(); int g = colorSensor.green(); int b = colorSensor.blue(); int a = colorSensor.alpha();
            if (a < PRESENCE_THRESHOLD) return "NONE";
            if (g > 100 || b > 100) {
                if (g > r && g > b) return "GREEN";
                if (b > r && b > g) return "PURPLE";
            }
            return "UNKNOWN";
        }

        private void addTelemetry(Telemetry telemetry) {
            telemetry.addData("AutoIntake", autoIntakeEnabled ? "ON" : "OFF");
            telemetry.addData("ServoDeg", currentServoDeg);
            telemetry.addData("Load", ballsLoaded);
            for (int i = 0; i < 3; i++) telemetry.addData("  Slot " + i, ballSlots[i].color);
        }

        public static class BallSlot {
            public String color = "Empty";
            public boolean isEmpty = true;
        }
    }
}