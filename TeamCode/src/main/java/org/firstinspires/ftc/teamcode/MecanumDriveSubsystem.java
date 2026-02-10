package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumDriveSubsystem {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // PID Constants (From MecanumPIDDrive)
    private static final double MAX_TICKS_PER_SEC = 2796.0;
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.00036;

    private PIDController flPID, frPID, blPID, brPID;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        initHardware(hardwareMap);
    }

    private void initHardware(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right");
        leftBack = hardwareMap.get(DcMotorEx.class, "back_left");
        rightBack = hardwareMap.get(DcMotorEx.class, "back_right");

        // Directions (Matching MecanumPIDDrive)
        // PIDDrive: FL=Rev, BR=Rev. (Implies FR=Fwd, BL=Fwd)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Modes (Run Without Encoder for Custom PID)
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero Power Behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init PIDs
        flPID = new PIDController(kP, kI, kD, kF);
        frPID = new PIDController(kP, kI, kD, kF);
        blPID = new PIDController(kP, kI, kD, kF);
        brPID = new PIDController(kP, kI, kD, kF);
    }

    public void manualDrive(Gamepad gamepad) {
        // --- 1. Map Controls ---
        // Axial: Left Stick Y (Up is negative, so invert)
        double y = -gamepad.left_stick_x;

        // Lateral: Triggers (Right = Right, Left = Left)
        double strafeRight = gamepad.left_trigger;
        double strafeLeft = gamepad.right_trigger;
        double x = strafeRight - strafeLeft;

        // Yaw: Right Stick X
        double rx = gamepad.right_stick_y;

        // --- 2. Calculate Kinematics (MecanumPIDDrive Logic) ---
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        // Normalize
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // --- 3. PID Control (MecanumPIDDrive Logic) ---
        double flTarget = fl * MAX_TICKS_PER_SEC;
        double frTarget = fr * MAX_TICKS_PER_SEC;
        double blTarget = bl * MAX_TICKS_PER_SEC;
        double brTarget = br * MAX_TICKS_PER_SEC;

        double flCurrent = leftFront.getVelocity();
        double frCurrent = rightFront.getVelocity();
        double blCurrent = leftBack.getVelocity();
        double brCurrent = rightBack.getVelocity();

        double flPower = flPID.calculate(flCurrent, flTarget);
        double frPower = frPID.calculate(frCurrent, frTarget);
        double blPower = blPID.calculate(blCurrent, blTarget);
        double brPower = brPID.calculate(brCurrent, brTarget);

        leftFront.setPower(flPower);
        rightFront.setPower(frPower);
        leftBack.setPower(blPower);
        rightBack.setPower(brPower);
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // Helpers

    private void setMotorMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    // --- Inner PID Class (from MecanumPIDDrive.java) ---
    public static class PIDController {
        private double kP, kI, kD, kF;
        private double integralSum = 0;
        private double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();
        private static final double INTEGRAL_MAX = 0.3;

        public PIDController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            timer.reset();
        }

        public double calculate(double current, double target) {
            double dt = timer.seconds();
            timer.reset();

            double error = target - current;

            double P = kP * error;

            integralSum += error * dt;
            integralSum = clamp(integralSum, -INTEGRAL_MAX / kI, INTEGRAL_MAX / kI);
            double I = kI * integralSum;

            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            double D = kD * derivative;

            double F = kF * target;

            lastError = error;

            double output = P + I + D + F;
            return clamp(output, -1.0, 1.0);
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
            timer.reset();
        }

        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }
}
