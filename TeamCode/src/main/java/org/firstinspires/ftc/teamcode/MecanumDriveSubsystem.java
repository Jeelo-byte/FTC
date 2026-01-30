package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumDriveSubsystem {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // PID Constants
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.00036;
    private static final double MAX_TICKS_PER_SEC = 2796;

    private PIDController flPID, frPID, blPID, brPID;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        initHardware(hardwareMap);
    }

    private void initHardware(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right");
        leftBack = hardwareMap.get(DcMotorEx.class, "back_left");
        rightBack = hardwareMap.get(DcMotorEx.class, "back_right");

        // Directions (Matching old MotorControl)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // Others default to FORWARD

        // Modes
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // PIDs
        flPID = new PIDController(kP, kI, kD, kF);
        frPID = new PIDController(kP, kI, kD, kF);
        blPID = new PIDController(kP, kI, kD, kF);
        brPID = new PIDController(kP, kI, kD, kF);
    }

    public void manualDrive(Gamepad gamepad) {
        double y = gamepad.left_trigger - gamepad.right_trigger;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Apply PID
        setMotorPowersPID(fl, fr, bl, br);
    }

    private void setMotorPowersPID(double fl, double fr, double bl, double br) {
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

    // Inner PID Class
    private static class PIDController {
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
            integralSum = Math.max(-INTEGRAL_MAX / kI, Math.min(INTEGRAL_MAX / kI, integralSum));
            double I = kI * integralSum;

            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            double D = kD * derivative;

            double F = kF * target;

            lastError = error;

            double output = P + I + D + F;
            return Math.max(-1.0, Math.min(1.0, output));
        }
    }
}
