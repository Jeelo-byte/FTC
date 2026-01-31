package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumPIDDrive", group = "Drive")
public class MecanumPIDDrive extends OpMode {
    
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    PIDController flPID, frPID, blPID, brPID;
    static final double MAX_TICKS_PER_SEC = 2796;
    
    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft   = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight  = hardwareMap.get(DcMotorEx.class, "back_right");
        
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        double kP = 0.0001;
        double kI = 0.0;
        double kD = 0.0;
        double kF = 0.00036;
        
        flPID = new PIDController(kP, kI, kD, kF);
        frPID = new PIDController(kP, kI, kD, kF);
        blPID = new PIDController(kP, kI, kD, kF);
        brPID = new PIDController(kP, kI, kD, kF);
    }
    
    @Override
    public void loop() {
        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.right_stick_x;
        double rx = gamepad1.left_stick_x;
        
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
        
        double flTarget = fl * MAX_TICKS_PER_SEC;
        double frTarget = fr * MAX_TICKS_PER_SEC;
        double blTarget = bl * MAX_TICKS_PER_SEC;
        double brTarget = br * MAX_TICKS_PER_SEC;
        
        double flCurrent = frontLeft.getVelocity();
        double frCurrent = frontRight.getVelocity();
        double blCurrent = backLeft.getVelocity();
        double brCurrent = backRight.getVelocity();
        
        double flPower = flPID.calculate(flCurrent, flTarget);
        double frPower = frPID.calculate(frCurrent, frTarget);
        double blPower = blPID.calculate(blCurrent, blTarget);
        double brPower = brPID.calculate(brCurrent, brTarget);
        
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
        
        telemetry.addData("FL Target/Current", "%.0f / %.0f", flTarget, flCurrent);
        telemetry.addData("FR Target/Current", "%.0f / %.0f", frTarget, frCurrent);
        telemetry.addData("FL Power", "%.3f", flPower);
        telemetry.addData("FL Error", "%.0f", flTarget - flCurrent);
        telemetry.update();
    }
    
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
        
        public void setCoefficients(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }
}


