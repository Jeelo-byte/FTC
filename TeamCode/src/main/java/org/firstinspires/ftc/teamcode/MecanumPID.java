package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MecanumPID", group = "Drive")
public class MecanumPID extends LinearOpMode {

    private DcMotorEx frontLeftMotor = null;
    private DcMotorEx backLeftMotor = null;
    private DcMotorEx frontRightMotor = null;
    private DcMotorEx backRightMotor = null;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontleftmotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backleftmotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontrightmotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backrightmotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double forwardInput = gamepad1.right_trigger;
            double backwardInput = gamepad1.left_trigger;

            double axial = forwardInput - backwardInput;
            double lateral = gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            frontLeftMotor.setPower(leftFrontPower);
            frontRightMotor.setPower(rightFrontPower);
            backLeftMotor.setPower(leftBackPower);
            backRightMotor.setPower(rightBackPower);

            telemetry.addData("Controls", "Axial: %.2f, Lat: %.2f, Yaw: %.2f", axial, lateral, yaw);
            telemetry.addData("Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                    leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeftMotor.setZeroPowerBehavior(behavior);
        backLeftMotor.setZeroPowerBehavior(behavior);
        frontRightMotor.setZeroPowerBehavior(behavior);
        backRightMotor.setZeroPowerBehavior(behavior);
    }
}