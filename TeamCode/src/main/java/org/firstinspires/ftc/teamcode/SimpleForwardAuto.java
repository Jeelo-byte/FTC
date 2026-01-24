package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "SimpleForwardAuto")
public class SimpleForwardAuto extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Reverse left drive to match your TeleOp setup
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            // Drive forward
            leftDrive.setPower(0.5);
            rightDrive.setPower(0.5);

            sleep(1000); // DRIVE FORWARD FOR 1 SECOND

            // Stop
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }
}
