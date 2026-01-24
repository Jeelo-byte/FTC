package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "SimpleAutoShoot")
public class SimpleAutoShoot extends LinearOpMode {

  // --- Hardware Definitions ---

  private DcMotor flywheel; // Shooter motor

  private DcMotor coreHex; // Feeder motor

  private DcMotor leftDrive; // Left drive motor

  private CRServo servo; // Hopper agitator servo

  private DcMotor rightDrive; // Right drive motor

  // --- Flywheel velocities in encoder ticks per second ---

    private static final int bankVelocity = 1400;

    private static final int farVelocity = 1900;

    private static final int maxVelocity = 1950;

  @Override
  public void runOpMode() throws InterruptedException {
    leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
    rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

    // Reverse left drive to match your TeleOp setup
    leftDrive.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {

      // Drive forward
      leftDrive.setPower(-0.5);
      rightDrive.setPower(-0.5);

      sleep(1000); // DRIVE FORWARD FOR 1 SECOND

      // Stop
      leftDrive.setPower(0);
      rightDrive.setPower(0);

      // Turn
      leftDrive.setPower(0.25);
      rightDrive.setPower(-0.25);
      sleep(100);
      
      farPowerAuto();
      sleep(500);
      farPowerAuto();
    }
  }

  private void farPowerAuto() {
    ((DcMotorEx) flywheel).setVelocity(farVelocity);
    servo.setPower(-1);
    sleep(1000);
    if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 100) {
      coreHex.setPower(1); // Feed balls only when flywheel is at speed
      sleep(2500);
    } else {
      coreHex.setPower(0);
    }
  }
}
