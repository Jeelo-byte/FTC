package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Red Auto - Ball Collection", group = "Auto")
public class RedAutoBallCollection extends LinearOpMode {
    
    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    
    // Constants - adjust these based on your robot
    private static final double COUNTS_PER_INCH = 50; // Adjust for your drivetrain
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();
        
        telemetry.addData("Status", "Ready to run Red Auto");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Starting position: Far end shooting triangle (red side)
            telemetry.addData("Step", "1: Moving to first ball set");
            telemetry.update();
            
            // Move to first set of balls
            driveForward(48); // Adjust distance as needed
            sleep(200);
            
            // Rotate to put back towards balls (180 degree turn)
            telemetry.addData("Step", "2: Rotating to collect balls");
            telemetry.update();
            turnDegrees(180);
            sleep(200);
            
            // Back up to collect balls
            driveBackward(12); // Adjust to get close to balls
            sleep(500); // Time to collect
            
            // Drive to close shooting triangle
            telemetry.addData("Step", "3: Moving to close shooting triangle");
            telemetry.update();
            driveForward(12); // Move away from balls
            turnDegrees(180); // Face forward again
            driveForward(36); // Move to close triangle
            sleep(500); // Time to shoot/score
            
            // Move to second set of balls
            telemetry.addData("Step", "4: Moving to second ball set");
            telemetry.update();
            driveBackward(24);
            strafeRight(24); // Adjust based on ball spacing
            sleep(200);
            
            // Rotate to put back towards balls
            telemetry.addData("Step", "5: Rotating for second collection");
            telemetry.update();
            turnDegrees(180);
            sleep(200);
            
            // Collect second set
            driveBackward(12);
            sleep(500);
            
            // Return to close shooting triangle
            telemetry.addData("Step", "6: Returning to shooting position");
            telemetry.update();
            driveForward(12);
            turnDegrees(180);
            strafeLeft(24);
            driveForward(24);
            sleep(500);
            
            // Move to third set of balls
            telemetry.addData("Step", "7: Moving to third ball set");
            telemetry.update();
            driveBackward(24);
            strafeRight(48); // Adjust based on ball spacing
            sleep(200);
            
            // Rotate to put back towards balls
            telemetry.addData("Step", "8: Rotating for third collection");
            telemetry.update();
            turnDegrees(180);
            sleep(200);
            
            // Collect third set
            driveBackward(12);
            sleep(500);
            
            // Final return to close shooting triangle
            telemetry.addData("Step", "9: Final return to shooting position");
            telemetry.update();
            driveForward(12);
            turnDegrees(180);
            strafeLeft(48);
            driveForward(24);
            
            telemetry.addData("Status", "Auto Complete!");
            telemetry.update();
        }
    }
    
    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        // Set motor directions (adjust based on your robot)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void driveForward(double inches) {
        int target = (int)(inches * COUNTS_PER_INCH);
        
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + target);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + target);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + target);
        backRight.setTargetPosition(backRight.getCurrentPosition() + target);
        
        setModeRunToPosition();
        setPower(DRIVE_SPEED);
        
        while (opModeIsActive() && motorsAreBusy()) {
            idle();
        }
        
        stopMotors();
    }
    
    private void driveBackward(double inches) {
        driveForward(-inches);
    }
    
    private void strafeRight(double inches) {
        int target = (int)(inches * COUNTS_PER_INCH);
        
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + target);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - target);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - target);
        backRight.setTargetPosition(backRight.getCurrentPosition() + target);
        
        setModeRunToPosition();
        setPower(DRIVE_SPEED);
        
        while (opModeIsActive() && motorsAreBusy()) {
            idle();
        }
        
        stopMotors();
    }
    
    private void strafeLeft(double inches) {
        strafeRight(-inches);
    }
    
    private void turnDegrees(double degrees) {
        // Approximate: adjust multiplier based on your robot's turning circle
        int target = (int)(degrees * 10); // You'll need to calibrate this
        
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + target);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - target);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + target);
        backRight.setTargetPosition(backRight.getCurrentPosition() - target);
        
        setModeRunToPosition();
        setPower(TURN_SPEED);
        
        while (opModeIsActive() && motorsAreBusy()) {
            idle();
        }
        
        stopMotors();
    }
    
    private void setModeRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    private void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    
    private void stopMotors() {
        setPower(0);
    }
    
    private boolean motorsAreBusy() {
        return frontLeft.isBusy() && frontRight.isBusy() && 
               backLeft.isBusy() && backRight.isBusy();
    }
}

