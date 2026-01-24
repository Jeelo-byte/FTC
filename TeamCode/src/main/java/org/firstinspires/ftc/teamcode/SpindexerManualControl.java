package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "DoesntWork", group = "TeleOp")
public class SpindexerManualControl extends OpMode {
    
    private CRServo spindexerServo;
    
    // Rotation constants
    private static final double SPIN_POWER = 0.3;        // Power for continuous rotation
    private static final long ROTATION_TIME_60 = 167;    // Time in ms for 60° rotation (adjust as needed)
    
    // State tracking
    private boolean isSpinning = false;
    private boolean aButtonPressed = false;    // For 60° rotation
    private boolean bButtonPressed = false;    // For continuous spin toggle
    
    // Total rotation tracking
    private double totalRotationDegrees = 0.0;
    
    @Override
    public void init() {
        // Initialize spindexer servo (continuous rotation)
        spindexerServo = hardwareMap.get(CRServo.class, "spindexer_servo");
        spindexerServo.setPower(0); // Start stopped
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A = +60° | B = Toggle Spin");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        
        // --- 60 DEGREE ROTATION (A BUTTON) ---
        // Press A once to rotate 60 degrees clockwise
        boolean aButtonCurrent = gamepad1.a;
        
        if (aButtonCurrent && !aButtonPressed && !isSpinning) {
            // Button just pressed - rotate 60 degrees
            rotate60Degrees();
            aButtonPressed = true;
        } else if (!aButtonCurrent) {
            aButtonPressed = false;
        }
        
        // --- CONTINUOUS SPIN TOGGLE (B BUTTON) ---
        // Press B to toggle continuous spinning on/off
        boolean bButtonCurrent = gamepad1.b;
        
        if (bButtonCurrent && !bButtonPressed) {
            // Toggle spinning state
            isSpinning = !isSpinning;
            
            if (isSpinning) {
                spindexerServo.setPower(SPIN_POWER);
                telemetry.addData("Action", "Continuous spin ON");
            } else {
                spindexerServo.setPower(0);
                telemetry.addData("Action", "Continuous spin OFF");
            }
            
            bButtonPressed = true;
        } else if (!bButtonCurrent) {
            bButtonPressed = false;
        }
        
        // Display telemetry
        telemetry.addData("Mode", isSpinning ? "SPINNING" : "STOPPED");
        telemetry.addData("Total Rotation", "%.1f°", totalRotationDegrees);
        telemetry.addData("Servo Power", spindexerServo.getPower());
        telemetry.addLine();
        telemetry.addData("Controls", "A = Rotate +60°");
        telemetry.addData("Controls", "B = Toggle Continuous Spin");
        telemetry.addLine();
        telemetry.addData("Note", "Calibrate ROTATION_TIME_60 if 60° is inaccurate");
        telemetry.update();
    }
    
    // Rotate spindexer 60 degrees clockwise
    private void rotate60Degrees() {
        telemetry.addData("Action", "Rotating +60°...");
        telemetry.update();
        
        // Spin for calculated time
        spindexerServo.setPower(SPIN_POWER);
        sleep(ROTATION_TIME_60);
        spindexerServo.setPower(0);
        
        // Update total rotation
        totalRotationDegrees += 60.0;
        totalRotationDegrees = totalRotationDegrees % 360.0; // Keep within 0-360
        
        telemetry.addData("Complete", "Now at %.1f°", totalRotationDegrees);
        telemetry.update();
    }
    
    // Sleep helper method
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
