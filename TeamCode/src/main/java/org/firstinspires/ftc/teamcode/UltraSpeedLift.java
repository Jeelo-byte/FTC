package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "UltraSpeedLift", group = "TeleOp")
public class UltraSpeedLift extends OpMode {
    
    private Servo liftServo;
    
    // Lift positions (matching MotorControl)
    // IMPORTANT: These are reversed from typical because servo starts at bottom
    private static final double LIFT_UP = 0.31;      // Lifted position (up)
    private static final double LIFT_DOWN = 0.81;    // Down position (bottom/resting)
    
    // State tracking
    private boolean isUp = false;
    private boolean lastButtonState = false;
    
    @Override
    public void init() {
        // Initialize lift servo
        liftServo = hardwareMap.get(Servo.class, "lift_servo");
        
        // Start at bottom position
        liftServo.setPosition(LIFT_DOWN);
        isUp = false;
        
        telemetry.addData("Status", "Initialized - Lift at BOTTOM");
        telemetry.addData("Controls", "Press A to toggle lift");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Toggle lift with A button (debounced)
        boolean currentButtonState = gamepad1.a;
        
        if (currentButtonState && !lastButtonState) {
            // Button just pressed - toggle state
            isUp = !isUp;
            
            if (isUp) {
                // Move lift UP
                liftServo.setPosition(LIFT_UP);
                telemetry.addData("Action", "Lifting UP");
            } else {
                // Move lift DOWN
                liftServo.setPosition(LIFT_DOWN);
                telemetry.addData("Action", "Lowering DOWN");
            }
        }
        
        lastButtonState = currentButtonState;
        
        // Display telemetry
        telemetry.addData("Lift Position", isUp ? "UP (0.1)" : "DOWN (0.7)");
        telemetry.addData("Servo Value", "%.2f", liftServo.getPosition());
        telemetry.addData("Status", isUp ? "LIFTED" : "RESTING");
        telemetry.addLine();
        telemetry.addData("Controls", "A Button = Toggle Lift");
        telemetry.update();
    }
}

