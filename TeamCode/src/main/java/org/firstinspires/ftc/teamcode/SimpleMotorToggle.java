package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SimpleMotorToggle extends LinearOpMode {

    private DcMotor motor;
    private Servo liftServo;
    
    // Lift servo positions
    // REV Smart Robot Servo: 270° total range mapped to 0.0-1.0
    // 90° counterclockwise = 90°/270° = 0.333
    private static final double LIFT_DOWN = 0.0;      // Starting position (0°)
    private static final double LIFT_UP = 0.45;      // +90° counterclockwise from start
    
    // Toggle variables for motor
    private boolean motorOn = false;
    private boolean previousButtonState = false;
    
    // Toggle variables for lift
    private boolean liftUp = false;
    private boolean previousLiftButtonState = false;

    @Override
    public void runOpMode() {
        
        // Map motor from hardware configuration
        motor = hardwareMap.get(DcMotor.class, "motor");
        
        // Map servo from hardware configuration
        liftServo = hardwareMap.get(Servo.class, "lift_servo");
        
        // Initialize lift to down position
        liftServo.setPosition(LIFT_DOWN);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A=Motor Toggle, B=Lift Toggle");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            // --- Motor Toggle (A button) ---
            boolean currentButtonState = gamepad1.a;
            
            // Toggle logic: only trigger on rising edge (button press, not hold)
            if (currentButtonState && !previousButtonState) {
                motorOn = !motorOn; // Flip the state
            }
            
            // Update previous button state for next loop
            previousButtonState = currentButtonState;
            
            // Set motor power based on toggle state
            if (motorOn) {
                motor.setPower(1); // 75% power forward
            } else {
                motor.setPower(0.0); // Stop
            }
            
            // --- Lift Toggle (B button) ---
            // First press: lift up, Second press: lift down
            boolean currentLiftButtonState = gamepad1.b;
            
            if (currentLiftButtonState && !previousLiftButtonState) {
                liftUp = !liftUp; // Toggle the state
                
                if (liftUp) {
                    // Move lift up (counterclockwise 90°)
                    liftServo.setPosition(LIFT_UP);
                    sleep(800); // Wait for servo to reach position
                } else {
                    // Move lift back down to starting position
                    liftServo.setPosition(LIFT_DOWN);
                    sleep(800); // Wait for servo to reach position
                }
            }
            
            // Update previous lift button state for next loop
            previousLiftButtonState = currentLiftButtonState;
            
            // Display status
            telemetry.addData("Motor Status", motorOn ? "ON (75%)" : "OFF");
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Lift Status", liftUp ? "UP (+90°)" : "DOWN (0°)");
            telemetry.addData("Lift Position", "%.3f", liftServo.getPosition());
            telemetry.update();
        }
    }
}
