package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="Spindexer Motor Control")
public class SpindexerControl extends LinearOpMode {
    
    private DcMotor spindexerMotor;
    private Servo liftServo;
    private ColorSensor rampSensor;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    private static final int BASE_TIME_PER_SLOT = 400; 
    private static final double SPIN_POWER = 0.3;
    
    private static final double LIFT_UP = 0.7;
    private static final double LIFT_DOWN = 0.1;
    private static final int LIFT_DELAY = 300;
    
    private static final int COLOR_THRESHOLD = 40;     
    private static final int PRESENCE_THRESHOLD = 100; 

    private BallSlot[] ballSlots = new BallSlot[3];
    private String[] motif = {"P", "P", "G"}; 
    private int currentSlot = 0; 
    private int ballsLoaded = 0;
    private boolean isIntaking = false;
    private boolean isShooting = false;
    
    class BallSlot {
        String color = "";
        boolean isEmpty = true;
    }
    
    @Override
    public void runOpMode() {
        initHardware();
        
        for (int i = 0; i < 3; i++) ballSlots[i] = new BallSlot();
        
        telemetry.addData("Status", "Ready");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            if (gamepad1.a && !isIntaking && !isShooting) {
                if (ballsLoaded < 3) {
                    intakeSequence();
                }
            }
            
            if (gamepad1.b && !isShooting && !isIntaking) {
                shootSequence();
            }

            if (gamepad1.dpad_left) {
                spindexerMotor.setPower(SPIN_POWER);
            } else if (gamepad1.dpad_right) {
                spindexerMotor.setPower(-SPIN_POWER);
            } else {
                if(!isIntaking && !isShooting) spindexerMotor.setPower(0);
            }
            
            updateTelemetry();
        }
        visionPortal.close();
    }
    
    private void intakeSequence() {
        isIntaking = true;
        
        while (ballsLoaded < 3 && opModeIsActive()) {
            
            int targetSlot = -1;
            int slotsToMove = 0;
            
            for (int i = 0; i < 3; i++) {
                int checkIndex = (currentSlot + i) % 3;
                if (ballSlots[checkIndex].isEmpty) {
                    targetSlot = checkIndex;
                    slotsToMove = i;
                    break;
                }
            }
            if (targetSlot == -1) break;
            
            if (slotsToMove > 0) {
                spindexerMotor.setPower(SPIN_POWER);
                long spinTime = (long)(slotsToMove * BASE_TIME_PER_SLOT);
                sleep(spinTime);
                spindexerMotor.setPower(0);
                
                currentSlot = targetSlot;
                sleep(200);
            }
            
            if (!waitUntilClear()) { isIntaking = false; return; }
            
            if (!waitForBall(targetSlot)) { isIntaking = false; return; }
        }
        
        isIntaking = false;
    }
    
    private boolean waitUntilClear() {
        while (opModeIsActive() && rampSensor.alpha() > PRESENCE_THRESHOLD) {
            if (gamepad1.b) return false;
        }
        return true;
    }
    
    private boolean waitForBall(int slotIndex) {
        while (opModeIsActive()) {
            String color = checkColorSensor();
            
            if (!color.equals("UNKNOWN") && !color.equals("NONE")) {
                ballSlots[slotIndex].color = color;
                ballSlots[slotIndex].isEmpty = false;
                ballsLoaded++;
                
                spindexerMotor.setPower(SPIN_POWER);
                sleep(150); 
                spindexerMotor.setPower(0);
                
                return true;
            }
            
            if (gamepad1.b) return false;
        }
        return false;
    }

    private void shootSequence() {
        isShooting = true;
        
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            String targetColor = motif[i];
            
            int slotIndex = -1;
            int dist = 0;
            for(int k=0; k<3; k++) {
                int idx = (currentSlot + k) % 3;
                if(ballSlots[idx].color.equals(targetColor) && !ballSlots[idx].isEmpty) {
                    slotIndex = idx;
                    dist = k;
                    break;
                }
            }
            
            if (slotIndex != -1) {
                if (dist > 0) {
                    spindexerMotor.setPower(SPIN_POWER);
                    long spinTime = (long)(dist * BASE_TIME_PER_SLOT);
                    sleep(spinTime);
                    spindexerMotor.setPower(0);
                    currentSlot = slotIndex;
                }
                
                liftServo.setPosition(LIFT_UP);
                sleep(LIFT_DELAY);
                liftServo.setPosition(LIFT_DOWN);
                sleep(LIFT_DELAY);
                
                ballSlots[slotIndex].isEmpty = true;
                ballSlots[slotIndex].color = "";
            }
        }
        
        isShooting = false;
        ballsLoaded = 0;
    }

    private String checkColorSensor() {
        int r = rampSensor.red();
        int g = rampSensor.green();
        int b = rampSensor.blue();
        
        if (r+g+b < PRESENCE_THRESHOLD) return "NONE";
        if (g > r+COLOR_THRESHOLD && g > b+COLOR_THRESHOLD) return "G";
        if ((r > g && b > g) || (r+b > g*1.5)) return "P";
        return "UNKNOWN";
    }
    
    private void updateTelemetry() {
        telemetry.addData("State", isShooting ? "SHOOTING" : (isIntaking ? "INTAKING" : "IDLE"));
        telemetry.addData("Balls Loaded", ballsLoaded + "/3");
        telemetry.update();
    }

    private void initHardware() {
        spindexerMotor = hardwareMap.get(DcMotor.class, "spindexer_motor");
        liftServo = hardwareMap.get(Servo.class, "lift_servo");
        rampSensor = hardwareMap.get(ColorSensor.class, "ramp_sensor");
        
        spindexerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .build();
        
        liftServo.setPosition(LIFT_DOWN);
        rampSensor.enableLed(true);
        
        spindexerMotor.setPower(0);
    }
}


