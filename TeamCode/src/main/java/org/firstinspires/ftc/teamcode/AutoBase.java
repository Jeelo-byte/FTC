package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class AutoBase extends LinearOpMode {

    // Drive motors
    protected DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // Mechanisms
    protected DcMotorEx intakeMotor;
    protected DcMotorEx shooterMotor;
    protected CRServo spindexerServo;
    protected Servo liftServo;
    protected ColorSensor colorSensor;

    // Constants (Copied from MotorControl)
    protected static final double MAX_TICKS_PER_SEC = 2796;
    protected static final double TICKS_PER_REV = 537.7; // GoBilda 5202/5203 series 312 RPM (Approx) - Adjust if needed
    // Assuming 96mm mecanum wheels (approx 3.78 inches diameter)
    // Circumference = 3.78 * PI ~= 11.87 inches
    protected static final double TICKS_PER_INCH = TICKS_PER_REV / (3.78 * Math.PI);

    // Motor mechanism constants
    protected static final double SHOOTER_POWER = 1.0;
    protected static final int SHOOTER_SPINUP_TIME = 1000;
    protected static final double SPIN_POWER = 0.3;
    protected static final double GEAR_RATIO = 15.0 / 20.0;
    protected static final long ROTATION_TIME_120 = (long) (500 * GEAR_RATIO);
    protected static final double LIFT_UP = 0.1;
    protected static final double LIFT_DOWN = 0.7;

    protected void initHardware() {
        // Drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right");
        leftBack = hardwareMap.get(DcMotorEx.class, "back_left");
        rightBack = hardwareMap.get(DcMotorEx.class, "back_right");

        // Reverse left side based on MotorControl configuration
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE); // MotorControl has this reversed too? Let's match
                                                                  // MotorControl exactly.
        // MotorControl:
        // leftFront.setDirection(REVERSE);
        // leftBack.setDirection(REVERSE);
        // rightFront.setDirection(REVERSE);
        // This seems odd (usually right is reversed), but I will trust MotorControl.

        resetEncoders();

        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Servos
        spindexerServo = hardwareMap.get(CRServo.class, "spindexer_servo");
        liftServo = hardwareMap.get(Servo.class, "lift_servo");
        liftServo.setPosition(LIFT_DOWN);

        // Sensors
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
    }

    protected void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        leftBack.setTargetPosition(0); //Hello
        rightBack.setTargetPosition(0);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Basic movement methods
    protected void driveForward(double inches, double power) {
        int target = (int) (inches * TICKS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + target);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + target);

        setRunToPosition();
        setPowers(power);
        waitForMotors();
    }

    protected void strafeRight(double inches, double power) {
        int target = (int) (inches * TICKS_PER_INCH);

        // Strafe right: FL+, FR-, BL-, BR+
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() - target);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + target);

        setRunToPosition();
        setPowers(power);
        waitForMotors();
    }

    protected void strafeLeft(double inches, double power) {
        strafeRight(-inches, power);
    }

    protected void turn(double degrees, double power) {
        // Approximate turning - would need tuning
        // Arc length = degrees/360 * PI * diameter_of_turn
        // Robot width estimation ~ 14-16 inches?
        double ticksPerDegree = 10; // Placeholder value, needs tuning!
        int target = (int) (degrees * ticksPerDegree);

        // Turn right: Left+, Right-
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + target);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() - target);

        setRunToPosition();
        setPowers(power);
        waitForMotors();
    }

    private void setRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setPowers(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    private void waitForMotors() {
        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy())) {
            // Wait
            idle();
        }
        setPowers(0);
    }

    // Mechanism Actions
    protected void shootAll() {
        // Spin up shooter
        shooterMotor.setPower(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_TIME);

        // Shoot 3 balls (assuming 3 slots)
        for (int i = 0; i < 3; i++) {
            // Lift and Shoot
            liftServo.setPosition(LIFT_UP);
            sleep(250);
            liftServo.setPosition(LIFT_DOWN);
            sleep(250);

            // Rotate Spindexer for next ball (if not last shot)
            if (i < 2) {
                spindexerServo.setPower(SPIN_POWER);
                sleep(ROTATION_TIME_120);
                spindexerServo.setPower(0);
                sleep(200); // Wait for settle
            }
        }

        // Stop shooter
        shooterMotor.setPower(0);
    }

    protected void shootAction() {
        shootAction(1);
    }

    protected void shootAction(int shots) {
        // Spin up
        shooterMotor.setPower(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_TIME);

        for (int i = 0; i < shots; i++) {
            // Lift
            liftServo.setPosition(LIFT_UP);
            sleep(250);
            liftServo.setPosition(LIFT_DOWN);
            sleep(250);

            // Rotate Spindexer if needed for next shot
            if (i < shots - 1) {
                spindexerServo.setPower(SPIN_POWER);
                sleep(ROTATION_TIME_120);
                spindexerServo.setPower(0);
            }
        }
        shooterMotor.setPower(0);
    }
}
