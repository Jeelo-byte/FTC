package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class AutoBase extends LinearOpMode {

    // Drive motors
    protected DcMotorEx leftFront, rightFront, leftBack, rightBack;
    protected GoBildaPinpointDriver pinpoint;

    // Mechanisms
    protected DcMotorEx intakeMotor;
    protected DcMotorEx shooterMotor;
    protected DcMotorEx spindexerMotor;
    protected CRServo liftServo;
    protected ColorSensor colorSensor;

    // Pose controller constants (tune these on-field)
    protected static final double AUTO_TRANSLATION_KP = 0.06;
    protected static final double AUTO_HEADING_KP = 0.015;
    protected static final double AUTO_MAX_TURN_POWER = 0.55;
    protected static final double AUTO_POSITION_TOLERANCE_IN = 0.75;
    protected static final double AUTO_HEADING_TOLERANCE_DEG = 2.0;
    protected static final double AUTO_DEFAULT_TIMEOUT_S = 4.0;
    protected static final double AUTO_MIN_POWER = 0.08;

    // Pinpoint config for 1x 4-bar odometry pack (tunable)
    // X offset: left +, right -, Y offset: forward +, backward -
    protected double pinpointXPodOffsetMm = -84.0;
    protected double pinpointYPodOffsetMm = -168.0;
    protected GoBildaPinpointDriver.EncoderDirection pinpointXDir =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    protected GoBildaPinpointDriver.EncoderDirection pinpointYDir =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Motor mechanism constants
    protected static final double SHOOTER_POWER = 1.0;
    protected static final int SHOOTER_SPINUP_TIME = 1000;
    protected static final double SPIN_POWER = 0.3;
    protected static final long LIFT_TIME = 220;
    protected static final double LIFT_UP_POWER = -1.0;
    protected static final double LIFT_DOWN_POWER = 1.0;
    protected static final double SPINDEXER_TICKS_PER_REV = 288.0;
    protected static final double SPINDEXER_GEAR_RATIO = 1.0;
    protected static final double SPINDEXER_ENCODER_SIGN = -1.0;

    protected void initHardware() {
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right");
        leftBack = hardwareMap.get(DcMotorEx.class, "back_left");
        rightBack = hardwareMap.get(DcMotorEx.class, "back_right");

        // Match drivetrain config in MecanumDriveSubsystem
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        configurePinpointMounting();
        initPinpoint();
        initOptionalMechanisms();
    }

    /**
     * Override in a subclass to set tuned pod offsets/directions before Pinpoint init.
     */
    protected void configurePinpointMounting() {
        // Defaults above; override if needed.
    }

    protected void setPinpointMounting(double xOffsetMm, double yOffsetMm,
                                       GoBildaPinpointDriver.EncoderDirection xDirection,
                                       GoBildaPinpointDriver.EncoderDirection yDirection) {
        pinpointXPodOffsetMm = xOffsetMm;
        pinpointYPodOffsetMm = yOffsetMm;
        pinpointXDir = xDirection;
        pinpointYDir = yDirection;
    }

    private void initPinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(pinpointXPodOffsetMm, pinpointYPodOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(pinpointXDir, pinpointYDir);
        pinpoint.resetPosAndIMU();
        setStartPose(0.0, 0.0, 0.0);
    }

    private void initOptionalMechanisms() {
        intakeMotor = getOptional(DcMotorEx.class, "intake_motor");
        shooterMotor = getOptional(DcMotorEx.class, "flywheel");
        spindexerMotor = getOptional(DcMotorEx.class, "core_hex");
        if (spindexerMotor == null) {
            spindexerMotor = getOptional(DcMotorEx.class, "coreHex");
        }
        liftServo = getOptional(CRServo.class, "liftServo");
        if (liftServo == null) {
            liftServo = getOptional(CRServo.class, "lift_servo");
        }
        colorSensor = getOptional(ColorSensor.class, "sensor_color");

        if (intakeMotor != null) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (spindexerMotor != null) {
            spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerMotor.setTargetPosition(0);
            spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (shooterMotor != null) {
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (liftServo != null) {
            liftServo.setPower(0.0);
        }
    }

    private <T> T getOptional(Class<? extends T> deviceClass, String name) {
        try {
            return hardwareMap.get(deviceClass, name);
        } catch (RuntimeException e) {
            return null;
        }
    }

    protected void setStartPose(double xInches, double yInches, double headingDegrees) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                xInches,
                yInches,
                AngleUnit.DEGREES,
                headingDegrees
        ));
        pinpoint.update();
    }

    protected Pose2D getPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    protected void holdHeading(double headingDegrees, long holdMs) {
        long startMs = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startMs) < holdMs) {
            moveToPose(getX(), getY(), headingDegrees, 0.35, AUTO_POSITION_TOLERANCE_IN,
                    AUTO_HEADING_TOLERANCE_DEG, 0.05);
        }
        stopDrive();
    }

    // Backward-compatible wrappers for simple auto scripts
    protected void driveForward(double inches, double power) {
        moveBy(inches, 0.0, 0.0, power, AUTO_DEFAULT_TIMEOUT_S + Math.abs(inches) * 0.08);
    }

    protected void driveForward(double inches) {
        driveForward(inches, 0.55);
    }

    protected void driveBackward(double inches, double power) {
        driveForward(-inches, power);
    }

    protected void driveBackward(double inches) {
        driveBackward(inches, 0.55);
    }

    protected void turn(double degrees, double power) {
        // Legacy convention: positive turn value means turn right.
        moveBy(0.0, 0.0, -degrees, Math.abs(power), AUTO_DEFAULT_TIMEOUT_S);
    }

    protected void turn(double degrees) {
        turn(degrees, 0.45);
    }

    protected void strafeRight(double inches, double power) {
        // Pinpoint Y increases to the left, so right is negative Y.
        moveBy(0.0, -inches, 0.0, power, AUTO_DEFAULT_TIMEOUT_S + Math.abs(inches) * 0.08);
    }

    protected void strafeRight(double inches) {
        strafeRight(inches, 0.55);
    }

    protected void strafeLeft(double inches, double power) {
        moveBy(0.0, inches, 0.0, power, AUTO_DEFAULT_TIMEOUT_S + Math.abs(inches) * 0.08);
    }

    protected void strafeLeft(double inches) {
        strafeLeft(inches, 0.55);
    }

    // Fundamental autonomous API for path scripting
    protected boolean moveBy(double deltaXInches, double deltaYInches, double deltaHeadingDeg,
                             double maxDrivePower, double timeoutSeconds) {
        Pose2D pose = getPose();
        return moveToPose(
                pose.getX(DistanceUnit.INCH) + deltaXInches,
                pose.getY(DistanceUnit.INCH) + deltaYInches,
                normalizeDegrees(pose.getHeading(AngleUnit.DEGREES) + deltaHeadingDeg),
                maxDrivePower,
                AUTO_POSITION_TOLERANCE_IN,
                AUTO_HEADING_TOLERANCE_DEG,
                timeoutSeconds
        );
    }

    protected boolean moveToPose(double targetXInches, double targetYInches, double targetHeadingDeg,
                                 double maxDrivePower, double posToleranceInches,
                                 double headingToleranceDeg, double timeoutSeconds) {
        long startTimeMs = System.currentTimeMillis();
        maxDrivePower = clip(Math.abs(maxDrivePower), 0.0, 1.0);

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double currentX = pose.getX(DistanceUnit.INCH);
            double currentY = pose.getY(DistanceUnit.INCH);
            double currentHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
            double currentHeadingRad = Math.toRadians(currentHeadingDeg);

            double errorXField = targetXInches - currentX;
            double errorYField = targetYInches - currentY;
            double distanceError = Math.hypot(errorXField, errorYField);
            double headingErrorDeg = angleErrorDegrees(targetHeadingDeg, currentHeadingDeg);
            double headingErrorRad = Math.toRadians(headingErrorDeg);

            boolean atPosition = distanceError <= posToleranceInches;
            boolean atHeading = Math.abs(headingErrorDeg) <= headingToleranceDeg;
            boolean timedOut = (System.currentTimeMillis() - startTimeMs) / 1000.0 >= timeoutSeconds;

            if ((atPosition && atHeading) || timedOut) {
                stopDrive();
                sendPoseTelemetry("Move Complete", targetXInches, targetYInches, targetHeadingDeg,
                        distanceError, headingErrorDeg, timedOut);
                return !timedOut;
            }

            // Convert field-frame translation error into robot-frame commands
            double errorForwardRobot = Math.cos(currentHeadingRad) * errorXField
                    + Math.sin(currentHeadingRad) * errorYField;
            double errorLeftRobot = -Math.sin(currentHeadingRad) * errorXField
                    + Math.cos(currentHeadingRad) * errorYField;

            double driveCmd = clip(errorForwardRobot * AUTO_TRANSLATION_KP, -maxDrivePower, maxDrivePower);
            double strafeCmd = clip(errorLeftRobot * AUTO_TRANSLATION_KP, -maxDrivePower, maxDrivePower);
            double turnCmd = clip(headingErrorRad * AUTO_HEADING_KP, -AUTO_MAX_TURN_POWER, AUTO_MAX_TURN_POWER);

            // Prevent stalling near friction threshold while still converging smoothly
            driveCmd = applyMinPower(driveCmd, AUTO_MIN_POWER);
            strafeCmd = applyMinPower(strafeCmd, AUTO_MIN_POWER);
            turnCmd = applyMinPower(turnCmd, 0.04);

            setDrivePower(driveCmd, strafeCmd, turnCmd);
            sendPoseTelemetry("Moving", targetXInches, targetYInches, targetHeadingDeg,
                    distanceError, headingErrorDeg, false);
            idle();
        }
        stopDrive();
        return false;
    }

    protected void stopDrive() {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
    }

    protected double getX() {
        return getPose().getX(DistanceUnit.INCH);
    }

    protected double getY() {
        return getPose().getY(DistanceUnit.INCH);
    }

    protected double getHeadingDegrees() {
        return getPose().getHeading(AngleUnit.DEGREES);
    }

    private void setDrivePower(double drive, double strafeLeft, double turnLeft) {
        double fl = drive + strafeLeft + turnLeft;
        double fr = drive - strafeLeft - turnLeft;
        double bl = drive - strafeLeft + turnLeft;
        double br = drive + strafeLeft - turnLeft;

        double maxMag = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br)))));
        leftFront.setPower(fl / maxMag);
        rightFront.setPower(fr / maxMag);
        leftBack.setPower(bl / maxMag);
        rightBack.setPower(br / maxMag);
    }

    private void sendPoseTelemetry(String mode, double targetX, double targetY, double targetHeading,
                                   double distanceError, double headingError, boolean timedOut) {
        telemetry.addData("Auto", mode);
        telemetry.addData("Pose (X,Y,H)", "%.1f, %.1f, %.1f", getX(), getY(), getHeadingDegrees());
        telemetry.addData("Target (X,Y,H)", "%.1f, %.1f, %.1f", targetX, targetY, targetHeading);
        telemetry.addData("Error (Dist,Head)", "%.2f in, %.2f deg", distanceError, headingError);
        telemetry.addData("Timed Out", timedOut);
        telemetry.update();
    }

    private double applyMinPower(double value, double minMagnitude) {
        if (Math.abs(value) < 1e-6) {
            return 0.0;
        }
        if (Math.abs(value) < minMagnitude) {
            return Math.copySign(minMagnitude, value);
        }
        return value;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double normalizeDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees < -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    private double angleErrorDegrees(double targetDeg, double currentDeg) {
        return normalizeDegrees(targetDeg - currentDeg);
    }

    // Mechanism Actions
    protected void shootAll() {
        if (shooterMotor == null || liftServo == null) {
            telemetry.addLine("Shooter hardware not fully configured.");
            telemetry.update();
            return;
        }

        // Spin up shooter
        shooterMotor.setPower(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_TIME);

        // Shoot 3 balls (assuming 3 slots)
        for (int i = 0; i < 3; i++) {
            // Lift and Shoot
            liftServo.setPower(LIFT_UP_POWER);
            sleep(LIFT_TIME);
            liftServo.setPower(0.0);
            sleep(100);

            liftServo.setPower(LIFT_DOWN_POWER);
            sleep(LIFT_TIME);
            liftServo.setPower(0.0);
            sleep(100);

            // Rotate Spindexer for next ball (if not last shot)
            if (i < 2) {
                indexSpindexerDegrees(120.0);
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
        if (shooterMotor == null || liftServo == null) {
            telemetry.addLine("Shooter hardware not fully configured.");
            telemetry.update();
            return;
        }

        // Spin up
        shooterMotor.setPower(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_TIME);

        for (int i = 0; i < shots; i++) {
            // Lift
            liftServo.setPower(LIFT_UP_POWER);
            sleep(LIFT_TIME);
            liftServo.setPower(0.0);
            sleep(100);

            liftServo.setPower(LIFT_DOWN_POWER);
            sleep(LIFT_TIME);
            liftServo.setPower(0.0);
            sleep(100);

            // Rotate Spindexer if needed for next shot
            if (i < shots - 1) {
                indexSpindexerDegrees(120.0);
            }
        }
        shooterMotor.setPower(0);
    }

    protected void indexSpindexerDegrees(double degrees) {
        if (spindexerMotor != null) {
            int deltaTicks = (int) Math.round(
                    (degrees * SPINDEXER_ENCODER_SIGN / 360.0) * SPINDEXER_TICKS_PER_REV * SPINDEXER_GEAR_RATIO
            );
            int target = spindexerMotor.getCurrentPosition() + deltaTicks;
            spindexerMotor.setTargetPosition(target);
            spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexerMotor.setPower(Math.abs(SPIN_POWER));

            long startMs = System.currentTimeMillis();
            while (opModeIsActive() && spindexerMotor.isBusy() && System.currentTimeMillis() - startMs < 1200) {
                idle();
            }
            spindexerMotor.setPower(0.0);
        }
    }
}
