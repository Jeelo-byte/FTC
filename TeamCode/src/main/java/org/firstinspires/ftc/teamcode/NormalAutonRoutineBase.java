package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Fundamentals-first autonomous routine built on AutoBase + Pinpoint.
 *
 * Assumptions:
 * - Pinpoint tracking point is at robot center.
 * - Robot footprint is 18in x 18in.
 * - Spindexer intake slots are at 0/120/240 and shoot alignments are 60/180/300.
 * - Naming perspective is while facing the small triangle:
 *   blue side is field-left (x < 0), red side is field-right (x > 0),
 *   and forward/backward correspond to +y/-y start lanes.
 */
public abstract class NormalAutonRoutineBase extends AutoBase {

    protected static final double ROBOT_SIZE_IN = 18.0;
    protected static final double ROBOT_HALF_IN = ROBOT_SIZE_IN / 2.0;
    public static boolean ENABLE_INTAKE_AUTON = false;

    // Tune these live as you brute-force your pinpoint mount.
    protected static double TUNE_PINPOINT_X_OFFSET_MM = -84.0;
    protected static double TUNE_PINPOINT_Y_OFFSET_MM = -168.0;
    protected static GoBildaPinpointDriver.EncoderDirection TUNE_PINPOINT_X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    protected static GoBildaPinpointDriver.EncoderDirection TUNE_PINPOINT_Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private static final double MOVE_POWER = 0.55;
    private static final double INTAKE_SWEEP_IN = 6.0;
    private static final long INTAKE_TIMEOUT_MS = 9000;
    private static final long INTAKE_SETTLE_MS = 250;
    private static final long FLYWHEEL_SPINUP_MS = 1200;
    private static final long SHOT_GAP_MS = 180;

    private SpindexerSubsystem spindexer;
    private ShooterLiftSubsystem shooterLift;

    protected static class PoseTarget {
        final double xIn;
        final double yIn;
        final double headingDeg;

        PoseTarget(double xIn, double yIn, double headingDeg) {
            this.xIn = xIn;
            this.yIn = yIn;
            this.headingDeg = headingDeg;
        }
    }

    protected abstract String routeName();

    protected abstract PoseTarget startPose();

    protected abstract PoseTarget intakePose();

    protected abstract PoseTarget shootPose();

    @Override
    protected void configurePinpointMounting() {
        setPinpointMounting(
                TUNE_PINPOINT_X_OFFSET_MM,
                TUNE_PINPOINT_Y_OFFSET_MM,
                TUNE_PINPOINT_X_DIR,
                TUNE_PINPOINT_Y_DIR
        );
    }

    @Override
    public final void runOpMode() {
        initHardware();
        spindexer = new SpindexerSubsystem(hardwareMap, ENABLE_INTAKE_AUTON);
        shooterLift = new ShooterLiftSubsystem(hardwareMap);

        PoseTarget start = startPose();
        setStartPose(start.xIn, start.yIn, start.headingDeg);

        telemetry.addData("Auto Route", routeName());
        telemetry.addData("Intake Enabled", ENABLE_INTAKE_AUTON);
        telemetry.addData("Start Pose", "(%.1f, %.1f, %.1f)", start.xIn, start.yIn, start.headingDeg);
        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        runNormalAuton();
    }

    private void runNormalAuton() {
        PoseTarget intake = intakePose();
        PoseTarget shoot = shootPose();

        moveToPose(intake.xIn, intake.yIn, intake.headingDeg, MOVE_POWER,
                AUTO_POSITION_TOLERANCE_IN, AUTO_HEADING_TOLERANCE_DEG, 5.0);

        collectThreeBallsWithColorSorting();

        moveToPose(shoot.xIn, shoot.yIn, shoot.headingDeg, MOVE_POWER,
                AUTO_POSITION_TOLERANCE_IN, AUTO_HEADING_TOLERANCE_DEG, 5.0);

        shootLoadedBalls();
        stopDrive();
    }

    private void collectThreeBallsWithColorSorting() {
        if (!ENABLE_INTAKE_AUTON || !spindexer.isIntakeEnabled()) {
            return;
        }

        spindexer.resetSlots();
        spindexer.startAutoIntake();

        ElapsedTime timer = new ElapsedTime();
        boolean sweepForward = true;

        while (opModeIsActive()
                && spindexer.getBallsLoaded() < 3
                && timer.milliseconds() < INTAKE_TIMEOUT_MS) {

            moveBy(sweepForward ? INTAKE_SWEEP_IN : -INTAKE_SWEEP_IN, 0.0, 0.0, 0.32, 1.2);
            sweepForward = !sweepForward;

            long settleStart = System.currentTimeMillis();
            while (opModeIsActive()
                    && spindexer.getBallsLoaded() < 3
                    && (System.currentTimeMillis() - settleStart) < INTAKE_SETTLE_MS) {
                spindexer.updateAutoIntake(telemetry, this::idle, true);
                telemetry.addData("Phase", "Intaking");
                telemetry.addData("Balls Loaded", spindexer.getBallsLoaded());
                telemetry.update();
                idle();
            }
        }

        spindexer.stopAutoIntake();
    }

    private void shootLoadedBalls() {
        int shots = Math.min(3, spindexer.getBallsLoaded());
        if (shots <= 0 || !opModeIsActive()) {
            return;
        }

        shooterLift.setShooterPower(1.0);
        sleep(FLYWHEEL_SPINUP_MS);

        // Intake slots: 0,120,240. Shooting slots: 60,180,300.
        double[] indexDegrees = {60.0, 120.0, 120.0};
        double totalRotation = 0.0;

        for (int i = 0; i < shots && opModeIsActive(); i++) {
            double step = indexDegrees[i];
            spindexer.moveSpindexerAutoDegrees(step, this::idle, true);
            totalRotation += step;

            shooterLift.toggleLift(this::idle, true); // up: fire
            sleep(100);
            shooterLift.toggleLift(this::idle, true); // down: reset
            sleep(SHOT_GAP_MS);
        }

        shooterLift.setShooterPower(0.0);
        spindexer.moveSpindexerAutoDegrees(-totalRotation, this::idle, true);
    }
}
