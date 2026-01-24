package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutoBase;

@Autonomous(name = "Leave and Shoot 51in", group = "Auto")
public class LeaveAndShootAuto extends AutoBase {

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Ready - LEAVE AND SHOOT (Back 51in)");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // "just move it backwards (6 * sqrt(2))/2 and shoot"
            // (6 * sqrt(2)) / 2 ft = 3 * 1.414 ft = 4.24 ft = ~51 inches

            // Move backwards 51 inches
            driveForward(-51, 0.5);

            // Shoot all balls
            //shootAll();
        }
    }
}
