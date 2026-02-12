package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Normal Auto - Blue Forward", group = "Auto")
public class NormalAutoBlueForward extends NormalAutonRoutineBase {

    @Override
    protected String routeName() {
        return "Blue Forward";
    }

    @Override
    protected PoseTarget startPose() {
        return new PoseTarget(-63.0, 36.0, 0.0);
    }

    @Override
    protected PoseTarget intakePose() {
        return new PoseTarget(-38.0, 36.0, 0.0);
    }

    @Override
    protected PoseTarget shootPose() {
        return new PoseTarget(-16.0, 30.0, 0.0);
    }
}
