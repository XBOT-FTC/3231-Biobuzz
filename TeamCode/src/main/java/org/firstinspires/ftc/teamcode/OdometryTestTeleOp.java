package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OdometryTestTeleOp", group = "OpMode")
public class OdometryTestTeleOp extends OpMode {
    Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
    }
}