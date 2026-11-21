package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name = "RedShootAndPark", group = "Autonomous")
public class RedShootAndPark extends LinearOpMode {

    private Follower follower;

    private DcMotorEx shooter;
    private CRServo windmillServo;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose path1 = poseFactory.of(34.1999, 28.9116, 180);
    private final Pose point2 = poseFactory.of(9.857, 116.0392, -74.3899);

    // Autonomous routine
    public Command autoRoutine() {
        shooter.setVelocity(1170);
        windmillServo.setPower(1.0);
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.update();
        }

        shooter.setVelocity(0);
        windmillServo.setPower(0);
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }
}