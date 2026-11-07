package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TeamDriveTrain;

@TeleOp(name = "2026TeleOp_2939", group="OpMode")
public class MecanumTeleOp_2939 extends OpMode {
    private TeamDriveTrain drive;
    private Shooter shooterSystem;
    private Follower follower;

    private Gamepad driverGamepad;
    private Gamepad opGamepad;

    public void init() {
        driverGamepad = gamepad1;
        opGamepad = gamepad2;

        drive = new TeamDriveTrain(hardwareMap);
        shooterSystem = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -driverGamepad.left_stick_y,
                driverGamepad.left_stick_x,
                driverGamepad.right_stick_x,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers);
        follower.update();

        //Pose robotPose = follower.pose();

        if (opGamepad.rightTriggerWasPressed()) {
            shooterSystem.turnOn().execute();
        }
        else {
            shooterSystem.turnOff().execute();
        }

    }
}