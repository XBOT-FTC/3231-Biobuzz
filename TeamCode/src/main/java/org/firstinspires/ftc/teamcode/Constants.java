package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(Constants.leftFront());
                c.backLeftName.set(Constants.leftRear());
                c.frontRightName.set(Constants.rightFront());
                c.backRightName.set(Constants.rightRear());

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.1536603912593812);
        c.yPodOffset.set(-3.9543314055195005);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3302734930083007);
                Controller secondaryTranslationalForward = Controller.proportional(0.12202723059630029);
                Controller primaryTranslationalLateral = Controller.proportional(2.721048948790103);
                Controller secondaryTranslationalLateral = Controller.proportional(1.0053548788109534);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.02708515020877627));
                c.brake.set(Controller.proportionalFeedforward(0.023022377677459828));

                c.headingFeedback.set(Controller.proportional(3.5511303262440097));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04731003932347212, 0.005686034478715731));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06693390719114989, 0.07410753614376389));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002294798587098989, 0.00191725406948465));

                c.maxAchievableForwardVelocity.set(39.560273725837405);
                c.maxAchievableStrafeVelocity.set(29.334466201232978);
                c.naturalForwardDeceleration.set(25.894928433858162);
                c.naturalStrafeDeceleration.set(47.14723081935718);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static String leftFront(){
        return "frontLeftMotor";
    }
    public static String leftRear(){
        return "backLeftMotor";
    }
    public static String rightFront() {
        return "frontRightMotor";
    }
    public static String rightRear() {
        return "backRightMotor";
    }


    public static Follower createFollower(HardwareMap h, Drivetrain drivetrain, Localizer localizer, Foresight foresight) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return new Follower(localizer,drivetrain,foresight);
    }

    public static double inPerTick3231 = 0.001979;
    public static double lateralInPerTick3231 = 0.001979;

    public static double ks3231 = 1.0967513649697667;

    public static double kv3231 = 0.0042730455553834186;

    public static double ka3231 = .00067;

    public static double trackWidth3231 = 1226.5643209432749;

    public static double axialGain3231 = 3;

    public static double axialVelGain3231 = 1.9;

    public static double headingGain3231 = 4.5;

    public static double headingVelGain3231 = .5;

    public static double lateralGain3231 = 1.3;

    public static double lateralVelGain3231 = 0;

    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        public double inPerTick = Constants.inPerTick3231;
        public double lateralInPerTick = Constants.lateralInPerTick3231;
        public double trackWidthTicks = Constants.trackWidth3231;

        // feedforward parameters (in tick units)
        public double kS = Constants.ks3231;
        public double kV = Constants.kv3231;
        public double kA = ka3231;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = axialGain3231;
        public double lateralGain = lateralGain3231;
        public double headingGain = headingGain3231; // shared with turn

        public double axialVelGain = axialVelGain3231;
        public double lateralVelGain = lateralVelGain3231;
        public double headingVelGain = headingVelGain3231; // shared with turn
    }

}
//    // This is for pure driver encoder odometry
//    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants() {}
//            .leftFrontMotorName("leftFront")
//            .leftRearMotorName("leftRear")
//            .rightFrontMotorName("rightFront")
//            .rightRearMotorName("rightRear")
//
//            .leftFrontEncoderDirection(Encoder.FORWARD)
//            .leftRearEncoderDirection(Encoder.FORWARD)
//            .rightFrontEncoderDirection(Encoder.FORWARD)
//            .rightRearEncoderDirection(Encoder.FORWARD);



