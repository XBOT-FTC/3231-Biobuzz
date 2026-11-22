package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous (name="LeaveAuto", group="Autonomous")
public class LeaveAuto extends LinearOpMode {
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    public void runOpMode() throws InterruptedException{
        frontLeftDrive  = hardwareMap.get(DcMotor.class, Constants.leftFront());
        frontRightDrive  = hardwareMap.get(DcMotor.class, Constants.rightFront());
        backLeftDrive  = hardwareMap.get(DcMotor.class, Constants.leftRear());
        backRightDrive  = hardwareMap.get(DcMotor.class, Constants.rightRear());

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        backRightDrive.setPower(.5);
        backLeftDrive.setPower(.5);
        frontRightDrive.setPower(.5);
        frontLeftDrive.setPower(.5);

        telemetry.addData("Positions", "frontLeft (%d), backLeft (%d), frontRight (%d), backRight (%d)", frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
        telemetry.update();

        moveStop();
        moveLinear(.3,2000, "Forward");
        moveStop();


    }

    public void setPower(double FLspeed, double FRspeed, double BLspeed, double BRspeed) {
        frontLeftDrive.setPower(FLspeed);
        frontRightDrive.setPower(FRspeed);
        backLeftDrive.setPower(BLspeed);
        backRightDrive.setPower(BRspeed);

    }
        public void moveLinear(double speed, int time, String direction) throws InterruptedException {
        if (direction.equalsIgnoreCase("Forward")) {
            setPower(speed, speed, speed, speed);
        }
        if (direction.equalsIgnoreCase("Backward")) {
            setPower(-speed, -speed, -speed, -speed);
        }
        if (direction.equalsIgnoreCase("Left")) {
            setPower(-speed, speed, speed, -speed);
        }
        if (direction.equalsIgnoreCase("Right")) {
            setPower(speed, -speed, -speed, speed);
        }
        Thread.sleep(time);
    }

    public void moveStop() {
        setPower(0,0,0,0);
    }


}
