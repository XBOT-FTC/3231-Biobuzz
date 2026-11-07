package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.pedropathing.ivy.commands.Commands.*;

public class Shooter {
    private DcMotorEx motor;
    double previous = 0;

    public Shooter(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, Constants.shooterMotor);
    }

    public Command maintain(double velocity) {
        return infinite(
            () -> {
                double current = motor.getVelocity();
                double error = current - velocity;
                double power = 0.5 * error + 0.05 * (error - previous);

                motor.setPower(power);
                previous = error;
            }
        ).requiring(motor);
    }

    public Command turnOn() {
        return instant(() -> motor.setPower(1));
    }

    public Command turnOff() {
        return instant(() -> motor.setPower(0));
    }
}
