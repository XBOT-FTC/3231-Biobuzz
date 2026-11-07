package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.pedropathing.ivy.commands.Commands.*;

public class Shooter {
    private DcMotorEx motor;
    double error;

    public Shooter(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, Constants.shooterMotor);
    }

    public Command maintain(double velocity) {
        error = 0;
        return infinite(
            () -> {
                double current = motor.getVelocity();
                double power = 0.5 * error + 0.05 * (current - error);

                motor.setPower(power);
                error = current;
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
