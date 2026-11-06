package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

//@Configurable
@TeleOp (name = "Test456")
public class Test456 extends OpMode {
    Follower follower;
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();

        telemetry.update();
    }
}