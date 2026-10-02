package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front");
                c.backLeftName.set("left_back");
                c.frontRightName.set("right_front");
                c.backRightName.set("right_back");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodOffset.set(2.821126622477854);
                c.yPodOffset.set(7.545534269077572);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
            }
    );
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                //all these are subject to change once you tune
                Controller primaryTranslationalForward = Controller.proportional(0.4689971488095553);
                Controller secondaryTranslationalForward = Controller.proportional(0.1732819146505124);
                Controller primaryTranslationalLateral = Controller.proportional(0.7671499388434017);
                Controller secondaryTranslationalLateral = Controller.proportional(0.28344140377874244);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01363691918303835));
                c.brake.set(Controller.proportionalFeedforward(0.011591381305582597));

                c.headingFeedback.set(Controller.proportional(8.17773323289037));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06471633476048785, 0.010735711135389292));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10336463413611006, 0.08445617157675939));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0019807614910681017, 0.0019189743311709183));

                c.maxAchievableForwardVelocity.set(66.16157438873356);
                c.maxAchievableStrafeVelocity.set(48.361819588070674);
                c.naturalForwardDeceleration.set(31.76525717518457);
                c.naturalStrafeDeceleration.set(72.6983607218218);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}