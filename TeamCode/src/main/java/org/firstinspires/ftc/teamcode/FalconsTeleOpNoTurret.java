package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

//@Configurable
@TeleOp (name = "FalconsTeleOp NT")
public class FalconsTeleOpNoTurret extends OpMode {
    Follower follower;
    KestrelLauncherNoTurret launcher;
    HeadingPIDF headingPIDF;
    public static boolean isShooting, red;
    double launchVelocity = 0;
    public static double closeVelocity = 1500, farVelocity = 2000;
    public static Pose stephenPose = new Pose(0,0);



    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        launcher = new KestrelLauncherNoTurret();
        headingPIDF = new HeadingPIDF(0,0,0,0);
        launcher.init(hardwareMap);
        launcher.setLoader(0);
    }

    @Override
    public void loop() {

        stephenPose = follower.pose();
        KestrelLauncherNoTurret.robotPose = stephenPose;

        // DRIVING
        /*ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                gamepad1.left_trigger_pressed ? headingPIDF.calculate(KestrelLauncherNoTurret.headingError) : -gamepad1.right_stick_x
        );*/
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                gamepad1.left_trigger_pressed ? headingPIDF.calculate(KestrelLauncherNoTurret.headingError) : -gamepad1.right_stick_x
        );

        stephenPose = follower.pose();
        KestrelLauncherNoTurret.robotPose = stephenPose;

        // LAUNCHER STUFF
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            red = !red;
        }
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            isShooting = !isShooting;
        }

        // INTAKE
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            launcher.motorIntake.setPower(1);
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            launcher.motorIntake.setPower(-1);
        } else {
            launcher.motorIntake.setPower(0);
        }

        // LAUNCHER
        launcher.setGate(gamepad1.right_trigger_pressed || gamepad2.right_trigger_pressed);

        launcher.setTarget(red);
        launcher.update();

        if (gamepad1.a || gamepad2.a) {
            launchVelocity = closeVelocity;
        } else if (gamepad1.b || gamepad2.b) {
            launchVelocity = farVelocity;
        } else {
            launchVelocity = 0;
        }
        launcher.setLauncher(isShooting);


        // LOADER
        launcher.setLoader(gamepad1.x || gamepad2.x);

    // TELEMETRY
        telemetry.addData("X", stephenPose.x());
        telemetry.addData("Y", stephenPose.y());
        telemetry.addData("Heading", Math.toDegrees(stephenPose.heading()));
        telemetry.addLine();
        launcher.addTelemetry(telemetry);


    // UPDATE
        follower.update();
        telemetry.update();
    }

    // Any additional methods go here
    public static double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public static double normalizeAngle(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    public static double determineRotationDirection(double current, double target) {
        double clockwiseDegrees;
        double counterclockwiseDegrees;

        // Determine the larger of the two headings
        if (target > current) {
            counterclockwiseDegrees = target - current;
            clockwiseDegrees = 360 - counterclockwiseDegrees;
        } else {
            clockwiseDegrees = current - target;
            counterclockwiseDegrees = 360 - clockwiseDegrees;
        }
        if (clockwiseDegrees < counterclockwiseDegrees) {
            return -clockwiseDegrees;
        } else {
            return counterclockwiseDegrees;
        }
    }

}