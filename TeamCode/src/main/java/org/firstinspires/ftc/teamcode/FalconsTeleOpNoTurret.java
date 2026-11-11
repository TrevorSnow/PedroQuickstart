package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp (name = "FalconsTeleOp NT")
public class FalconsTeleOpNoTurret extends OpMode {
    Follower follower;
    KestrelLauncherNoTurret launcher;
    HeadingPIDF headingPIDF;
    public static boolean isShooting, red;
    double launchVelocity = 0;
    public static double closeVelocity = 1500, farVelocity = 2000;
    public static Pose stephenPose = new Pose(0,0);
    public static PIDFCoefficients headingPIDFCoefficients = new PIDFCoefficients(0,0,0,0);

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        launcher = new KestrelLauncherNoTurret();
        headingPIDF = new HeadingPIDF(0,0,0,0);
        launcher.init(hardwareMap);
        KestrelLauncherNoTurret.setLoader(false);
    }

    @Override
    public void loop() {

        stephenPose = follower.pose();
        KestrelLauncherNoTurret.robotPose = stephenPose;
        headingPIDF.setCoefficients(headingPIDFCoefficients);

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
            KestrelLauncherNoTurret.motorIntake.setPower(1);
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            KestrelLauncherNoTurret.motorIntake.setPower(-1);
        } else {
            KestrelLauncherNoTurret.motorIntake.setPower(0);
        }

    // LAUNCHER
        KestrelLauncherNoTurret.setGate(gamepad1.right_trigger_pressed || gamepad2.right_trigger_pressed);

        launcher.setTarget(red);
        launcher.update();

        if (gamepad1.a || gamepad2.a) {
            launchVelocity = closeVelocity;
        } else if (gamepad1.b || gamepad2.b) {
            launchVelocity = farVelocity;
        } else {
            KestrelLauncherNoTurret.setLauncher(isShooting);
        }


    // LOADER
        KestrelLauncherNoTurret.setLoader(gamepad1.x || gamepad2.x);

    // TELEMETRY
        telemetry.addData("X", stephenPose.x());
        telemetry.addData("Y", stephenPose.y());
        telemetry.addData("Heading", Math.toDegrees(stephenPose.heading()));
        telemetry.addLine();
        launcher.addTelemetry(telemetry);

        dashboardTelemetry.addData("Pose", stephenPose);
        launcher.addDashboardTelemetry(dashboardTelemetry);

    // UPDATE
        follower.update();
        dashboardTelemetry.update();
        telemetry.update();
    }

    // Any additional methods go here

}