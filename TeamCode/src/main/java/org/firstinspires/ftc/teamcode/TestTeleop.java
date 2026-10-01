package org.firstinspires.ftc.teamcode;

//import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

//@Configurable
@TeleOp (name = "Test Teleop")
public class TestTeleop extends OpMode {
    Follower follower;
    KestrelLauncher launcher;
    public static boolean isShooting, red, turretTracking = true;
    double targetDistance = 0, targetHeading = 0;
    public static Pose
            stephenPose = new Pose(0,0),
            blueTarget = new Pose(0,0),
            redTarget = new Pose(0,0),
            currentTarget = new Pose(0,0);


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        launcher = new KestrelLauncher();
        launcher.init(hardwareMap);

    }

    @Override
    public void loop() {

        // DRIVING
        /*ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );*/
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
        stephenPose = follower.pose();


        // LAUNCHER STUFF
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            red = !red;
        }
        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
            isShooting = !isShooting;
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            turretTracking = !turretTracking;
        }

        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            launcher.motorIntake.setPower(1);
        } else if (!launcher.isBusy()) {
            launcher.motorIntake.setPower(0);
        }

        if (stephenPose.y() < 71) {
            blueTarget = new Pose (59, 51);
            redTarget = new Pose (83, 51);
        } else {
            blueTarget = new Pose (59, 91);
            redTarget = new Pose (83, 91);
        }
        if (red) {
            currentTarget = redTarget;
        } else {
            currentTarget = blueTarget;
        }

        targetDistance = calculateDistance(currentTarget.x(), currentTarget.y(), stephenPose.x(), stephenPose.y());
        targetHeading = normalizeAngle(Math.toDegrees(Math.atan2(currentTarget.y() - stephenPose.y(),  currentTarget.x() - stephenPose.x())));
        launcher.update(isShooting, targetDistance, targetHeading);
        if (!turretTracking) {
            launcher.motorTurret.setPower(gamepad2.right_stick_x);
        }


        // TELEMETRY
        telemetry.addData("X", stephenPose.x());
        telemetry.addData("Y", stephenPose.y());
        telemetry.addData("Heading", Math.toDegrees(stephenPose.heading()));
        telemetry.addLine();
        telemetry.addData("Target Pose", currentTarget);
        telemetry.addData("Target Distance", targetDistance);
        telemetry.addData("Target Heading", targetHeading);
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