package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.teamcode.KestrelLauncherNoTurret.shootBalls;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class TestAutoNoTurret extends OpMode {
    private Follower follower;
    private KestrelLauncherNoTurret launcher;
    private final PoseFactory p = PoseFactory.degrees();
    boolean red = true, isShooting = false;

    private final Pose
            startPose = p.of(24, 24, 0),
            scorePose = p.of(48, 24, 90),
            parkPose = p.of(24, 24, 180);

    Path
            score = linePath(startPose, scorePose),
            park = linePath(scorePose,parkPose);

    private Command autoRoutine() {
        return sequential(
                follow(follower, score),

                shootBalls(4),

                follow(follower, park)
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        launcher = new KestrelLauncherNoTurret();
        launcher.init(hardwareMap);

    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        launcher.update();
        launcher.updateShooter();
        launcher.setLauncher(isShooting);

        Pose robotPose = follower.pose();
        KestrelLauncher.robotPose = robotPose;

        telemetry.addData("X", robotPose.x());
        telemetry.addData("Y", robotPose.y());
        telemetry.addData("Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Follower Mode", follower.mode());

        telemetry.update();
    }



    public Path linePath(Pose start, Pose end) {
        return Paths.line(start, end).linear(start, end);
    }
    public Path curvePath(Pose start, Pose control, Pose end) {
        return Paths.curve(start, control, end).linear(start, end);
    }

}
