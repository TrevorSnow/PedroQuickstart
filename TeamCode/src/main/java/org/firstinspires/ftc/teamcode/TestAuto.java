package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class TestAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(24, 24, 0);
    private final Pose scorePose = p.of(48, 48, 90);
    private final Pose parkPose = p.of(72, 48, 90);

    Path score = linePath(startPose, scorePose);
    Path park = linePath(scorePose,parkPose);

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

    }

    @Override
    public void start() {
        schedule(autoRoutine());

    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        Pose robotPose = follower.pose();

        telemetry.addData("X", robotPose.x());
        telemetry.addData("Y", robotPose.y());
        telemetry.addData("Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Follower Mode", follower.mode());

        telemetry.update();
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, score),
                // Add mechanism commands here.
                follow(follower, park)
        );
    }



    public Path linePath(Pose start, Pose end) {
        return Paths.line(start, end);
    }
    public Path curvePath(Pose start, Pose control, Pose end) {
        return Paths.curve(start, control, end);
    }


}
