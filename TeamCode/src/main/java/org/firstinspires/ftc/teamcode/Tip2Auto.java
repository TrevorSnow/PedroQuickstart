package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.KestrelLauncherNoTurret.loaderDown;
import static org.firstinspires.ftc.teamcode.KestrelLauncherNoTurret.lowerLoader;
import static org.firstinspires.ftc.teamcode.KestrelLauncherNoTurret.raiseLoader;
import static org.firstinspires.ftc.teamcode.KestrelLauncherNoTurret.setIntakePower;
import static org.firstinspires.ftc.teamcode.KestrelLauncherNoTurret.shootBalls;
import static org.firstinspires.ftc.teamcode.KestrelLauncherNoTurret.spinUpLauncher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Tip2Auto", group = "Autonomous")
public class Tip2Auto extends LinearOpMode {

    private Follower follower;
    private KestrelLauncherNoTurret launcher;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    boolean red = true, isShooting = false;

    // POSES
        private final Pose start = poseFactory.of(38.431, 8.3993, 270);
        private final Pose shoot1 = poseFactory.of(42.8657, 16.0952, 250);
        private final Pose intakeCornerStart = poseFactory.of(42.8657, 16.0952, 250);
        private final Pose intakeCorner = poseFactory.of(8.7116, 8.8947, 270);
        private final Pose intakeCornerControl1 = poseFactory.of(17.6087, 28.5325, 0);
        private final Pose shootCorner = poseFactory.of(50.7496, 121.608, 110);
        private final Pose shootCornerControl1 = poseFactory.of(26.479, 103.0932, 0);
        private final Pose shootCornerSegment1Start = poseFactory.of(50.7496, 121.608, 270);
        private final Pose shootCornerSegment1End = poseFactory.of(50.7496, 121.608, 70);
        private final Pose shootCornerSegment3Start = poseFactory.of(50.7496, 121.608, 70);
        private final Pose shootCornerSegment3End = poseFactory.of(50.7496, 121.608, 110);
        private final Pose intakeFlower1Start = poseFactory.of(50.7496, 121.608, 110);
        private final Pose intakeFlower1 = poseFactory.of(48.0709, 131.5765, 90);
        private final Pose shootFlower1Start = poseFactory.of(48.0709, 131.5765, 90);
        private final Pose shootFlower1 = poseFactory.of(43.8436, 120.3326, 115);
        private final Pose intakeFlower2Start = poseFactory.of(43.8436, 120.3326, 115);
        private final Pose intakeFlower2 = poseFactory.of(9.1633, 47.3168, 180);
        private final Pose intakeFlower2Control1 = poseFactory.of(27.4221, 100.8263, 0);
        private final Pose intakeFlower2Control2 = poseFactory.of(29.331, 50.8904, 0);
        private final Pose shootFlower2Start = poseFactory.of(9.1633, 47.3168, 180);
        private final Pose shootFlower2 = poseFactory.of(27.0556, 19.2448, 240);
        private final Pose park = poseFactory.of(11.4854, 95.1529, -78.4083);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1())
                        .with(spinUpLauncher),
                shootBalls(4),

                follow(follower, intakeCorner())
                        .with(setIntakePower(1)),
                follow(follower, shootCorner())
                        .with(spinUpLauncher),
                shootBalls(4),

                follow(follower, intakeFlower1())
                        .with(lowerLoader)
                        .with(setIntakePower(1)),
                follow(follower, shootFlower1())
                        .with(raiseLoader)
                        .with(spinUpLauncher),
                shootBalls(4),

                follow(follower, intakeFlower2())
                        .with(lowerLoader)
                        .with(setIntakePower(1)),
                follow(follower, shootFlower2())
                        .with(raiseLoader)
                        .with(spinUpLauncher),
                shootBalls(4),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path intakeCorner() {
        return Paths.curve(intakeCornerStart, intakeCornerControl1, intakeCorner).linear(intakeCornerStart, intakeCorner);
    }

    public Path shootCorner() {
        return Paths.curve(intakeCorner, shootCornerControl1, shootCorner).heading(Interpolator.piecewise().until(0.3555, Interpolator.linear(shootCornerSegment1Start, shootCornerSegment1End)).until(0.8, Interpolator.tangent).until(1, Interpolator.linear(shootCornerSegment3Start, shootCornerSegment3End)));
    }

    public Path intakeFlower1() {
        return Paths.line(intakeFlower1Start, intakeFlower1).linear(intakeFlower1Start, intakeFlower1);
    }

    public Path shootFlower1() {
        return Paths.line(shootFlower1Start, shootFlower1).linear(shootFlower1Start, shootFlower1);
    }

    public Path intakeFlower2() {
        return Paths.curve(intakeFlower2Start, intakeFlower2Control1, intakeFlower2Control2, intakeFlower2).linear(intakeFlower2Start, intakeFlower2);
    }

    public Path shootFlower2() {
        return Paths.line(shootFlower2Start, shootFlower2).linear(shootFlower2Start, shootFlower2);
    }

    public Path park() {
        return Paths.line(shootFlower2, park).reverseTangent();
    }
}
