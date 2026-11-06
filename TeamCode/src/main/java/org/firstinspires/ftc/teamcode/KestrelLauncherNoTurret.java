package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.FalconsTeleOp.calculateDistance;
import static org.firstinspires.ftc.teamcode.FalconsTeleOp.determineRotationDirection;
import static org.firstinspires.ftc.teamcode.FalconsTeleOp.normalizeAngle;
import static org.firstinspires.ftc.teamcode.FalconsTeleOp.stephenPose;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KestrelLauncherNoTurret {
    static DcMotorEx motorLaunch;
    static DcMotorEx motorIntake;
    static Servo servoGate;
    static Servo servoLoader;
    public static double launcher_p = 0, launcher_f = 0; // TODO tune ts
    HeadingPIDF headingPIDF;
    public static Pose robotPose;

    static int shotsRemaining = 0;

    public void init(HardwareMap hwMap) {
        shotsRemaining = 0;

        motorLaunch = hwMap.get(DcMotorEx.class, "launch");
        motorIntake = hwMap.get(DcMotorEx.class, "intake");
        servoGate = hwMap.get(Servo.class, "gate");
        servoLoader = hwMap.get(Servo.class, "loader");


        motorLaunch.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLaunch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLaunch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLaunch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launcher_p, 0, 0, launcher_f));
    }

    public static Pose blueTarget = new Pose(59,51),
            redTarget = new Pose(83,91),
            currentTarget = new Pose(83,91);

    double targetDistance = 0;
    double targetHeading = 0;
    static double targetVelocity = 0;
    public static double headingError = 0, velocityError = 0;
    boolean red = true;
    public void setTarget(boolean Red) {
        red = Red;
    }

    public void update() {
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

        motorLaunch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launcher_p, 0, 0, launcher_f));

        targetVelocity = 0;//distance * 2; // TODO tune ts equation

        headingError = determineRotationDirection(robotPose.heading(), targetHeading);
        velocityError = targetVelocity - motorLaunch.getVelocity();
    }

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        LAUNCH
    }
    private static LaunchState launchState = LaunchState.IDLE;

    public void updateShooter() {
        // For Auto
        switch(launchState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    stateTimer.reset();
                    openGate();
                    setIntake(0);
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (shotsRemaining > 0) {
                    if (Math.abs(velocityError) < 100 && headingError < 10) {
                        if (stateTimer.seconds() < 0.30) {
                            setIntake(1);
                        } else {
                            shotsRemaining -= 1;
                            stateTimer.reset();
                        }
                    }
                } else {
                    setIntake(0);
                    setLauncher(0);
                    closeGate();
                    stateTimer.reset();
                    launchState = LaunchState.IDLE;
                }
        }
    }
    public static Command shootBalls(double balls) {
        return Command.build()
                .setStart(() -> shotsRemaining = 4)
                .setDone(() -> launchState == LaunchState.IDLE);
    }

    public static Command spinUpLauncher = Command.build()
                .setStart(() -> setLauncher(true))
                .setDone(() -> true);

    public static Command setIntakePower(double power) {
        return Command.build()
                .setStart(() -> setIntake(power))
                .setDone(() -> true);}

    public static Command raiseLoader = Command.build()
                .setStart(() -> setLoader(false))
                .setDone(() -> true);

    public static Command lowerLoader = Command.build()
                .setStart(() -> setLoader(true))
                .setDone(() -> true);

// LAUNCHER
    public static void setLauncher(boolean isShooting) {motorLaunch.setVelocity(isShooting ? targetVelocity : 0);}
    public static void setLauncher(double velocity) {motorLaunch.setVelocity(velocity);}

// INTAKE
    public static void setIntake(double power) {motorIntake.setPower(power);}

// GATE SERVO
    static double gateOpen = 0, gateClosed = 0.5;
    public static void openGate() {servoGate.setPosition(gateOpen);}
    public static void closeGate() {servoGate.setPosition(gateClosed);}

    public static void setGate(boolean open) {servoGate.setPosition(open ? gateOpen : gateClosed);}
    public static void setGate(double pose) {servoGate.setPosition(pose);}

// LOADER
    static double loaderUp = 0.2, loaderDown = 0.5;
    public static void setLoader(boolean down) {servoLoader.setPosition(down ? loaderDown : loaderUp);}
    public static void setLoader(double pose) {servoLoader.setPosition(pose);}

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Target Pose", currentTarget);
        telemetry.addData("Target Distance", targetDistance);
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addLine();
        telemetry.addData("Launch target vel", targetVelocity);
        telemetry.addData("Launch actual vel", motorLaunch.getVelocity());
    }
}
