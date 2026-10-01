package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TestTeleop.determineRotationDirection;
import static org.firstinspires.ftc.teamcode.TestTeleop.normalizeAngle;
import static org.firstinspires.ftc.teamcode.TestTeleop.stephenPose;
import static org.firstinspires.ftc.teamcode.TestTeleop.turretTracking;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HeadingPIDF;

public class KestrelLauncher {
    DcMotorEx motorLaunch, motorIntake, motorTurret;
    public static double
            launcher_p = 0, launcher_f = 0, turret_p = 0.005, turret_i = 0, turret_d = 0, turret_f = 0.01; // TODO tune ts
    HeadingPIDF headingPIDF;

    public void init(HardwareMap hwMap) {
        //motorLaunch = (DcMotorEx) hwMap.dcMotor.get("launch");
        motorLaunch = hwMap.get(DcMotorEx.class, "launch");
        motorIntake = hwMap.get(DcMotorEx.class, "intake");
        motorTurret = hwMap.get(DcMotorEx.class, "turret");

        motorLaunch.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTurret.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLaunch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLaunch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLaunch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launcher_p, 0, 0, launcher_f));
        headingPIDF = new HeadingPIDF(turret_p,turret_i,turret_d,turret_f);
    }

    double targetVelocity = 0;
    public static double ticksPerDegree = 381/360; // TODO tune ts
    double turretAngle = 0, targetAngle = 0, angleError = 0;

    public void update(boolean isShooting, double distance, double targetHeading) {
        motorLaunch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launcher_p, 0, 0, launcher_f));
        headingPIDF.setCoefficients(turret_p,turret_i,turret_d,turret_f);

        targetVelocity = 0;//distance * 2; // TODO tune ts equation
        if (isShooting) {
            motorLaunch.setVelocity(targetVelocity);
        } else {
            motorLaunch.setVelocity(0);
        }

        turretAngle = (double) (motorTurret.getCurrentPosition()*360/381);//ticksPerDegree);
        targetAngle = targetHeading - Math.toDegrees(stephenPose.heading());
        if (targetAngle > 400) {
            targetAngle = targetAngle - 360;
        }
        if (targetAngle < -40) {
            targetAngle = targetAngle + 360;
        }
        //angleError = determineRotationDirection(turretAngle, targetAngle);
        angleError = targetAngle - turretAngle;


        if (turretTracking) {
            motorTurret.setPower(headingPIDF.calculate(angleError));
        }
        double velocityError = targetVelocity - motorLaunch.getVelocity();
        boolean inPosition = Math.abs(stephenPose.y()-71) > 24;
        //if (Math.abs(velocityError) < 100 && inPosition && Math.abs(angleError) < 5) {
        //    motorIntake.setPower(1);
        //}
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret angle", turretAngle);
        telemetry.addData("Target angle", targetAngle);
        telemetry.addData("Angle error", angleError);
        telemetry.addData("Turret ticks", motorTurret.getCurrentPosition());
        telemetry.addData("Turret power", motorTurret.getPower());
        telemetry.addData("Launch target vel", targetVelocity);
        telemetry.addData("Launch actual vel", motorLaunch.getVelocity());
    }

    public boolean isBusy() {
        //if (currentlyLaunching) {true} else {false} //TODO
        return false;
    }
}
