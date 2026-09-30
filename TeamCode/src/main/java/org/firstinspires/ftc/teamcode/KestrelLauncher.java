package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TestTeleop.determineRotationDirection;
import static org.firstinspires.ftc.teamcode.TestTeleop.normalizeAngle;
import static org.firstinspires.ftc.teamcode.TestTeleop.turretTracking;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.HeadingPIDF;

public class KestrelLauncher {
    DcMotorEx motorLaunch, motorIntake, motorTurret;
    public static double
            launcher_p = 0, launcher_f = 0, turret_p = 0, turret_i = 0, turret_d = 0, turret_f = 0; // TODO tune ts
    Pose stephenPose = TestTeleop.stephenPose;
    HeadingPIDF headingPIDF;

    public void init(HardwareMap hwMap) {
        motorLaunch = (DcMotorEx) hwMap.dcMotor.get("launch");
        motorIntake = (DcMotorEx) hwMap.dcMotor.get("intake");
        motorTurret = (DcMotorEx) hwMap.dcMotor.get("turret");

        motorLaunch.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTurret.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLaunch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLaunch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLaunch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launcher_p, 0, 0, launcher_f));
        headingPIDF = new HeadingPIDF(turret_p,turret_i,turret_d,turret_f);
    }

    double targetVelocity = 0;
    public static double ticksPerDegree = 10; // TODO tune ts

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

        double turretAngle = motorTurret.getCurrentPosition()/ticksPerDegree;
        double targetAngle = normalizeAngle(targetHeading - stephenPose.heading());
        double angleError = determineRotationDirection(turretAngle, targetAngle);
        if (turretAngle > 400) {
            targetAngle = targetAngle - 360;
        }
        if (turretAngle < -40) {
            targetAngle = targetAngle + 360;
        }

        if (turretTracking) {
            motorTurret.setPower(headingPIDF.calculate(angleError));
        }
        double velocityError = targetVelocity - motorLaunch.getVelocity();
        boolean inPosition = Math.abs(stephenPose.y()-71) > 24;
        if (Math.abs(velocityError) < 100 && inPosition && Math.abs(angleError) < 5) {
            motorIntake.setPower(1);
        }
    }


    public boolean isBusy() {
        //if (currentlyLaunching) {true} else {false} //TODO
        return false;
    }
}
