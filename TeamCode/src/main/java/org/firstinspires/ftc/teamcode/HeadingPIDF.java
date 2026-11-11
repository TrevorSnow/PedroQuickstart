package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class HeadingPIDF {
    // PIDF coefficients
    public double kP, kI, kD, kF;

    // State variables
    private double sumError = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public HeadingPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setCoefficients(PIDFCoefficients pidfCoefficients) {
        kP = pidfCoefficients.p;
        kI = pidfCoefficients.i;
        kD = pidfCoefficients.d;
        kF = pidfCoefficients.f;
    }

    public void reset() {
        sumError = 0;
        lastError = 0;
        lastTime = 0;
    }

    public double calculate(double error) {
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = (lastTime == 0) ? 0 : (currentTime - lastTime);

        // Proportional
        double P = kP * error;

        // Integral
        if (deltaTime > 0) {
            sumError += error * deltaTime;
        }
        double I = kI * sumError;

        // Derivative
        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        double D = kD * derivative;

        // Feedforward
        double F = kF * Math.signum(error);

        // Store for next loop
        lastError = error;
        lastTime = currentTime;

        // Total output
        return P + I + D + F;
    }
}
