package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    private double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastError = 0;
    private double maxIntegral = 1.0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer.reset();
    }

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = maxIntegral;
    }

    /**
     * 計算 PIDF 輸出功率
     * @param error 當前角度誤差 (目標值 - 當前值)
     */
    public double calculate(double error) {
        double dt = timer.seconds();
        timer.reset();

        if (dt == 0.0) {
            dt = 0.01; // 避免除以零
        }

        // 1. Proportional
        double proportional = error * kP;

        // 2. Integral
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double integral = integralSum * kI;

        // 3. Derivative
        double derivative = (error - lastError) / dt;
        lastError = error;
        double dTerm = derivative * kD;

        // 4. Feedforward
        double feedforward = Math.signum(error) * kF;

        double output = proportional + integral + dTerm + feedforward;

        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}