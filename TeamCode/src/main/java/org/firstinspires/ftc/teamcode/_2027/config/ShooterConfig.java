package org.firstinspires.ftc.teamcode._2027.config;

import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Flywheel;

public class ShooterConfig {

    public static final double FLYWHEEL_RADIUS = 0;
    public static final double IDEAL_INCIDENCE_ANGLE = 60; // 低於水平面的夾角
    public String turretName;
    public String flywheelName;
    public String pitchName;

    public double offset;
    public double center; //voltage
    public Flywheel flywheelInstance;
    public ShooterConfig turretName(String turretName){
        this.turretName = turretName;
        return this;
    }
    public ShooterConfig flywheelName(String flywheelName){
        this.flywheelName = flywheelName;
        return this;
    }
    public ShooterConfig pitchName(String pitchName){
        this.pitchName = pitchName;
        return this;
    }
    public ShooterConfig offset(double offset){
        this.offset = offset;
        return this;
    }
    public ShooterConfig center(double center){
        this.center = center;
        return this;
    }
    public ShooterConfig flywheelInstance(Flywheel flywheelInstance){
        this.flywheelInstance = flywheelInstance;
        return this;
    }
    public static class physical{
        public static final double G = 386.09; // inch
        public static final double H = 0;
        public static double calcuteAngle(double R, double H){ //步驟一:算出理想仰角 #不會用到
            return Math.toDegrees(Math.atan(Math.tan(Math.toRadians(IDEAL_INCIDENCE_ANGLE)) + 2 * H / R));
        }
        public static double calculateVelocity(double R, double H){ //步驟二:算出理想初速度
            double theta = Math.toRadians(calcuteAngle(R, H));
            return Math.sqrt(physical.G * R / (Math.pow(Math.cos(theta), 2) * (Math.tan(theta) + Math.tan(Math.toRadians(IDEAL_INCIDENCE_ANGLE)))));
        }
        public static Double VelocityCalculateAngle(double v, double R, double H){ //步驟三:當下速度反解仰角
            //Ax^2 + Bx + C = 0, let x = tanθ
            double A = (physical.G * Math.pow(R, 2)) / (2 * Math.pow(v, 2));
            double B = -R;
            double C = H + A;
            double delta = Math.pow(B, 2) - 4 * A * C;
            if(delta < 0){
                return null;
            }else{
                double x = (-B + Math.sqrt(delta)) / (2 * A);
                return Math.toDegrees(Math.atan(x));
            }
        }
    }
}
