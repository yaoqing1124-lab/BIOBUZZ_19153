package org.firstinspires.ftc.teamcode._2027.drive.swerve;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode._2027.config.Config;
import org.firstinspires.ftc.teamcode.util.Pose;

public class SwerveConfig {
    public static final double length = 210;
    public static final double width = 296;
    public static SwerveConfig rf = new SwerveConfig()
            .Quadrant(1)
            .MotorName(  Config.Drive.Name[1])
            .ServoName(  Config.Drive.Name[1] + "s")
            .EncoderName(Config.Drive.Name[1] + "e")
            .MinVoltage(Config.Drive.minVoltage.rf)
            .MaxVoltage(Config.Drive.maxVoltage.rf)
            .ServoOffset(Config.Drive.offset.rf)
            .pidfCoefficients(0.3, 0.001, 0.015, 0);
    public static SwerveConfig lf = new SwerveConfig()
            .Quadrant(2)
            .MotorName(  Config.Drive.Name[2])
            .ServoName(  Config.Drive.Name[2] + "s")
            .EncoderName(Config.Drive.Name[2] + "e")
            .MinVoltage(Config.Drive.minVoltage.lf)
            .MaxVoltage(Config.Drive.maxVoltage.lf)
            .ServoOffset(Config.Drive.offset.lf)
            .pidfCoefficients(0.3, 0.001, 0.015, 0);
    public static SwerveConfig lb = new SwerveConfig()
            .Quadrant(3)
            .MotorName(  Config.Drive.Name[3])
            .ServoName(  Config.Drive.Name[3] + "s")
            .EncoderName(Config.Drive.Name[3] + "e")
            .MinVoltage(Config.Drive.minVoltage.lb)
            .MaxVoltage(Config.Drive.maxVoltage.lb)
            .ServoOffset(Config.Drive.offset.lb)
            .pidfCoefficients(0.3, 0.001, 0.015, 0);
    public static SwerveConfig rb = new SwerveConfig()
            .Quadrant(4)
            .MotorName(  Config.Drive.Name[4])
            .ServoName(  Config.Drive.Name[4] + "s")
            .EncoderName(Config.Drive.Name[4] + "e")
            .MinVoltage(Config.Drive.minVoltage.rb)
            .MaxVoltage(Config.Drive.maxVoltage.rb)
            .ServoOffset(Config.Drive.offset.rb)
            .pidfCoefficients(0.3, 0.001, 0.015, 0);

    public int Quadrant;
    public String MotorName;
    public String ServoName;
    public String EncoderName;
    public double ServoOffset;
    public double MinVoltage;
    public double MaxVoltage;
    public Pose ModulePose;
    public PIDFCoefficients pidfCoefficients;

    public SwerveConfig Quadrant(int Quadrant){
        this.Quadrant = Quadrant;
        switch (Quadrant){
            case 1:
                ModulePose = new Pose( width/2,  length/2);
                break;
            case 2:
                ModulePose = new Pose( width/2, -length/2);
                break;
            case 3:
                ModulePose = new Pose(-width/2, -length/2);
                break;
            case 4:
                ModulePose = new Pose(-width/2,  length/2);
                break;
        }
        return this;
    }

    public SwerveConfig MotorName(String MotorName){
        this.MotorName = MotorName;
        return this;
    }

    public SwerveConfig ServoName(String ServoName){
        this.ServoName = ServoName;
        return this;
    }

    public SwerveConfig EncoderName(String EncoderName){
        this.EncoderName = EncoderName;
        return this;
    }

    public SwerveConfig ServoOffset(double voltageOffset){
        this.ServoOffset = (voltageOffset / (MaxVoltage - MinVoltage)) * (2 * Math.PI);
        return this;
    }

    public SwerveConfig MinVoltage(double MinVoltage){
        this.MinVoltage = MinVoltage;
        return this;
    }

    public SwerveConfig MaxVoltage(double MaxVoltage){
        this.MaxVoltage = MaxVoltage;
        return this;
    }

    public SwerveConfig pidfCoefficients(double kp, double ki, double kd, double kf){
        pidfCoefficients = new PIDFCoefficients(kp, ki, kd, kf);
        return this;
    }

    public SwerveConfig pidfCoefficients(double kp, double ki, double kd){
        pidfCoefficients = new PIDFCoefficients(kp, ki, kd, 0);
        return this;
    }
}