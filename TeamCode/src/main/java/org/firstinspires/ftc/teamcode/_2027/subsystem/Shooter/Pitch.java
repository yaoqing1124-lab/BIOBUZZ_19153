package org.firstinspires.ftc.teamcode._2027.subsystem.Shooter;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode._2027.config.Config;
import org.firstinspires.ftc.teamcode._2027.config.ShooterConfig;
import org.firstinspires.ftc.teamcode._2027.drive.odometry.Pinpoint;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.util.Pose;

public class Pitch extends OpBehavior {
    private Pitch(ShooterConfig config){
        this.config = config;
    }
    private final static Pitch s_instance;
    private final static Pitch l_instance;
    static {
        s_instance = new Pitch(Config.Shooter.S);
        l_instance = new Pitch(Config.Shooter.L);
//        OpObject.RegisterGlobalBehavior(s_instance);
//        OpObject.RegisterGlobalBehavior(l_instance);
    }
    public static Pose target;
    public static Pitch get_s_instance(){
        return s_instance;
    }
    public static Pitch get_l_instance(){
        return l_instance;
    }
    private ShooterConfig config;
    private Servo m_pitch;
    @Override
    public void Init() {
        m_pitch = m_object.hardwareMap.get(Servo.class, config.pitchName);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        switch (mode){
            case correction:
                Double degree = ShooterConfig.physical.VelocityCalculateAngle(config.flywheelInstance.getActualVelocity(), R(), H());
                if(degree != null){
                    m_pitch.setPosition(degree2position(degree));
                }else{
                    m_pitch.setPosition(degree2position(ShooterConfig.physical.calcuteAngle(R(), H())));
                }
                break;
            case test:
                m_pitch.setPosition(degree2position(ShooterConfig.physical.calcuteAngle(R(), H())));
                break;
        }
    }

    @Override
    public void Stop() {

    }
    public Pose turretPose(){
        Pose2D currentRobot = Pinpoint.getPose2D();
        double X = currentRobot.getX(DistanceUnit.INCH) + config.offset * Math.sin(currentRobot.getHeading(AngleUnit.RADIANS));
        double Y = currentRobot.getY(DistanceUnit.INCH) + config.offset * Math.cos(currentRobot.getHeading(AngleUnit.RADIANS));
        return new Pose(X, Y);
    }
    public double R(){
        double dx = turretPose().x - Pinpoint.getPose2D().getX(DistanceUnit.INCH);
        double dy = turretPose().y - Pinpoint.getPose2D().getY(DistanceUnit.INCH);
        return Math.hypot(dx, dy);
    }
    public double H() {
        return ShooterConfig.physical.H;
    }
    public double degree2position(double degree){
        //TODO
        return 0;
    }
    public enum Mode{
        test, correction
    }
    public Mode mode;
}
