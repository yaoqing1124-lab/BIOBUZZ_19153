package org.firstinspires.ftc.teamcode._2027.subsystem.Shooter;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode._2027.config.Config;
import org.firstinspires.ftc.teamcode._2027.config.ShooterConfig;
import org.firstinspires.ftc.teamcode._2027.drive.odometry.Pinpoint;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.util.Pose;

public class Turret extends OpBehavior {
    private Turret(ShooterConfig config){
        this.config = config;
    }
    private final static Turret s_instance;
    private final static Turret l_instance;
    static {
        s_instance = new Turret(Config.Shooter.S);
        l_instance = new Turret(Config.Shooter.L);
//        OpObject.RegisterGlobalBehavior(s_instance);
//        OpObject.RegisterGlobalBehavior(l_instance);
    }
    public static Pose target = new Pose(0,0);
    public static Turret get_s_instance(){
        return s_instance;
    }
    public static Turret get_l_instance(){
        return l_instance;
    }
    private ShooterConfig config;
    private Servo turret;
    @Override
    public void Init() {
        turret = m_object.hardwareMap.get(Servo.class, config.turretName);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        target = AimTarget.get_instance().getTargetPose();
        turret.setPosition(degree2position(calculateFieldAimDegree()));
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
    public double calculateFieldAimDegree(){
        double dx = target.x - turretPose().x; //12-0
        double dy = target.y - turretPose().y; //-19-0
        double AimRadians = Math.atan2(dy, dx);
        double FieldAimDegree = Math.toDegrees(AimRadians) - Pinpoint.getHeading();
        while (FieldAimDegree > 180) FieldAimDegree -= 360;
        while (FieldAimDegree < -180) FieldAimDegree += 360;
        return FieldAimDegree;
    }
    public double R(){
        double dx = turretPose().x - Pinpoint.getPose2D().getX(DistanceUnit.INCH);
        double dy = turretPose().y - Pinpoint.getPose2D().getY(DistanceUnit.INCH);
        return Math.hypot(dx, dy);
    }
    public double H(){
        return ShooterConfig.physical.H;
    }
    public double degree2position(double degree){
        final double MAX_DEG = 350 * 1.1;
        double position = config.center - degree / (MAX_DEG / 2);
        return position;
    }
}
