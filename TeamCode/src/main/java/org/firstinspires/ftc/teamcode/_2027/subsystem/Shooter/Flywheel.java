package org.firstinspires.ftc.teamcode._2027.subsystem.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode._2027.config.Config;
import org.firstinspires.ftc.teamcode._2027.config.ShooterConfig;
import org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry.Pinpoint;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.util.Pose;
public class Flywheel extends OpBehavior {
    private Flywheel(ShooterConfig config){
        this.config = config;
    }
    private final static Flywheel s_instance;
    private final static Flywheel l_instance;
    static {
        s_instance = new Flywheel(Config.Shooter.S);
        l_instance = new Flywheel(Config.Shooter.L);
        if(OpObject.getMode() == OpObject.Mode.Competition){
            OpObject.RegisterGlobalBehavior(s_instance);
            OpObject.RegisterGlobalBehavior(l_instance);
        }
        OpObject.RegisterGlobalBehavior(s_instance);

    }
    public static Pose target;
    public static Flywheel get_s_instance(){
        return s_instance;
    }
    public static Flywheel get_l_instance(){
        return l_instance;
    }
    private ShooterConfig config;
    private DcMotorEx m_motor;
    @Override
    public void Init() {
        m_motor = m_object.hardwareMap.get(DcMotorEx.class, config.flywheelName);
//        m_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, config.flywheelPID);
        m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m_motor.setVelocity(0);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        target = Turret.target;
        m_motor.setVelocity(3300 / 60.0 * 28.0);
//        m_motor.setVelocity(VirtualtoActualVeloci。ty(ShooterConfig.physical.calculateVelocity(R(), H())));
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
    public double H(){
        return ShooterConfig.physical.H;
    }
    public double VirtualtoActualVelocity(double virtualVelocity){
        //TODO
        return 0;
    }
    public double getActualVelocity(){
        //TODO
        return 0;
    }
    public double getVelocity(){
        return m_motor.getVelocity() / 28 * 60;
    }
}
