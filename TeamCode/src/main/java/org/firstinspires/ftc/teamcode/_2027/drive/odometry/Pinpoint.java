package org.firstinspires.ftc.teamcode._2027.drive.odometry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode._2027.config.Config;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.util.Pose;

public class Pinpoint extends OpBehavior {
    private Pinpoint(){}
    private final static Pinpoint s_instance;
    static {
        s_instance = new Pinpoint();
        OpObject.RegisterGlobalBehavior(s_instance);
    }
    public static Pinpoint get_instance(){
        return s_instance;
    }
    public static GoBildaPinpointDriver pinpoint;
    @Override
    public void Init() {
        pinpoint = m_object.hardwareMap.get(GoBildaPinpointDriver.class, Config.Drive.pinpoint.name);

        pinpoint.setOffsets(Config.Drive.pinpoint.Xoffset, Config.Drive.pinpoint.Yoffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        pinpoint.update();
    }

    @Override
    public void Stop() {

    }
    public static Pose2D getPose2D(){
        return pinpoint.getPosition();
    }
    public static Pose getPose(){
        double x = pinpoint.getPosition().getX(DistanceUnit.INCH);
        double y = pinpoint.getPosition().getY(DistanceUnit.INCH);
        return new Pose(x, y);
    }
    public static double getHeading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }
    public static double getXVelocityComponent(){
        return pinpoint.getVelX(DistanceUnit.INCH);
    }
    public static double getYVelocityComponent(){
        return pinpoint.getVelY(DistanceUnit.INCH);
    }
}
