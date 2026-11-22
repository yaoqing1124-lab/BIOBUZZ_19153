
package org.firstinspires.ftc.teamcode._2027.subsystem.Shooter;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode._2027.config.Config;
import org.firstinspires.ftc.teamcode._2027.config.ShooterConfig;
import org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry.Pinpoint;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.util.Pose;

public class Turret extends OpBehavior {

    private final ShooterConfig config;
    private Servo turret;
    private static final double MAX_DEG = 350.0 * 11.0 / 10.0;
    private double lastAimDegree = 0.0;

    private Turret(ShooterConfig config) {
        this.config = config;
    }

    private static final Turret s_instance;
    private static final Turret l_instance;

    static {
        s_instance = new Turret(Config.Shooter.S);
        l_instance = new Turret(Config.Shooter.L);

        OpObject.RegisterGlobalBehavior(s_instance);

        if (OpObject.getMode() == OpObject.Mode.Competition) {
            OpObject.RegisterGlobalBehavior(s_instance);
            OpObject.RegisterGlobalBehavior(l_instance);
        }
    }

    public static Pose target = new Pose(0, 0);

    public static Turret get_s_instance() {
        return s_instance;
    }

    public static Turret get_l_instance() {
        return l_instance;
    }

    @Override
    public void Init() {
        turret = m_object.hardwareMap.get(
                Servo.class,
                config.turretName
        );

        lastAimDegree = 0.0;
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        target = AimTarget.get_instance().getTargetPose();

        double aimDegree = calculateFieldAimDegree();

        double selectedDegree = selectAimDegree(aimDegree);

        if (!Double.isNaN(selectedDegree)) {
            double servoPosition = degree2position(selectedDegree);

            turret.setPosition(servoPosition);
            lastAimDegree = selectedDegree;
        }
    }

    @Override
    public void Stop() {
    }

    public Pose turretPose() {
        Pose2D robot = Pinpoint.getPose2D();

        double heading = robot.getHeading(AngleUnit.RADIANS);

        double robotX = robot.getX(DistanceUnit.INCH);
        double robotY = robot.getY(DistanceUnit.INCH);

        double X = robotX
                - config.offset * Math.sin(heading);

        double Y = robotY
                + config.offset * Math.cos(heading);

        return new Pose(X, Y);
    }

    public double calculateFieldAimDegree() {
        Pose turretPos = turretPose();

        double dx = target.x - turretPos.x;
        double dy = target.y - turretPos.y;

        double fieldAimDegree = Math.toDegrees(
                Math.atan2(-dx, dy)
        );

        double robotHeading = Pinpoint.getPose2D()
                .getHeading(AngleUnit.DEGREES);

        return fieldAimDegree - robotHeading;
    }

    private double selectAimDegree(double degree) {
        double minDegree = (config.center - 1.0) * MAX_DEG;
        double maxDegree = config.center * MAX_DEG;

        double bestDegree = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (int k = -3; k <= 3; k++) {

            double candidate = degree + 360.0 * k;

            if (candidate >= minDegree &&
                    candidate <= maxDegree) {

                double distance =
                        Math.abs(candidate - lastAimDegree);

                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestDegree = candidate;
                }
            }
        }

        return bestDegree;
    }

    public double degree2position(double degree) {
        return config.center - degree / MAX_DEG;
    }
}
