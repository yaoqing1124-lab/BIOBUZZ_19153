package org.firstinspires.ftc.teamcode._2027.drive.odometry;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode._2027.drive.swerve.Swerve;
import org.firstinspires.ftc.teamcode._2027.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.Pose;
import java.util.ArrayList;
import java.util.List;

public class Follower extends OpBehavior {
    private static Follower s_instance = new Follower();
    static {
        s_instance = new Follower();
    }
    public static Follower get_instance() {
        return s_instance;
    }

    private Swerve swerveDrive;
    private Pinpoint pinpointOdom;
    private List<Pose> fullPathPoints = new ArrayList<>();
    private int currentTargetIndex = 0;
    private double lookaheadDistance = 6.0;
    private double maxSpeed = 0.8;
    private double maxAngularSpeed = 0.6;
    private boolean isFollowing = false;
    private PIDFController headingController;
    private boolean brakeOnFinish = true;
    private SwerveModule.ZeroPowerMode finishPowerMode = SwerveModule.ZeroPowerMode.X_LOCK;

    @Override
    public void Init() {
        swerveDrive = Swerve.get_instance();
        pinpointOdom = Pinpoint.get_instance();
        headingController = new PIDFController(0.05, 0.0, 0.002, 0.0);
    }

    @Override
    public void Start() {
        isFollowing = false;
        currentTargetIndex = 0;
        headingController.reset();
    }

    @Override
    public void Loop() {
        if (!isFollowing || fullPathPoints == null || fullPathPoints.isEmpty()) {
            return;
        }

        Pose2D currentPose2d = Pinpoint.getPose2D();
        double currentX = currentPose2d.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double currentY = currentPose2d.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double currentHeading = Pinpoint.getHeading();

        updateTargetIndex(currentX, currentY);
        if (currentTargetIndex >= fullPathPoints.size()) {
            stopFollowing();
            return;
        }

        Pose target = fullPathPoints.get(currentTargetIndex);
        double dx = target.x - currentX;
        double dy = target.y - currentY;
        double headingRad = Math.toRadians(currentHeading);

        double rotX = dx * Math.cos(-headingRad) - dy * Math.sin(-headingRad);
        double rotY = dx * Math.sin(-headingRad) + dy * Math.cos(-headingRad);
        double magnitude = Math.hypot(rotX, rotY);
        if (magnitude > 1.0) {
            rotX /= magnitude;
            rotY /= magnitude;
        }

        double headingError = target.heading - currentHeading;
        headingError = Math.atan2(Math.sin(Math.toRadians(headingError)), Math.cos(Math.toRadians(headingError)));
        headingError = Math.toDegrees(headingError);

        double yawOutput = headingController.calculate(headingError);
        yawOutput = Math.max(-maxAngularSpeed, Math.min(maxAngularSpeed, yawOutput));

        swerveDrive.manual(rotY * maxSpeed, rotX * maxSpeed, yawOutput);
    }

    @Override
    public void Stop() {
        stopFollowing();
    }

    public void setCustomSegmentsPath(List<PathSegment> segments, boolean brake) {
        if (segments == null || segments.isEmpty()) {
            return;
        }
        List<Pose> generatedFullPath = new ArrayList<>();
        for (PathSegment seg : segments) {
            List<Pose> segmentPoints = seg.generatePath(50);
            if (!generatedFullPath.isEmpty() && !segmentPoints.isEmpty()) {
                segmentPoints = segmentPoints.subList(1, segmentPoints.size());
            }
            generatedFullPath.addAll(segmentPoints);
        }
        this.fullPathPoints = generatedFullPath;
        this.currentTargetIndex = 0;
        this.brakeOnFinish = brake;
        this.isFollowing = true;
        headingController.reset();
    }

    public void stopFollowing() {
        isFollowing = false;
        if (swerveDrive != null) {
            if (brakeOnFinish) {
                swerveDrive.setZeroPowerMode(finishPowerMode);
            } else {
                swerveDrive.setZeroPowerMode(SwerveModule.ZeroPowerMode.FLOAT);
            }
            swerveDrive.manual(0, 0, 0);
        }
    }

    public boolean isBusy() {
        return isFollowing;
    }

    private void updateTargetIndex(double curX, double curY) {
        for (int i = currentTargetIndex; i < fullPathPoints.size(); i++) {
            Pose p = fullPathPoints.get(i);
            double dist = Math.hypot(p.x - curX, p.y - curY);
            if (dist >= lookaheadDistance) {
                currentTargetIndex = i;
                return;
            }
        }
        currentTargetIndex = fullPathPoints.size() - 1;
    }
}