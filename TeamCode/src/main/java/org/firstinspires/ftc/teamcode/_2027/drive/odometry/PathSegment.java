package org.firstinspires.ftc.teamcode._2027.drive.odometry;

import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.List;

public interface PathSegment {
    List<Pose> generatePath(int resolution);
}