package org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry;

import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.List;

public interface PathSegment {
    List<Pose> generatePath(int resolution);
}