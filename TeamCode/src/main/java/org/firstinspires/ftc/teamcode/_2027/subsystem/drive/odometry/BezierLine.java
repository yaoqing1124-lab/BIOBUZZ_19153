package org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry;

import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.List;

public class BezierLine implements PathSegment {
    private final Pose start;
    private final Pose end;

    public BezierLine(Pose start, Pose end) {
        this.start = start;
        this.end = end;
    }

    @Override
    public List<Pose> generatePath(int resolution) {
        List<Pose> path = new ArrayList<>();
        for (int i = 0; i <= resolution; i++) {
            double t = (double) i / resolution;
            double x = start.x + (end.x - start.x) * t;
            double y = start.y + (end.y - start.y) * t;

            double heading = interpolateHeading(start.heading, end.heading, t);
            path.add(new Pose(x, y, heading));
        }
        return path;
    }

    private double interpolateHeading(double startDeg, double endDeg, double t) {
        double diff = endDeg - startDeg;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return startDeg + diff * t;
    }
}
