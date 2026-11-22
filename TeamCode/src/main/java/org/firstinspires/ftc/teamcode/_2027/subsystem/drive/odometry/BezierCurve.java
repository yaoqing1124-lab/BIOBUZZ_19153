package org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry;

import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.List;

public class BezierCurve implements PathSegment {
    private final List<Pose> controlPoints = new ArrayList<>();

    public BezierCurve(Pose start, Pose... points) {
        controlPoints.add(start);
        for (Pose p : points) {
            controlPoints.add(p);
        }
    }

    @Override
    public List<Pose> generatePath(int resolution) {
        List<Pose> path = new ArrayList<>();
        int n = controlPoints.size() - 1;

        for (int i = 0; i <= resolution; i++) {
            double t = (double) i / resolution;
            Pose point = evaluateBezier(t, n);
            path.add(point);
        }
        return path;
    }

    private Pose evaluateBezier(double t, int n) {
        double x = 0;
        double y = 0;

        for (int i = 0; i <= n; i++) {
            double bernstein = binomialCoefficient(n, i) * Math.pow(1 - t, n - i) * Math.pow(t, i);
            Pose p = controlPoints.get(i);
            x += bernstein * p.x;
            y += bernstein * p.y;
        }

        double startHeading = controlPoints.get(0).heading;
        double endHeading = controlPoints.get(controlPoints.size() - 1).heading;
        double heading = interpolateHeading(startHeading, endHeading, t);

        return new Pose(x, y, heading);
    }

    private double binomialCoefficient(int n, int k) {
        return factorial(n) / (factorial(k) * factorial(n - k));
    }

    private double factorial(int num) {
        double res = 1;
        for (int i = 2; i <= num; i++) {
            res *= i;
        }
        return res;
    }

    private double interpolateHeading(double startDeg, double endDeg, double t) {
        double diff = endDeg - startDeg;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return startDeg + diff * t;
    }
}
