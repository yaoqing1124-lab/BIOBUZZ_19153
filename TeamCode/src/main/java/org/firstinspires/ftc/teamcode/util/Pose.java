package org.firstinspires.ftc.teamcode.util;

public class Pose {
    public double x;
    public double y;
    public double heading;
    public Pose(double x, double y){
        this.x = x;
        this.y = y;
        this.heading = 0;
    }
    public Pose(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
