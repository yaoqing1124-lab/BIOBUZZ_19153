package org.firstinspires.ftc.teamcode._2027.config;

import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.util.Team;

public class Config {
    public static Team team = Team.BLUE;
    // +---------------------------------+
    // |                               　|
    // |           底盤參數               |
    // |　                               |
    // +---------------------------------+
    public static class Drive{
        public static String[] Name = {null, "rf", "lf", "lb", "rb"};
        public static class offset{
            public static double lf = 0.401;
            public static double lb = 0.245;
            public static double rf = 1.559;
            public static double rb = 1.025;
        }
        public static class minVoltage{
            public static double lf = 0.004;
            public static double lb = 0.004;
            public static double rf = 0.003;
            public static double rb = 0.004;
        }
        public static class maxVoltage{
            public static double lf = 3.197;
            public static double lb = 3.195;
            public static double rf = 3.191;
            public static double rb = 3.194;
        }
        public static class pinpoint{
            public static String name = "pinpoint";
            public static double Xoffset = 60 / 25.4;
            public static double Yoffset = 16.6 / 25.4;
        }
    }
    // +---------------------------------+
    // |                               　|
    // |           炮台參數               |
    // |　                               |
    // +---------------------------------+
    public static class Shooter{
        public static final ShooterConfig S = new ShooterConfig()
                .turretName("Sturret")
                .flywheelName("Sflywheel")
                .pitchName("Spitch")
                .offset(-93 / 25.4)
                .center(0.68)
                .flywheelInstance(Flywheel.get_s_instance());
        public static final ShooterConfig L = new ShooterConfig()
                .turretName("Lturret")
                .flywheelName("Lflywheel")
                .pitchName("Lpitch")
                .offset(-93 / 25.4)
                .center(0.05)
                .flywheelInstance(Flywheel.get_s_instance());
    }
}
