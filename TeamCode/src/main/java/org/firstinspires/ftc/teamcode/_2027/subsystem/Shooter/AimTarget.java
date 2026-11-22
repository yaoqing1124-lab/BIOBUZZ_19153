package org.firstinspires.ftc.teamcode._2027.subsystem.Shooter;

import org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry.Pinpoint;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.util.Pose;

public class AimTarget extends OpBehavior {
    private AimTarget(){}
    private final static AimTarget s_instance;
    static {
        s_instance = new AimTarget();
        OpObject.RegisterGlobalBehavior(s_instance);
    }
    public static AimTarget get_instance(){
        return s_instance;
    }
    private Pose currentPose;
    private Pose targetPose;
    @Override
    public void Init() {
        targetPose = new Pose(0, 0);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        currentPose = Pinpoint.getPose();
        if(OpObject.getMode() == OpObject.Mode.Test) {
            targetPose = new Pose(12, -19);
        }
//        switch (Config.team){
//            case BLUE:
//                if(currentPose.y > 0){
//                    targetPose.y = currentPose.y > 20 ? targetPose.y : 19;
//                }
//                break;
//            case RED:
//                break;
//        }
    }

    @Override
    public void Stop() {

    }
    public Pose getTargetPose(){
        return targetPose;
    }
}
