package org.firstinspires.ftc.teamcode._2027.state;

import org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry.Follower;
import org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry.PathSegment;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;

import java.util.List;

public class Follow extends StateObject {
    private final List<PathSegment> segments;
    private final boolean brake;
    private Follower follower;

    public Follow(List<PathSegment> segments, boolean brake) {
        this.segments = segments;
        this.brake = brake;
    }

    @Override
    public void Init(double timestamp) {
        follower = Follower.get_instance();
        // 傳入更新後的路徑區段列表與煞車設定
        follower.setCustomSegmentsPath(segments, brake);
    }

    @Override
    public void Loop(double timestamp, double deltaTime) {
        // Follower 會作為 OpBehavior 自動在背景執行 Loop 更新
    }

    @Override
    public boolean IsDone(double timestamp) {
        // 當 Follower 執行完所有路徑點後，狀態即完成
        return follower != null && !follower.isBusy();
    }
}