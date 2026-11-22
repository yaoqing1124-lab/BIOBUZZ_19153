package org.firstinspires.ftc.teamcode._2027.subsystem.drive.swerve;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry.Pinpoint;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.util.PIDFController;

public class Swerve extends OpBehavior {
    private Swerve(){}
    private final static Swerve s_instance;
    static {
        s_instance = new Swerve();
        OpObject.RegisterGlobalBehavior(s_instance);
    }
    public static Swerve get_instance(){
        return s_instance;
    }

    @Override
    public void Init() {
        lf = new SwerveModule(SwerveConfig.lf, m_object.hardwareMap);
        lb = new SwerveModule(SwerveConfig.lb, m_object.hardwareMap);
        rf = new SwerveModule(SwerveConfig.rf, m_object.hardwareMap);
        rb = new SwerveModule(SwerveConfig.rb, m_object.hardwareMap);

        headingController = new PIDFController(0.005, 0.000001, 0.003, 0.0);
//        headingController = new PIDFController(0, 0.0, 0, 0.0);
        stopWatch.reset();
    }

    @Override
    public void Start() {
        targetHeading = Pinpoint.getHeading();
        stopWatch.reset();
    }

    @Override
    public void Loop() {
        update();
    }

    @Override
    public void Stop() {
        lf = null;
        lb = null;
        rf = null;
        rb = null;
    }

    private SwerveModule rf;
    private SwerveModule lf;
    private SwerveModule lb;
    private SwerveModule rb;

    public static double axial;
    public static double lateral;
    public static double yaw;

    // 鎖航向與緩衝相關變數
    private double targetHeading = 0.0;
    private PIDFController headingController;
    private boolean wasRotating = false;
    private ElapsedTime stopWatch = new ElapsedTime(); // 計算旋轉停止後的時間
    private static final double RELEASE_DELAY_MS = 100; // 剛放開轉向後的緩衝時間（毫秒）

    // 以場地為中心 (Field-Centric) 模式變數
    private static boolean isFieldCentric = true; // 預設啟用場地中心模式
    private double fieldCentricOffset = 0.0; // 場地正前方歸零偏移量

    private static SwerveModule.ZeroPowerMode currentZeroPowerMode = SwerveModule.ZeroPowerMode.HOLD;

    public static void setZeroPowerMode(SwerveModule.ZeroPowerMode mode) {
        Swerve.currentZeroPowerMode = mode;
    }
    public static SwerveModule.ZeroPowerMode getZeroPowerMode(){
        return Swerve.currentZeroPowerMode;
    }

    public static void setFieldCentric(boolean fieldCentric) {
        Swerve.isFieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }

    public void resetFieldCentricOffset() {
        this.fieldCentricOffset = Pinpoint.getHeading();
    }

    public void manual(double lateral, double axial, double yaw){
        Swerve.lateral = axial;
        Swerve.axial = lateral;
        Swerve.yaw = yaw;
        inputYaw = yaw;
    }
    double inputYaw;
    public void update(){
        // 1. 取得搖桿輸入的原始平移量
        double inputAxial = axial;
        double inputLateral = lateral;

        // 2. 以場地為中心 (Field-Centric) 座標轉換
        if (isFieldCentric) {
            double robotHeadingRad = Math.toRadians(Pinpoint.getHeading() - fieldCentricOffset);
            double rotatedAxial = inputAxial * Math.cos(robotHeadingRad) - inputLateral * Math.sin(robotHeadingRad);
            double rotatedLateral = inputAxial * Math.sin(robotHeadingRad) + inputLateral * Math.cos(robotHeadingRad);
            inputAxial = rotatedAxial;
            inputLateral = rotatedLateral;
        }

        // 3. 鎖航向與旋轉緩衝邏輯
        boolean isRotating = Math.abs(yaw) > 0.01;

        if (isRotating) {
            // 正在旋轉中：不斷更新目標角度，並重置計時器
            targetHeading = Pinpoint.getHeading();
            headingController.reset();
            wasRotating = true;
            stopWatch.reset();
            lf.update(inputAxial, inputLateral, yaw, currentZeroPowerMode);
            lb.update(inputAxial, inputLateral, yaw, currentZeroPowerMode);
            rf.update(inputAxial, inputLateral, yaw, currentZeroPowerMode);
            rb.update(inputAxial, inputLateral, yaw, currentZeroPowerMode);
        } else {
            // 沒有給旋轉指令
            if (wasRotating) {
                wasRotating = false;
                stopWatch.reset();
            }

            // 檢查放開轉向後是否已經過了緩衝時間 (RELEASE_DELAY_MS)
            if (stopWatch.milliseconds() < RELEASE_DELAY_MS) {
                // 剛放開的瞬間：不進行鎖向補正，讓慣性自然滑行
                targetHeading = Pinpoint.getHeading();
                headingController.reset();
            } else {
                // 超過緩衝時間後：正式啟動鎖航向
                double currentHeading = Pinpoint.getHeading();
                double error = targetHeading - currentHeading;
                while (error > 180) error -= 360;
                while (error < -180) error += 360;
                if(Math.abs(error) > 5) {
                    yaw = headingController.calculate(error);
                }
            }
            lf.update(inputAxial, inputLateral, yaw, currentZeroPowerMode);
            lb.update(inputAxial, inputLateral, yaw, currentZeroPowerMode);
            rf.update(inputAxial, inputLateral, yaw, currentZeroPowerMode);
            rb.update(inputAxial, inputLateral, yaw, currentZeroPowerMode);
        }
    }
}