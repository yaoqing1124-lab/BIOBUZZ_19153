package org.firstinspires.ftc.teamcode._2027.subsystem.drive.swerve;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode._2027.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Pitch;
import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Turret;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

@TeleOp(name = "Swerve MinMax Tuner")
public class MinMaxTuner extends OpObject {
    private AnalogInput lfEncoder;
    private AnalogInput lbEncoder;
    private AnalogInput rfEncoder;
    private AnalogInput rbEncoder;

    // 記錄 4 個模組的最小與最大電壓極值
    private static double lfMin = 3.3, lfMax = 0.0;
    private static double lbMin = 3.3, lbMax = 0.0;
    private static double rfMin = 3.3, rfMax = 0.0;
    private static double rbMin = 3.3, rbMax = 0.0;

    @Override
    public void Init() {
        Swerve.setZeroPowerMode(SwerveModule.ZeroPowerMode.FLOAT);
        UnregisterBehavior(Turret.get_s_instance());
        UnregisterBehavior(Turret.get_l_instance());
        UnregisterBehavior(Flywheel.get_s_instance());
        UnregisterBehavior(Flywheel.get_l_instance());
        UnregisterBehavior(Pitch.get_s_instance());
        UnregisterBehavior(Pitch.get_l_instance());
        UnregisterBehavior(Intake.get_instance());
        // 從 SwerveConfig 取得 4 個模組的編碼器裝置名稱
        lfEncoder = hardwareMap.get(AnalogInput.class, SwerveConfig.lf.EncoderName);
        lbEncoder = hardwareMap.get(AnalogInput.class, SwerveConfig.lb.EncoderName);
        rfEncoder = hardwareMap.get(AnalogInput.class, SwerveConfig.rf.EncoderName);
        rbEncoder = hardwareMap.get(AnalogInput.class, SwerveConfig.rb.EncoderName);

        telemetry.addData("Status", "Initialized. Rotate all modules manually to find Min/Max.");
        telemetry.update();
    }

    @Override
    public void Start() {
        lfMin = 3.3; lfMax = 0.0;
        lbMin = 3.3; lbMax = 0.0;
        rfMin = 3.3; rfMax = 0.0;
        rbMin = 3.3; rbMax = 0.0;
    }

    @Override
    public void Loop() {
        // 更新並追蹤各個編碼器的極值
        if (lfEncoder != null) {
            double v = lfEncoder.getVoltage();
            if (v < lfMin) lfMin = v;
            if (v > lfMax) lfMax = v;
        }
        if (lbEncoder != null) {
            double v = lbEncoder.getVoltage();
            if (v < lbMin) lbMin = v;
            if (v > lbMax) lbMax = v;
        }
        if (rfEncoder != null) {
            double v = rfEncoder.getVoltage();
            if (v < rfMin) rfMin = v;
            if (v > rfMax) rfMax = v;
        }
        if (rbEncoder != null) {
            double v = rbEncoder.getVoltage();
            if (v < rbMin) rbMin = v;
            if (v > rbMax) rbMax = v;
        }

        // 在 Driver Hub 上即時輸出 4 顆輪子的即時電壓與已記錄的 Min/Max
        telemetry.addData("=== 4-Encoder Min/Max Tuning ===", "");

        telemetry.addData("[LF] Left-Front", "Cur: %.3fV | Min: %.3fV | Max: %.3fV (Span: %.3fV)",
                lfEncoder != null ? lfEncoder.getVoltage() : 0.0, lfMin, lfMax, lfMax - lfMin);

        telemetry.addData("[LB] Left-Back", "Cur: %.3fV | Min: %.3fV | Max: %.3fV (Span: %.3fV)",
                lbEncoder != null ? lbEncoder.getVoltage() : 0.0, lbMin, lbMax, lbMax - lbMin);

        telemetry.addData("[RF] Right-Front", "Cur: %.3fV | Min: %.3fV | Max: %.3fV (Span: %.3fV)",
                rfEncoder != null ? rfEncoder.getVoltage() : 0.0, rfMin, rfMax, rfMax - rfMin);

        telemetry.addData("[RB] Right-Back", "Cur: %.3fV | Min: %.3fV | Max: %.3fV (Span: %.3fV)",
                rbEncoder != null ? rbEncoder.getVoltage() : 0.0, rbMin, rbMax, rbMax - rbMin);
        telemetry.update();
    }

    @Override
    public void Stop() {
    }
}