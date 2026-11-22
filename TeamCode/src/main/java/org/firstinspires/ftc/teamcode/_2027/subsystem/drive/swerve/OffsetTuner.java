package org.firstinspires.ftc.teamcode._2027.subsystem.drive.swerve;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode._2027.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Pitch;
import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Turret;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

@TeleOp(name = "Swerve Offset Tuner")
public class OffsetTuner extends OpObject {

    private AnalogInput lfEncoder;
    private AnalogInput lbEncoder;
    private AnalogInput rfEncoder;
    private AnalogInput rbEncoder;

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
        // 直接從 SwerveConfig 定義的名稱獲取 4 個底盤模組的 Analog Input 裝置
        lfEncoder = hardwareMap.get(AnalogInput.class, SwerveConfig.lf.EncoderName);
        lbEncoder = hardwareMap.get(AnalogInput.class, SwerveConfig.lb.EncoderName);
        rfEncoder = hardwareMap.get(AnalogInput.class, SwerveConfig.rf.EncoderName);
        rbEncoder = hardwareMap.get(AnalogInput.class, SwerveConfig.rb.EncoderName);
    }

    @Override
    public void Start() {
        // 程式開始時不需額外動作
    }

    @Override
    public void Loop() {
        // 即時在 Driver Hub 上顯示 4 個底盤模組的 Analog Voltage 數值
        telemetry.addData("=== Swerve Analog Encoders ===", "");
        telemetry.addData("LF Encoder (lfe)", lfEncoder != null ? String.format("%.3f V", lfEncoder.getVoltage()) : "Not Found");
        telemetry.addData("LB Encoder (lbe)", lbEncoder != null ? String.format("%.3f V", lbEncoder.getVoltage()) : "Not Found");
        telemetry.addData("RF Encoder (rfe)", rfEncoder != null ? String.format("%.3f V", rfEncoder.getVoltage()) : "Not Found");
        telemetry.addData("RB Encoder (rbe)", rbEncoder != null ? String.format("%.3f V", rbEncoder.getVoltage()) : "Not Found");
        telemetry.update();
    }

    @Override
    public void Stop() {
        // 停止時的清理邏輯
    }
}