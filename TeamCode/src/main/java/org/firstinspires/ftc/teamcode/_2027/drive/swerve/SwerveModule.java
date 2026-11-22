package org.firstinspires.ftc.teamcode._2027.drive.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDFController;

public class SwerveModule {
    public enum ZeroPowerMode {
        X_LOCK,
        HOLD,
        FLOAT
    }

    private final SwerveConfig config;
    private final DcMotorEx m_motor;
    private final CRServo m_servo;
    private final AnalogInput m_encoder;

    // 宣告專門給 CRServo 使用的 PIDF 控制器
    private final PIDFController servoPIDF;

    private double lastTargetAngle = 0.0;

    public SwerveModule(SwerveConfig config, HardwareMap hw){
        this.config = config;
        m_motor   = hw.get(DcMotorEx.class,   config.MotorName);
        m_servo   = hw.get(CRServo.class,     config.ServoName);
        m_encoder = hw.get(AnalogInput.class, config.EncoderName);

        m_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ★ 從 config 取得 PIDF 係數來初始化控制器
        this.servoPIDF = new PIDFController(
                config.pidfCoefficients.p,
                config.pidfCoefficients.i,
                config.pidfCoefficients.d,
                config.pidfCoefficients.f
        );
    }

    private double voltage2Radian(double voltage){
        return voltage / (config.MaxVoltage - config.MinVoltage) * (2 * Math.PI);
    }

    private double CurrentRadian(){
        double rawAngle = voltage2Radian(m_encoder.getVoltage());
        double correctedAngle = rawAngle - config.ServoOffset;

        correctedAngle = correctedAngle % (2 * Math.PI);
        if (correctedAngle < 0) {
            correctedAngle += (2 * Math.PI);
        }
        return correctedAngle;
    }

    public void update(double Vx, double Vy, double Vangular, ZeroPowerMode zeroPowerMode) {
        double targetVelocity;
        double targetAngle;

        if (Math.abs(Vx) < 0.001 && Math.abs(Vy) < 0.001 && Math.abs(Vangular) < 0.001) {
            targetVelocity = 0.0;

            switch (zeroPowerMode) {
                case X_LOCK:
                    switch (config.Quadrant) {
                        case 1: targetAngle = Math.toRadians(225); break;
                        case 2: targetAngle = Math.toRadians(315); break;
                        case 3: targetAngle = Math.toRadians(45);  break;
                        case 4: targetAngle = Math.toRadians(135); break;
                        default: targetAngle = 0.0; break;
                    }
                    lastTargetAngle = targetAngle;
                    break;

                case HOLD:
                    targetAngle = lastTargetAngle;
                    break;

                case FLOAT:
                default:
                    m_motor.setPower(0);
                    m_servo.setPower(0);
                    servoPIDF.reset();
                    return;
            }
        } else {
            double vxComponent = Vx + Vangular * (config.ModulePose.y / Math.hypot(config.ModulePose.x, config.ModulePose.y));
            double vyComponent = Vy - Vangular * (config.ModulePose.x / Math.hypot(config.ModulePose.x, config.ModulePose.y));

            targetVelocity = Math.hypot(vxComponent, vyComponent);
            targetAngle = Math.atan2(vyComponent, vxComponent);
            lastTargetAngle = targetAngle;
        }

        double currentAngle = CurrentRadian();
        double angleError = targetAngle - currentAngle;
        angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));

        // 最短路徑最佳化
        if (Math.abs(angleError) > Math.PI / 2) {
            targetVelocity *= -1;
            angleError -= Math.signum(angleError) * Math.PI;
        }

        // 驅動輪輸出
        m_motor.setPower(targetVelocity);

        // 使用從 Config 讀取的 PIDF 參數計算轉向功率
        double servoPower = servoPIDF.calculate(angleError);
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

        m_servo.setPower(servoPower);
    }
}