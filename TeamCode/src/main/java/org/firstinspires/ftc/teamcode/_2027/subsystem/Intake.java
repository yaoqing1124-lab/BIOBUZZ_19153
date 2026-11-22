package org.firstinspires.ftc.teamcode._2027.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode._2027.drive.swerve.Swerve;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

public class Intake extends OpBehavior {
    private Intake(){}
    private final static Intake s_instance;
    static {
        s_instance = new Intake();
//        OpObject.RegisterGlobalBehavior(s_instance);
    }
    public static Intake get_instance(){
        return s_instance;
    }
    private DcMotorEx intake;
    private DcMotorEx intakeCtrl;

    @Override
    public void Init() {
        intake = m_object.hardwareMap.get(DcMotorEx.class, "intake");
        intakeCtrl = m_object.hardwareMap.get(DcMotorEx.class, "intakeCtrl");
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        if(intake.getCurrent(CurrentUnit.AMPS) > 0.5 || intakeCtrl.getCurrent(CurrentUnit.AMPS) > 0.5){
            mode = Mode.FAULT;
        }
        switch (mode){
            case IDLE:
                intake.setPower(0);
                intakeCtrl.setPower(0);
            case SHOOT:
                intake.setPower(1);
                intakeCtrl.setPower(-1);
            case INTAKE:
                intake.setPower(1);
                intakeCtrl.setPower(0.5);
            case FAULT:
                mode = Mode.INTAKE;
        }
    }

    @Override
    public void Stop() {
        intake = null;
        intakeCtrl = null;
    }

    public enum Mode{
        IDLE, INTAKE, SHOOT, FAULT
    }
    private static Mode mode = Mode.IDLE;
    public void setMode(Mode mode){
        Intake.mode = mode;
    }
}