package org.firstinspires.ftc.teamcode._2027.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

public class Intake extends OpBehavior {
    private Intake(){}
    private final static Intake s_instance;
    static {
        s_instance = new Intake();
        if(OpObject.getMode() == OpObject.Mode.Competition){
            OpObject.RegisterGlobalBehavior(s_instance);
        }
        OpObject.RegisterGlobalBehavior(s_instance);

    }
    public static Intake get_instance(){
        return s_instance;
    }
    private DcMotorEx intake;
    private DcMotorEx intakeCtrl;

    @Override
    public void Init() {
        intake = m_object.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeCtrl = m_object.hardwareMap.get(DcMotorEx.class, "intakeCtrl");
        intakeCtrl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
//        if(intake.getCurrent(CurrentUnit.AMPS) > 0.5 || intakeCtrl.getCurrent(CurrentUnit.AMPS) > 0.5){
//            mode = Mode.FAULT;
//        }
        switch (mode){
            case IDLE:
                intake.setPower(0);
                intakeCtrl.setPower(0);
                break;
            case SHOOT:
                intake.setPower(1);
                intakeCtrl.setPower(-1);
                break;
            case INTAKE:
                intake.setPower(1);
                intakeCtrl.setPower(0.5);
                break;
            case OUT:
                intake.setPower(-1);
                intakeCtrl.setPower(0.5);
                break;
            case FAULT:
                break;
            default:
                mode = Mode.IDLE;
                break;
        }
    }

    @Override
    public void Stop() {
        intake = null;
        intakeCtrl = null;
    }

    public enum Mode{
        IDLE, INTAKE, SHOOT, FAULT, OUT
    }
    private static Mode mode = Mode.IDLE;
    public static void setMode(Mode mode){
        Intake.mode = mode;
    }
    public static Mode getMode(){
        return mode;
    }
}