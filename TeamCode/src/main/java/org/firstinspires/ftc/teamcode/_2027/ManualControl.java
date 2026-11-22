package org.firstinspires.ftc.teamcode._2027;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode._2027.drive.odometry.Pinpoint;
import org.firstinspires.ftc.teamcode._2027.drive.swerve.Swerve;
import org.firstinspires.ftc.teamcode._2027.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Turret;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.arch.utils.Debug;

@TeleOp
public class ManualControl extends OpObject {
    public Swerve drive;

    @Override
    public void Init() {
        drive = Swerve.get_instance();
    }

    @Override
    public void Start() {}

    @Override
    public void Loop() {
        Swerve.setZeroPowerMode(SwerveModule.ZeroPowerMode.FLOAT);
        drive.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        Debug.Log("Pose : (", Pinpoint.getPose2D().getX(DistanceUnit.INCH), ",",Pinpoint.getPose2D().getY(DistanceUnit.INCH), ")");
        Debug.Log("Heading : ", Pinpoint.getHeading());
        Debug.Log("s_TargetTurretPosition : ", Turret.get_s_instance().degree2position(Turret.get_s_instance().calculateFieldAimDegree()));
        Debug.Log("s_FieldAimDegree", Turret.get_s_instance().calculateFieldAimDegree());
    }

    @Override
    public void Stop() {}
}