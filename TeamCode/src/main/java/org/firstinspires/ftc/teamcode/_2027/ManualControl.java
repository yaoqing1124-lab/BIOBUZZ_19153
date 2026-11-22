package org.firstinspires.ftc.teamcode._2027;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode._2027.config.Config;
import org.firstinspires.ftc.teamcode._2027.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode._2027.subsystem.drive.odometry.Pinpoint;
import org.firstinspires.ftc.teamcode._2027.subsystem.drive.swerve.Swerve;
import org.firstinspires.ftc.teamcode._2027.subsystem.drive.swerve.SwerveModule;
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
    public void Start() {
    }

    @Override
    public void Loop() {
        if (gamepad1.circleWasPressed()) {
            Swerve.get_instance().resetFieldCentricOffset();
        }
        if (gamepad1.left_bumper) {
            Intake.setMode(Intake.Mode.INTAKE);
        } else if (gamepad1.left_trigger > 0.5) {
            Intake.setMode(Intake.Mode.OUT);
        } else if (gamepad1.right_trigger > 0.5) {
            Intake.setMode(Intake.Mode.SHOOT);
        } else {
            Intake.setMode(Intake.Mode.IDLE);
        }

        double drivePower = 1;
        Swerve.setZeroPowerMode(SwerveModule.ZeroPowerMode.FLOAT);
        drive.manual(-drivePower * gamepad1.left_stick_y, drivePower * gamepad1.left_stick_x, -drivePower * gamepad1.right_stick_x);
        Debug.Log("Pose : (", Pinpoint.getPose2D().getX(DistanceUnit.INCH), ",", Pinpoint.getPose2D().getY(DistanceUnit.INCH), ")");
        Debug.Log("Heading : ", Pinpoint.getHeading());
        Debug.Log("s_TargetTurretPosition : ", Turret.get_s_instance().degree2position(Turret.get_s_instance().calculateFieldAimDegree()));
        Debug.Log("s_FieldAimDegree", Turret.get_s_instance().calculateFieldAimDegree());
        Debug.Log("DriveMode", Swerve.getZeroPowerMode().toString());
        Debug.Log("IntakeMode", Intake.getMode());
        Debug.Log("LeftTrigger", gamepad1.left_trigger);
        Debug.Log("FlyWheelVelocity", Flywheel.get_s_instance().getVelocity());
    }

    @Override
    public void Stop() {
    }
}