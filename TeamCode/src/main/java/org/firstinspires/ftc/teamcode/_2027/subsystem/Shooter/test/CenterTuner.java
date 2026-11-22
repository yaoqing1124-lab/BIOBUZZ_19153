package org.firstinspires.ftc.teamcode._2027.subsystem.Shooter.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.arch.utils.Debug;

@TeleOp(name = "Turret Center Tuner")
public class CenterTuner extends OpMode {
    private Servo s_servo;
    private Servo l_servo;
    double s_position = 0.5;
    double l_position = 0.5;

    @Override
    public void init() {
        s_servo = hardwareMap.get(Servo.class, "Sturret");
        l_servo = hardwareMap.get(Servo.class, "Lturret");
    }

    @Override
    public void loop() {
        s_position = gamepad1.squareWasPressed()   ? s_position - 0.1  : s_position;
        s_position = gamepad1.circleWasPressed()   ? s_position + 0.1  : s_position;
        s_position = gamepad1.crossWasPressed()    ? s_position - 0.01 : s_position;
        s_position = gamepad1.triangleWasPressed() ? s_position + 0.01 : s_position;
        l_position = gamepad1.dpadLeftWasPressed() ? l_position - 0.1  : l_position;
        l_position = gamepad1.dpadRightWasPressed()? l_position + 0.1  : l_position;
        l_position = gamepad1.dpadDownWasPressed() ? l_position - 0.01 : l_position;
        l_position = gamepad1.dpadUpWasPressed()   ? l_position + 0.01 : l_position;
        s_position = Range.clip(s_position, -1, 1);
        l_position = Range.clip(l_position, -1, 1);

        s_servo.setPosition(s_position);
        l_servo.setPosition(l_position);

        telemetry.addData("s_Position", s_servo.getPosition());
        telemetry.addData("l_Position", l_servo.getPosition());
    }
}
