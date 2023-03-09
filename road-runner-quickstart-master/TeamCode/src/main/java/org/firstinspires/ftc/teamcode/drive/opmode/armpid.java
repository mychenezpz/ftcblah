package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class armpid extends OpMode {
    private int position=0;
    DcMotorEx arm;
    @Override
    public void init() {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm=hardwareMap.get(DcMotorEx.class, "motor5");
    }

    @Override
    public void loop() {
        int armPos=arm.getCurrentPosition();
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(position);
        arm.setTargetPositionTolerance(20);
        arm.setPower(-1.0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (arm.getCurrentPosition()==arm.getTargetPositionTolerance()) {
            telemetry.addData("it worked","lol");
        }
        telemetry.addData("pos", armPos);
        telemetry.update();
    }
}
