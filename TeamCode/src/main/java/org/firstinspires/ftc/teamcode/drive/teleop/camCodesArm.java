package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp( name = "Cam Codes a Scuffed Teleop--> Arm")
public class camCodesArm extends LinearOpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    DcMotorEx armPivot = null;
    private double basePower = 0.4;



    @Override
    public void runOpMode() throws InterruptedException {


        armPivot = hardwareMap.get(DcMotorEx.class, "clawRot1");

        //separates initialization movement & start movement
        waitForStart();








        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * basePower,
                        -(gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.left_stick_x) * basePower * 1.2,
                        -gamepad1.right_stick_x * basePower
                )
        );
        drive.update();





    }
}
