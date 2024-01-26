package org.firstinspires.ftc.teamcode.drive.teleop;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp( name = "Cam Codes a Scuffed Teleop--> Arm")
public class camCodesArm extends LinearOpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    DcMotorEx armPivot = null;
    private double basePower = 0.4;

    private static final int MAX_PIVOT_POS = 4055;

    private double armPivotPower = 1;

    @Override
    public void runOpMode() throws InterruptedException {


        armPivot = hardwareMap.get(DcMotorEx.class, "clawLift1");

        armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivot.setPower(0);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //separates initialization movement & start movement
        waitForStart();
        //drive code
        if ((gamepad2.left_stick_y * armPivotPower * -1 > 0 && armPivot.getCurrentPosition() < MAX_PIVOT_POS - 50) || (gamepad2.left_stick_y * armPivotPower * -1 < 0 && armPivot.getCurrentPosition() > 100) || (!(armPivot.getCurrentPosition() <= 100) && gamepad2.left_stick_y != 0 && !(armPivot.getCurrentPosition() >= MAX_PIVOT_POS  - 50))) {
            if (gamepad2.left_stick_y * -1 < 0) {
                armPivot.setPower(0);
            } else if (gamepad2.left_stick_y != 0) {
                armPivot.setPower(gamepad2.left_stick_y * armPivotPower * -1);
                armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("currently moving", "clawLift1");
                telemetry.addData("clawPivot Power", armPivot.getPower());
            } else {
                armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else if (armPivot.getCurrentPosition() >= 100 && armPivot.getCurrentPosition() <= MAX_PIVOT_POS - 50) {
            armPivot.setPower(0);
        } else if(armPivot.getCurrentPosition() < 100) {
            if (armPivot.getCurrentPosition() < 0) {
                armPivot.setPower(armPivotPower * 0.1);
            } else {
                armPivot.setPower(armPivotPower * 0.1 * -1);
            }
            armPivot.setTargetPosition(0);
            armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(armPivot.getCurrentPosition() > MAX_PIVOT_POS - 50) {
            if (armPivot.getCurrentPosition() < MAX_PIVOT_POS) {
                armPivot.setPower(MAX_PIVOT_POS * 0.1);
            } else {
                armPivot.setPower(armPivotPower * 0.1 * -1);
            }
            armPivot.setTargetPosition(MAX_PIVOT_POS);
            armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("Claw Pivot Position", armPivot.getCurrentPosition());
        telemetry.update();







            //this is the basic drive code
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
