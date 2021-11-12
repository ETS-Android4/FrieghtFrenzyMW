package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "TeleOp", group = "teleop")

public class Teleop extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    private DcMotor lifter;
    private DcMotor intake;

    private double theta = 22.5;
    private double delta = 45;
    private double speed = 0;
    private double calc_power;
    private double stick_directon = 0;

    private int direction = -1;
    // Setting scaling to full speed.
    private double scaleFactor = 1;
    private double scaleTurningSpeed = .8;

    String drivingState = "";

    public void runOpMode() {


       rightFront =  hardwareMap.dcMotor.get("right_front");
       rightBack =  hardwareMap.dcMotor.get("right_back");
       leftFront =  hardwareMap.dcMotor.get("left_front");
       leftBack =  hardwareMap.dcMotor.get("left_back");
       lifter =  hardwareMap.dcMotor.get("lifter");
       intake =  hardwareMap.dcMotor.get("intake");
       rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
       rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
       waitForStart();


        while (opModeIsActive()) {

            drivingState = "";

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
// When the direction value is reversed this if statement inverts the addition and subtraction for turning.
// Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
            if (direction == 1) {
                final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            } else {
                final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            }

            if (gamepad1.dpad_up==true&&gamepad1.dpad_down==true){
                lifter.setPower(0);
            }else{
                if (gamepad1.dpad_up==true){
                    lifter.setPower(-.5);
                }
                if (gamepad1.dpad_down==true){
                    lifter.setPower(.5);
                }
            }



            if (gamepad1.left_trigger==0&&gamepad1.right_trigger==0){
                intake.setPower(0);
            }
            if (gamepad1.left_trigger>0&&gamepad1.right_trigger>0){
                intake.setPower(0);
            }else{
                if (gamepad1.right_trigger>0){
                    intake.setPower(1);
                }
                if (gamepad1.left_trigger>0){
                    intake.setPower(-1);
                }
            }

        }


    }
}