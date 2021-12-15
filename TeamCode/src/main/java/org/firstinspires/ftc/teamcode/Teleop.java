package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "TeleOp", group = "teleop")

public class Teleop extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    private DcMotor duckspinner;
    private DcMotor lifter;
    private DcMotor intake;
    private CRServo bucket;



    private double theta = 22.5;
    private double delta = 45;
    private double speed = 0;
    private double calc_power;
    private double stick_directon = 0;
    private int bucket_case = 1;

    private int direction = -1;
    // Setting scaling to full speed.
    private double scaleFactor = 1;
    private double scaleTurningSpeed = .8;
    //Previously 2

    public static boolean SwitchBucket = true;
    boolean BucketPressed = false;

    String drivingState = "";

    public void runOpMode() {


       rightFront =  hardwareMap.dcMotor.get("right_front");
       rightBack =  hardwareMap.dcMotor.get("right_back");
       leftFront =  hardwareMap.dcMotor.get("left_front");
       leftBack =  hardwareMap.dcMotor.get("left_back");
       lifter =  hardwareMap.dcMotor.get("lifter");
       intake =  hardwareMap.dcMotor.get("intake");
       bucket = hardwareMap.crservo.get("bucket");
       duckspinner = hardwareMap.dcMotor.get("duck_spinner");

       lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
       leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


       waitForStart();



        while (opModeIsActive()) {
            telemetry.addData("Current lifter height: ", rightFront.getCurrentPosition()*1000);
            telemetry.update();
            drivingState = "";

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
// When the direction value is reversed this if statement inverts the addition and subtraction for turning.
// Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
            if (direction == 1) {
                final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            } else {

                final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            }
            int input = 0;
            if (gamepad2.dpad_right){
                input++;
            }
            if (gamepad2.dpad_left){
                input--;
            }
            if (input == 1){
                duckspinner.setPower(.8f);
            }
            if (input == -1){
                duckspinner.setPower(-.8f);
            }
            if (input == 0){
                duckspinner.setPower(0);
            }

            //This script makes sure that the motor is not turning in two ways at one time.
            if (gamepad2.dpad_up==true&&gamepad2.dpad_down==true){
                lifter.setPower(0);
            }else {
                boolean ison = false;
                if (gamepad2.dpad_down==true&&gamepad2.dpad_up == true){
                    lifter.setPower(0);
                }else{
                    if (gamepad2.dpad_up == true){
                        lifter.setPower(-.2);
                        ison = true;
                    }
                    if (gamepad2.dpad_down == true){
                        lifter.setPower(.2);
                        ison= true;
                    }
                }

                if (ison == false){
                    lifter.setPower(0);
                }
            }



            if (gamepad2.left_trigger==0&&gamepad2.right_trigger==0){
                intake.setPower(0);
            }
            if (gamepad2.left_trigger>0&&gamepad2.right_trigger>0){
                intake.setPower(0);
            }else{
                if (gamepad2.right_trigger>0){
                    intake.setPower(1);
                }
                if (gamepad2.left_trigger>0){
                    intake.setPower(-1);
                }
            }

            if (gamepad2.b){
                if (SwitchBucket){
                    SwitchBucket = false;
                    BucketSwitch();
                }
            }

        }


    }
    void BucketSwitch(){
        //This is how we switch the buckets point in which it changes which way it is going when we hold
        //the button B on controller 1.

        if (bucket_case == 0){
            bucket.setPower(-1);
        }
        if (bucket_case == 1){
            //Set speed belows
            bucket.setPower(.8);
        }
        //Power is position in CRservos.
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000 );
        bucket.setPower(.8);
        if (bucket_case == 1){
            bucket_case=0;
        }else{
            bucket_case=1;
        }
        SwitchBucket = true;


    }
}