/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DuckBlueAuto", group="A")
public class DuckBlueAuto extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwareMW robot = new HardwareMW();   // Use a MW hardware
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    private DcMotor duckspin;
    private DcMotor lifter;
    private DcMotor intake;
    private CRServo bucket;
    private ElapsedTime runtime = new ElapsedTime();

    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    //private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .1;
    static final double     TURN_SPEED              = 0.5;

    float lastspeed = 0;
    // Speed is set to strafe or forward motion.
    float lastmode = 0;
    //If lastmode is equal to 0, than it is in forward mode, if it is 1, than it is under strafe mode.
    boolean fr = false;
    //Fr referse to "Forward Right" We can get whether or not it is moving forward or right on the strafe
    //move functions.

    @Override
    public void runOpMode() {


        rightFront =  hardwareMap.dcMotor.get("right_front");
        rightBack =  hardwareMap.dcMotor.get("right_back");
        leftFront =  hardwareMap.dcMotor.get("left_front");
        leftBack =  hardwareMap.dcMotor.get("left_back");
        lifter =  hardwareMap.dcMotor.get("lifter");
        intake =  hardwareMap.dcMotor.get("intake");
        duckspin =  hardwareMap.dcMotor.get("duck_spinner");
        bucket = hardwareMap.crservo.get("bucket");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("leftBackpos", leftBack.getCurrentPosition());
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftBack.getCurrentPosition(),
                rightBack.getCurrentPosition());
        telemetry.update();

        waitForStart();
        //encoderDrive(DRIVE_SPEED, .1f, .1f, -.1f, -.1f, 5.0);
        //9.2 in per unit
        sleep(500);

        float distance = .25f;
        float distancex = 1;
        encoderDrive(1, distance*distancex, -distance*distancex, distance*distancex, -distance*distancex, 1);
        //encoderDrive(1, distance*distancex, -distance*distancex, distance*distancex, -distance*distancex, 1);
        sleep(500);

        duckspin.setPower(-.2);
        sleep(2200*8);
        //sleep(100*8);
        duckspin.setPower(0);
        sleep(1500);

        encoderDrive(1, -distance*.75f, -distance*.75f, -distance*.75f, -distance*.75f, 1);
        float distance2 = .1f;
        encoderDrive(.8f, distance2*distancex, -distance2*distancex, distance2*distancex, -distance2*distancex, 1);



        //This while loops calculates how long to wait until it should move to the next step.


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        sleep(1000);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    void movefront(float distance){
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 1);
    }
    void strafe(float distance, boolean right)
    {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastmode = 1;
        distance = distance/1000;
        lastspeed = distance;
        //Set mode speed distance etc
        if (right == false){
            encoderDrive(DRIVE_SPEED, -1*distance, -1*distance, distance, distance, 5.0);
            fr = true;
        }else{
            encoderDrive(DRIVE_SPEED, distance, distance, -1*distance, -1*distance, 5.0);
            fr = false;
        }
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftbackInches, double rightbackInches, double rightfrontInches, double leftfrontInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftBack.getCurrentPosition() + (int)(leftbackInches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftfrontInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightbackInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightfrontInches * COUNTS_PER_INCH);
            leftBack.setTargetPosition(newLeftBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightBack.setTargetPosition(newRightBackTarget);
            rightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftBack.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBack.isBusy() && rightBack.isBusy())&&leftFront.isBusy()&& rightFront.isBusy()) {

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftBack.getCurrentPosition(),
                        rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
