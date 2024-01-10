
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

/*
    game plan!
    Need encoders for linear slide + code for them
    Need to test all systems once robot is (theoretically) functioning
*/
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Game Opmode", group="Linear Opmode")
//@Disabled
public class GameTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //drive motors
    private DcMotor leftFrontDrive = null; // Motor Port: cntrl 0
    private DcMotor rightFrontDrive = null;// Motor Port: cntrl 1
    private DcMotor leftRearDrive = null;  // Motor Port: cntrl 2
    private DcMotor rightRearDrive = null; // Motor Port: cntrl 3


    //    private DcMotor linearSlide = null;   //Motor Port: expan 3
    private DcMotor roller = null;       //Motor Port: expan 0
    private DcMotor climber = null;      //Motor Port: expan 1
    private DcMotor climberPulley = null;//Motor Port: expan 2
    private Servo droneShooter = null;   //Servo Port: cntrl 0

    boolean climberMode; // for later use when we have a linear slide


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");// To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.

//        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        climber = hardwareMap.get(DcMotor.class, "climber_motor");
        climberPulley = hardwareMap.get(DcMotor.class, "pulley_motor");
        roller = hardwareMap.get(DcMotor.class, "roller_motor");
        droneShooter = hardwareMap.get(Servo.class, "drone_shooter");

        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

//        linearSlide.setDirection(DcMotor.Direction.FORWARD); //Linear slide, possibly change after test
        roller.setDirection(DcMotor.Direction.FORWARD); //Linear slide, possibly change after test
        climber.setDirection(DcMotor.Direction.FORWARD); //Linear slide, possibly change after test
        climberPulley.setDirection(DcMotor.Direction.FORWARD); //Linear slide, possibly change after test
        droneShooter.setDirection(Servo.Direction.FORWARD); //Linear slide, possibly change after test

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double rollerIn = gamepad2.right_trigger;
            double rollerOut = gamepad2.left_trigger;

            double climberPower = gamepad2.right_stick_y / 4;
            double climberPulleyPower = -gamepad2.left_stick_y;

            double droneShooterPosition = gamepad1.right_trigger;

            double frontLeftPower = Range.clip(y + x + rx, -0.5,0.5);
            double backLeftPower = Range.clip(y - x + rx,-0.5,0.5);
            double frontRightPower = Range.clip(y - x - rx,-0.5,0.5);
            double backRightPower = Range.clip(y + x - rx,-0.5,0.5);

            double rollerPower = (rollerIn - rollerOut); //change speed multiplier after testing

//          Set motor powers:
            leftFrontDrive.setPower(frontLeftPower); //
            leftRearDrive.setPower(backLeftPower); //.
            rightFrontDrive.setPower(frontRightPower); //drive forward slowly and keep straight.
            rightRearDrive.setPower(backRightPower); //

//            if (gamepad2.right_trigger > 0.1) {
//                climberPower = 1;
//            } else if (gamepad2.left_trigger > 0.1) {
//                climberPower = -1;
//            } else {
//                climberPower = 0;
//            }

            roller.setPower(rollerPower);
            climber.setPower(climberPower);
            climberPulley.setPower(climberPulleyPower);
            droneShooter.setPosition(droneShooterPosition);

            //e.toString());
            //%.2f)", leftPower, rightPower);
            //
        }
    }
}
