package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.TeleOpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Field Centric Competition TeleOp", group="Linear Opmode")
//@Disabled
public class FieldCentricCompTeleOp extends LinearOpMode {

    FieldCentricSampleDrive drive;

    // Declare OpMode members.
//    private final ElapsedTime runtime = new ElapsedTime();
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
    private DcMotor slideMotor;

    // boolean climberMode; // for later use when we have a linear slide


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");// To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.

        // Initialize the linear slide
        drive.linearSlide(slideMotor);

        //linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
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

        //linearSlide.setDirection(DcMotor.Direction.FORWARD); //Linear slide, possibly change after test
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

        // Position of the arm when it's lifted
        int slideUpPosition = 1000;

        // Position of the arm when it's down
        int slideDownPosition = 0;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            // If the A button is pressed, raise the arm
            if (gamepad1.right_bumper) {
                slideMotor.setTargetPosition(slideUpPosition);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.left_bumper) {
                slideMotor.setTargetPosition(slideDownPosition);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(0.5);
            }

//            // Get the current position of the armMotor
//            double position = slideMotor.getCurrentPosition();
//
//            // Get the target position of the armMotor
//            double desiredPosition = slideMotor.getTargetPosition();

            double rollerIn = gamepad2.right_trigger;
            double rollerOut = gamepad2.left_trigger;

            double climberPower = gamepad2.right_stick_y;
            //double climberPulleyPower = -gamepad2.left_stick_y / 4;

            double droneShooterPosition = gamepad1.right_trigger;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            double rollerPower = (rollerIn - rollerOut); //change speed multiplier after testing

            // Set motor powers:
            drive.FieldCentricCartesianDrive(leftFrontDrive, rightFrontDrive, leftRearDrive, rightRearDrive);

            roller.setPower(rollerPower);
            climber.setPower(climberPower);
            //  climberPulley.setPower(climberPulleyPower);
            droneShooter.setPosition(droneShooterPosition);

            //e.toString());
            //%.2f)", leftPower, rightPower);
            //
        }
    }
}