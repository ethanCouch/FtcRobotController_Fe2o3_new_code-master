package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.autoOpmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes.SimpleOpenCVColor;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;


@Disabled
@Autonomous(name="TimedPlaceParkAuto")
public class TimedPlaceParkAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
    private DcMotor roller = null;       //Motor Port: expan 0

    RobotMecanumDrive drive;
    public static double FORWARD = 0.8;
    public static double BACKWARD = -0.5;
    public static double TURNSPEED = 0.8;
    Boolean Blue = null;
    Boolean Red = null;
    /*
        private Encoder left = null;

        private Encoder right = null;

        private Encoder back = null;
        */

    boolean hasRunMovement = false;
    @Override
    public void runOpMode() {
        drive = new RobotMecanumDrive(hardwareMap, telemetry);
        SimpleOpenCVColor colorDetector = new SimpleOpenCVColor(hardwareMap, telemetry);
        colorDetector.init();

        roller = hardwareMap.get(DcMotor.class, "roller_motor");
        roller.setDirection(DcMotor.Direction.FORWARD); //Linear slide, possibly change after test
        roller.setPower(0);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
//
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        double leftFrontPower = 1;
//        double rightFrontPower = 1;
//        double leftBackPower = 1;
//        double rightBackPower = 1;
//
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);

        while(opModeIsActive()) {
            colorDetector.loop();
            boolean getBlue = colorDetector.getBlue();
            boolean getRed = colorDetector.getRed();
            Blue = getBlue;
            Red = getRed;

            boolean forwardDrive = false;
            boolean BackwardDrive = false;
            boolean rollerDrive = false;
            boolean turnDrive = false;


            if(!hasRunMovement)
            {
                if (Blue == true) {

                    // The amount of if statements gives me pain,
                    //although we dont have to use this program if I get trajectories working

                    if (runtime.seconds() < 1){
                        drive.drive(false, FORWARD,0,0);
                        forwardDrive = true;
                    }
                    resetRuntime();

                    if (runtime.seconds() < 1 && forwardDrive == true){
                        roller.setPower(-1);
                        rollerDrive = true;
                    }
                    resetRuntime();

                    if (runtime.seconds() < 1 && rollerDrive == true){
                        drive.drive(false, BACKWARD,0,0);
                        BackwardDrive = true;
                    }

                    resetRuntime();
                    if (runtime.seconds() < 1 && BackwardDrive == true){
                        drive.drive(false, 0,0,TURNSPEED);
                        turnDrive = true;
                    }

                    resetRuntime();
                    if (runtime.seconds() < 1 && turnDrive == true){
                        drive.drive(false, FORWARD,0,0);
                    }

                    hasRunMovement = true;
                }


                if (Red == true) {
                    if (runtime.seconds() < 1){
                        drive.drive(false, FORWARD,0,0);
                        forwardDrive = true;
                    }
                    resetRuntime();

                    if (runtime.seconds() < 1 && forwardDrive == true){
                        roller.setPower(-1);
                        rollerDrive = true;
                    }
                    resetRuntime();

                    if (runtime.seconds() < 1 && rollerDrive == true){
                        drive.drive(false, BACKWARD,0,0);
                        BackwardDrive = true;
                    }

                    resetRuntime();
                    if (runtime.seconds() < 1 && BackwardDrive == true){
                        drive.drive(false, 0,0,-TURNSPEED);
                        turnDrive = true;
                    }

                    resetRuntime();
                    if (runtime.seconds() < 1 && turnDrive == true){
                        drive.drive(false, FORWARD,0,0);
                    }

                    hasRunMovement = true;
                }
            }
        }
    }
}