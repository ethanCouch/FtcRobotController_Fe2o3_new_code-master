package org.firstinspires.ftc.teamcode.drive.autoOpmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes.SimpleOpenCVColor;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;


@Autonomous(name="SimplePlaceParkAuto")
public class SimplePlaceParkAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
private DcMotor roller = null;       //Motor Port: expan 0

    RobotMecanumDrive drive;
    public static double FORWARDDISTANCE = 2;
    public static double BACKWARDDISTANCE = -1;
    public static double ANGLE1 = -90;
    public static double ANGEL2 = 90;
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

        Trajectory trajectoryForward1A = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARDDISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryBackward1A = drive.trajectoryBuilder(new Pose2d())
                .forward(BACKWARDDISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryForward1B = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARDDISTANCE)
                .build();
        waitForStart();
        Trajectory trajectoryForward2A = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARDDISTANCE)
                .build();
        waitForStart();
        Trajectory trajectoryForward2B = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARDDISTANCE)
                .build();
        waitForStart();

        while(opModeIsActive()) {
            colorDetector.loop();
            boolean getBlue = colorDetector.getBlue();
            boolean getRed = colorDetector.getRed();
            Blue = getBlue;
            Red = getRed;


            if(!hasRunMovement) {
                if (Blue == true) {
                    drive.followTrajectory(trajectoryForward1A);
                    resetRuntime();
                    if (runtime.seconds() < 1){
                        roller.setPower(-1);
                    }
                    drive.followTrajectory(trajectoryBackward1A);
                    drive.turn(Math.toRadians(ANGEL2));

                    drive.followTrajectory(trajectoryForward1B);
                    hasRunMovement = true;
                }
                if (Red == true) {
                    drive.followTrajectory(trajectoryForward2A);

                    if (runtime.seconds() < 1){
                        roller.setPower(-1);
                    }
                    drive.followTrajectory(trajectoryBackward1A);
                    drive.turn(Math.toRadians(ANGLE1));

                    drive.followTrajectory(trajectoryForward2B);
                    hasRunMovement = true;
                }
            }
        }
    }
}