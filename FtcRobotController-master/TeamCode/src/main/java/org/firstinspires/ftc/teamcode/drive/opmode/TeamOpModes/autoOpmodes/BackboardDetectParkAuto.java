package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.autoOpmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes.SimpleOpenCVColor;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;


//!!Color thresholds need to be tuned to the field!!
@Autonomous(name="BackboardDetectParkAuto")
public class BackboardDetectParkAuto extends LinearOpMode {

//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;

    RobotMecanumDrive drive;

    //todo: Once roadrunner is tuned, make sure to fix/tune the distance values here:
    public static double DISTANCE = 10;
    public static double STRAFEDISTANCE = 90;

    //Create color booleans.
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
// Initialize the hardware variables. Note that the strings used here must correspond
// to the names assigned during the robot configuration step on the DS or RC devices.

        drive = new RobotMecanumDrive(hardwareMap, telemetry);
        SimpleOpenCVColor colorDetector = new SimpleOpenCVColor(hardwareMap, telemetry);
        colorDetector.init();

        Trajectory trajectoryForward1A = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryLeft2B = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(STRAFEDISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryRight1B = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(STRAFEDISTANCE)
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

                    drive.followTrajectory(trajectoryLeft2B);
                    hasRunMovement = true;
                }

                if (Red == true) {
                    drive.followTrajectory(trajectoryForward1A);

                    drive.followTrajectory(trajectoryRight1B);
                    hasRunMovement = true;
                }
            }

        }
    }
}