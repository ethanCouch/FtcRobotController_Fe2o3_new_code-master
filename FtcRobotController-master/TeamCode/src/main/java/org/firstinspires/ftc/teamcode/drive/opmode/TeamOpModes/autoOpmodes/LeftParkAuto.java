package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.autoOpmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;

@Autonomous(name="LeftParkAuto")
public class LeftParkAuto extends LinearOpMode {

//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;

    RobotMecanumDrive drive;

    //todo: Once roadrunner is tuned, make sure to fix/tune the distance values here:
    public static double DISTANCE = 10;
    public static double STRAFEDISTANCE = 90;

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

        // Build Trajectories.
        Trajectory trajectoryForward1A = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryLeft2B = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(STRAFEDISTANCE)
                .build();
        waitForStart();

        // If the robot hasn't run movement, then run movement
            if(!hasRunMovement) {
                // Drive forward to get off of the wall
                    drive.followTrajectory(trajectoryForward1A);

                // Drive left to park
                    drive.followTrajectory(trajectoryLeft2B);
                    hasRunMovement = true;

            }
    }
}
