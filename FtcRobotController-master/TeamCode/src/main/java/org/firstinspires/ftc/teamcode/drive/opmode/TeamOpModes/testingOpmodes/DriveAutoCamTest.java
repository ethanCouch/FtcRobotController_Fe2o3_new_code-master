package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.testingOpmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EasyOpenCv.AutoDirection;
import org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes.OpenCVBlob;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;

/*Test OpMode that will detect an object, find its relative position in relation to the camera, and park.*/
@Autonomous(name="DriveAutoCamTest", group = "test")
public class DriveAutoCamTest extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    RobotMecanumDrive drive;

    //todo: Once roadrunner is tuned, make sure to fix/tune the distance values here:
    public static double DISTANCE = 10;
    public static double STRAFEDISTANCE = 90;

    //Create color booleans.
    AutoDirection Direction = null;

    boolean hasRunMovement = false;

    @Override
    public void runOpMode()
    {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        drive = new RobotMecanumDrive(hardwareMap, telemetry);

        // Initialize the blobDetector
        OpenCVBlob blobDetector = new OpenCVBlob(hardwareMap, telemetry);
        blobDetector.init();

        // Build trajectories
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

        runtime.reset();

        while (opModeIsActive())
        {
            blobDetector.loop();
            AutoDirection getDirection = blobDetector.getDirection();
            Direction = getDirection;

            // If the robot hasn't run movement, then run movement
            if (!hasRunMovement && runtime.seconds() < 20)
            {

                if (Direction == AutoDirection.RIGHT)
                {

                    // Follow trajectories
                    drive.followTrajectory(trajectoryForward1A);
                    drive.followTrajectory(trajectoryLeft2B);

                    // After all of the trajectories have been run then set hasRunMovement to true
                    // and print Direction
                    hasRunMovement = true;
                    telemetry.addData("Direction:", Direction);
                }
                else if (Direction == AutoDirection.MIDDLE)
                {

                    // Follow trajectories
                    drive.followTrajectory(trajectoryForward1A);
                    drive.followTrajectory(trajectoryLeft2B);

                    // After all of the trajectories have been run then set hasRunMovement to true
                    // and print Direction
                    hasRunMovement = true;
                    telemetry.addData("Direction:", Direction);
                }
                else if (Direction == AutoDirection.LEFT)
                {

                    // Follow trajectories
                    drive.followTrajectory(trajectoryForward1A);
                    drive.followTrajectory(trajectoryLeft2B);

                    // After all of the trajectories have been run then set hasRunMovement to true
                    // and print Direction
                    hasRunMovement = true;
                    telemetry.addData("Direction:", Direction);
                }
            }
            else if (!hasRunMovement && runtime.seconds() > 20)
            {

                // Follow trajectories
                drive.followTrajectory(trajectoryForward1A);
                drive.followTrajectory(trajectoryLeft2B);

                // After all of the trajectories have been run then set hasRunMovement to true
                // and print Direction
                hasRunMovement = true;
                telemetry.addData("Direction:", "NONE");
            }

        }
    }
}

