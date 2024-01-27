package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.testingOpmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EasyOpenCv.AutoDirection;
import org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes.OpenCVBlob;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;

@Autonomous(name="Drive Auto Detect Blob Place")
public class DriveAutoDetectBlobPlace extends LinearOpMode
{
   // Create an instance of ElapsedTime
    private ElapsedTime runtime = new ElapsedTime();

   // Create an instance of RobotMecanumDrive
    RobotMecanumDrive drive;

   // Initialize distance variables
   //todo: Once roadrunner is tuned, make sure to fix/tune the distance values here:
   //todo: test and tune theses when u can ;)
    public static double BACKDISTANCE = -5;
    public static double FORWARDDISTANCE = 20;
    public static double MIDDLEDISTANCE = 25;
    public static double STRAFEDISTANCE = 15;

   // Create an instance of the Directions in AutoDirection
    AutoDirection Direction = null;

   // Initialize hasRunMovement as false
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
        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARDDISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(new Pose2d())
                .forward(BACKDISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryStrafeLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(STRAFEDISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryStrafeRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(STRAFEDISTANCE)
                .build();
        waitForStart();

        Trajectory trajectoryForwardMiddle = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(MIDDLEDISTANCE)
                .build();
        waitForStart();


      // Reset runtime before opmode starts
        runtime.reset();

        while(opModeIsActive())
        {
            blobDetector.loop();
            AutoDirection getDirection = blobDetector.getDirection();
            Direction = getDirection;


           // If the robot hasn't run movement, then check
            if (!hasRunMovement && runtime.seconds() < 20)
            {
               // todo: This probably should be a switch case statement... but im sure it'll be "fine"
                if (Direction == AutoDirection.RIGHT)
                {

                   // Follow trajectories
                    drive.followTrajectory(trajectoryForward);
                    drive.followTrajectory(trajectoryStrafeRight);
                    drive.followTrajectory(trajectoryBackward);

                   // After all of the trajectories have been run then set hasRunMovement to true
                   // and print Direction
                    hasRunMovement = true;
                    telemetry.addData("Direction:",Direction);
                }
                else if(Direction==AutoDirection.MIDDLE)
                {

                   // Follow trajectories
                    drive.followTrajectory(trajectoryForwardMiddle);
                    drive.followTrajectory(trajectoryBackward);

                   // After all of the trajectories have been run then set hasRunMovement to true
                   // and print Direction
                    hasRunMovement = true;
                    telemetry.addData("Direction:",Direction);
                }

                else if(Direction==AutoDirection.LEFT)
                {

                   // Follow trajectories
                    drive.followTrajectory(trajectoryForward);
                    drive.followTrajectory(trajectoryStrafeLeft);
                    drive.followTrajectory(trajectoryBackward);

                   // After all of the trajectories have been run then set hasRunMovement to true
                   // and print Direction
                    hasRunMovement = true;
                    telemetry.addData("Direction:",Direction);
                }
            }

            else if (!hasRunMovement && runtime.seconds() > 20)
            {

               // Follow trajectories
                drive.followTrajectory(trajectoryForwardMiddle);
                drive.followTrajectory(trajectoryBackward);

               // After all of the trajectories have been run then set hasRunMovement to true
               // and print Direction
                hasRunMovement = true;
                telemetry.addData("Direction:","NONE");
            }

        }
    }
}
