package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.testingOpmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EasyOpenCv.AutoDirection;
import org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes.OpenCVBlob;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;


@Autonomous(name="DriveAutoCamTest", group = "test")
public class DriveAutoCamTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotMecanumDrive drive;

    //todo: Once roadrunner is tuned, make sure to fix/tune the distance values here:
    public static double DISTANCE = 10;
    public static double MIDDLEDISTANCE = 20;
    public static double STRAFEDISTANCE = 90;
    public static double PARKDISTANCE = 90;

    //Create color booleans.
    AutoDirection Direction = null;
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
        OpenCVBlob blobDetector = new OpenCVBlob(hardwareMap, telemetry);
        blobDetector.init();

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

        while(opModeIsActive()) {
            blobDetector.loop();
            AutoDirection getDirection = blobDetector.getDirection();
            Direction = getDirection;

            if (!hasRunMovement) {

                if (Direction == AutoDirection.RIGHT) {
                    drive.followTrajectory(trajectoryForward1A);

                    drive.followTrajectory(trajectoryLeft2B);
                    hasRunMovement = true;
                    telemetry.addData("Direction:",Direction);
                }
              else if(Direction==AutoDirection.MIDDLE){
                    drive.followTrajectory(trajectoryForward1A);

                    drive.followTrajectory(trajectoryLeft2B);
                    hasRunMovement=true;
                    telemetry.addData("Direction:",Direction);
                }

             else if(Direction==AutoDirection.LEFT){
                    drive.followTrajectory(trajectoryForward1A);

                    drive.followTrajectory(trajectoryLeft2B);
                    hasRunMovement = true;
                    telemetry.addData("Direction:",Direction);
                }
            }
            else if (runtime.seconds() > 10 && Direction == AutoDirection.NONE)
            {
                drive.followTrajectory(trajectoryForward1A);

                drive.followTrajectory(trajectoryRight1B);
                hasRunMovement=true;
                telemetry.addData("Direction:",Direction);
            }

        }
    }
}

