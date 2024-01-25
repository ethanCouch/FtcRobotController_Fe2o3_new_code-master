package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.testingOpmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EasyOpenCv.AutoDirection;
import org.firstinspires.ftc.teamcode.EasyOpenCv.CameraConstants;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "DriveAutoCamTest", group = "Test")
public class DriveAutoCamTest extends LinearOpMode {
    //todo: Once roadrunner is tuned, make sure to fix/tune the distance values here:
    public static double Distance = 10;
    public static double StrafeDistance = 90;

    OpenCvWebcam webcam;
    OpenCVAutoBlobPipeline pipeline;
    AutoDirection Direction;
    boolean hasRunMovement = false;
    RobotMecanumDrive drive;

    @Override
    public void runOpMode() {
        // Movement
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        drive = new RobotMecanumDrive(hardwareMap, telemetry);


        // Camera
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            WebcamName webcamName;
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            pipeline = new OpenCVAutoBlobPipeline();
            webcam.setPipeline(pipeline);


            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(CameraConstants.STREAM_WIDTH, CameraConstants.STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Camera Failed", "L");
                    telemetry.update();
                }
            });

        waitForStart();

       // Build Trajectories.
        Trajectory trajectoryForward1A = drive.trajectoryBuilder(new Pose2d())
                .forward(Distance)
                .build();
        waitForStart();

        Trajectory trajectoryLeft2B = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(StrafeDistance)
                .build();
        waitForStart();

        if (!hasRunMovement && Direction == AutoDirection.RIGHT) {
            drive.followTrajectory(trajectoryForward1A);

            drive.followTrajectory(trajectoryLeft2B);
            hasRunMovement = true;
            telemetry.addData("Direction:", Direction);

        }

        if (!hasRunMovement && Direction == AutoDirection.MIDDLE) {
            drive.followTrajectory(trajectoryForward1A);

            drive.followTrajectory(trajectoryLeft2B);
            hasRunMovement = true;
            telemetry.addData("Direction:", Direction);
        }

        if (!hasRunMovement && Direction == AutoDirection.LEFT) {
            drive.followTrajectory(trajectoryForward1A);

            drive.followTrajectory(trajectoryLeft2B);
            hasRunMovement = true;
            telemetry.addData("Direction:", Direction);
        }

    }
}

    class OpenCVAutoBlobPipeline extends OpenCvPipeline {
        MatOfKeyPoint keypoints = new MatOfKeyPoint();
        String lastError = "";

        Mat tempHSV = new Mat();
        Mat tempThreshold = new Mat();
        AutoDirection Direction;

        OpenCVAutoBlobPipeline pipeline;

        public Mat CvtImg2Binary(Mat input) {

            //Imgproc.cvtColor(input, tempThreshold, Imgproc.COLOR_RGB2GRAY);
            //Imgproc.threshold(tempThreshold, tempGrayscale, 200, 500, Imgproc.THRESH_BINARY);
            Imgproc.cvtColor(input, tempHSV, Imgproc.COLOR_RGB2HSV);
            Core.inRange(tempHSV, new Scalar(112, 80, 80), new Scalar(184, 255, 255), tempThreshold);
            return tempThreshold;
        }

        public MatOfKeyPoint getKeypoints() {
            return keypoints;
        }

        public String getLastError() {
            return lastError;
        }

        @Override
        public Mat processFrame(Mat input) {
            try {

                input = CvtImg2Binary(input);

                if (Direction == null) {
                    processFrameGetBlob(input);
                }

                System.out.println("processing requested");
            } catch (Exception ex) {
                lastError = ex.getMessage();
                System.out.println("error processing");
            }
            return input;
        }

        public void processFrameGetBlob(Mat input) {


            //Create Blob detector
            SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();

//             params.set_collectContours(true);
//             params.set_filterByColor(true);
            params.set_filterByArea(true);
            params.set_minThreshold(25);//30
            params.set_maxThreshold(255);
            params.set_minArea(32);//34
            params.set_maxArea(50000);//ideal value between 25000 and 30000

            SimpleBlobDetector detector = SimpleBlobDetector.create(params);

            //Detect blobs
            detector.detect(input, keypoints);

            MatOfKeyPoint matKeypoints = pipeline.getKeypoints();
            KeyPoint[] keyPoints = matKeypoints.toArray();

            for (KeyPoint key : keyPoints) {

                if (CameraConstants.maxRightY > key.pt.y && key.pt.y > CameraConstants.minRightY && CameraConstants.maxRightX > key.pt.x && key.pt.x > CameraConstants.minRightX) {
                    Direction = AutoDirection.RIGHT;
                } else if (CameraConstants.maxMiddleY > key.pt.y && key.pt.y > CameraConstants.minMiddleY && CameraConstants.maxMiddleX > key.pt.x && key.pt.x > CameraConstants.minMiddleX) {
                    Direction = AutoDirection.MIDDLE;
                } else if (CameraConstants.maxLeftY > key.pt.y && key.pt.y > CameraConstants.minLeftY && CameraConstants.maxLeftX > key.pt.x && key.pt.x > CameraConstants.minLeftX) {
                    Direction = AutoDirection.LEFT;
                }
            }


        }
    }
