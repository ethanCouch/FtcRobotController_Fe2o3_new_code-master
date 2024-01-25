package org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EasyOpenCv.AutoDirection;

import org.firstinspires.ftc.teamcode.EasyOpenCv.CameraConstants;
import org.firstinspires.ftc.teamcode.drive.RobotMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;

 @TeleOp (name = "OpenCVBlobOpmode")
public class OpenCVBlobOpmode extends OpMode {
     OpenCvWebcam webcam;
     OpenCVBlobPipeline pipeline;
     AutoDirection Direction;
     RobotMecanumDrive drive;
     double MaxArea = 50000;
     double MinArea = 30;

     @Override
     public void init() {
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         WebcamName webcamName;
         webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
         webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
         pipeline = new OpenCVBlobPipeline();
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
     }

     @Override
     public void loop() {
         long Time = System.currentTimeMillis();

         MaxArea = MaxArea + (gamepad1.right_stick_y/10);
         MinArea = MinArea + (gamepad1.left_stick_y/10);

         MatOfKeyPoint matKeypoints = pipeline.getKeypoints();
         KeyPoint[] keyPoints = matKeypoints.toArray();


//         telemetry.addData("Number of Detects", keyPoints.length);
//         telemetry.addData("Last Error", pipeline.lastError);
//
         telemetry.addData("MinArea:",(int)MinArea);
         telemetry.addData("MinArea:",(int)MaxArea);

         for (KeyPoint key : keyPoints) {
             telemetry.addData("Position of keypoint:", key.toString());
             telemetry.addData("Direction:", Direction);
         }
         long Time2 = System.currentTimeMillis();

         telemetry.addData("Elapsed time:", Time2 - Time);

         telemetry.update();
     }
 }
      class OpenCVBlobPipeline extends OpenCvPipeline {
         MatOfKeyPoint keypoints = new MatOfKeyPoint();
         String lastError = "";
          OpenCVBlobOpmode opmode;
         Mat tempHSV = new Mat();
         Mat tempThreshold = new Mat();

         AutoDirection Direction;

         RobotMecanumDrive drive;

         OpenCVBlobPipeline pipeline;

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

                 if (Direction == null)
                 {
                     processFrameGetBlob(input);
                 }

                 System.out.println("processing requested");
             } catch (Exception ex) {
                 lastError = ex.getMessage();
                 System.out.println("error processing");
             }
             return input;
         }
         public void processFrameGetBlob(Mat input)
         {


             //Create Blob detector
             SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();

//             params.set_collectContours(true);
//             params.set_filterByColor(true);
             params.set_filterByArea(true);
             params.set_minThreshold(25);//30
             params.set_maxThreshold(255);
             params.set_minArea((int)opmode.MinArea);//34
             params.set_maxArea((int)opmode.MaxArea);//ideal value between 25000 and 30000|40000

             SimpleBlobDetector detector = SimpleBlobDetector.create(params);

             //Detect blobs
             detector.detect(input, keypoints);

             MatOfKeyPoint matKeypoints = pipeline.getKeypoints();
             KeyPoint[] keyPoints = matKeypoints.toArray();

             for (KeyPoint key : keyPoints) {

                 if (CameraConstants.maxRightY > key.pt.y && key.pt.y > CameraConstants.minRightY && CameraConstants.maxRightX > key.pt.x && key.pt.x > CameraConstants.minRightX)
                 {
                     Direction = Direction.RIGHT;
                 }
                 else if (CameraConstants.maxMiddleY > key.pt.y && key.pt.y > CameraConstants.minMiddleY && CameraConstants.maxMiddleX > key.pt.x && key.pt.x > CameraConstants.minMiddleX)
                 {
                     Direction = Direction.MIDDLE;
                 }
                 else if (CameraConstants.maxLeftY > key.pt.y && key.pt.y > CameraConstants.minLeftY && CameraConstants.maxLeftX > key.pt.x && key.pt.x > CameraConstants.minLeftX)
                 {
                     Direction = Direction.LEFT;
                 }
             }


         }
     }
