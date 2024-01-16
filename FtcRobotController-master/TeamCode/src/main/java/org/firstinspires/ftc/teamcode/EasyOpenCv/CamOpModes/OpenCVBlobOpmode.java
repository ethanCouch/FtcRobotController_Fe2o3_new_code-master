package org.firstinspires.ftc.teamcode.EasyOpenCv.CamOpModes;

import static org.opencv.features2d.Features2d.drawKeypoints;

import static java.lang.Thread.sleep;

import android.media.ImageReader;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EasyOpenCv.CameraConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;

import java.util.List;
 @TeleOp (name = "OpenCVBlobOpmode")
public class OpenCVBlobOpmode extends OpMode {

     //Resolution
     static final int STREAM_WIDTH = 1280; // modify for your camera
     static final int STREAM_HEIGHT = 720; // modify for your camera
     OpenCvWebcam webcam;
     OpenCVBlobPipeline pipeline;

     String direction = "none";

     @Override
     public void init() {
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         WebcamName webcamName = null;
         webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
         webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
         pipeline = new OpenCVBlobPipeline();
         webcam.setPipeline(pipeline);


         webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
             @Override
             public void onOpened() {
                 webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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
         MatOfKeyPoint matKeypoints = pipeline.getKeypoints();
         KeyPoint[] keypoints = matKeypoints.toArray();
         telemetry.addData("Number of Detects", keypoints.length);
         telemetry.addData("Last Error", pipeline.lastError);
         for (KeyPoint key : keypoints) {
//            telemetry.addData("Position of keypoint:", key.pt.x);
//            telemetry.addData("Position of keypoint:", key.pt.y);
             telemetry.addData("Position of keypoint:", key.toString());

             if (CameraConstants.maxRightY > key.pt.y && key.pt.y > CameraConstants.minRightY && CameraConstants.maxRightX > key.pt.x && key.pt.x > CameraConstants.minRightX)
             {
//                     direction = "Right";
                     telemetry.addData("Direction:", "Right");
             } else if (CameraConstants.maxMiddleY > key.pt.y && key.pt.y > CameraConstants.minMiddleY && CameraConstants.maxMiddleX > key.pt.x && key.pt.x > CameraConstants.minMiddleX) {
//                     direction = "Middle";
                     telemetry.addData("Direction:", "Middle");

             } else if (CameraConstants.maxLeftY > key.pt.y && key.pt.y > CameraConstants.minLeftY && CameraConstants.maxLeftX > key.pt.x && key.pt.x > CameraConstants.minLeftX) {
//                     direction = "left";
                     telemetry.addData("Direction:", "left");
             }
             telemetry.addData("X:", key.pt.x);
             telemetry.addData("Y:", key.pt.y);

         }
     }

     class OpenCVBlobPipeline extends OpenCvPipeline {
         MatOfKeyPoint keypoints = new MatOfKeyPoint();
         String lastError = "";

         Mat tempHSV = new Mat();
         Mat tempThreshold = new Mat();


         public void CreateBlobDetector(SimpleBlobDetector detector) {

             SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();

             params.set_collectContours(true);
//            params.set_filterByColor(true);
             params.set_filterByArea(true);
             params.set_minThreshold(10);
             params.set_maxThreshold(255);
             params.set_minArea(20);
             params.set_maxArea(100000);

             detector = SimpleBlobDetector.create(params);
         }

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

         public SimpleBlobDetector CreateBlobDetector() {

             SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();

             params.set_collectContours(true);
//            params.set_filterByColor(true);
             params.set_filterByArea(true);
             params.set_minThreshold(10);
             params.set_maxThreshold(255);
             params.set_minArea(30);
             params.set_maxArea(100000);

             SimpleBlobDetector detector = SimpleBlobDetector.create(params);

             return detector;
         }

         @Override
         public Mat processFrame(Mat input) {
             try {
                 input = CvtImg2Binary(input);

                 CreateBlobDetector().detect(input, keypoints);
                 sleep(200);

//            Mat output = new Mat();
//            drawKeypoints(input, keypoints, output);

                 System.out.println("processing requested");
             } catch (Exception ex) {
                 lastError = ex.getMessage();
                 System.out.println("error processing");
             }
             return input;
         }
     }
 }