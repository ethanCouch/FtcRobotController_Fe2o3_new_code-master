package org.firstinspires.ftc.teamcode.EasyOpenCv;

// todo: fully implement crop variables

import org.openftc.easyopencv.OpenCvWebcam;

public class CameraConstants {


//  Resolution
    public static final int STREAM_WIDTH = 640; // modify for your camera
    public static final int STREAM_HEIGHT = 360; // modify for your camera


//  Cropping Variables
    public static int verticalCrop = 112;
    public static int HorizontalCrop = 130;

// Blob positions:


    // right:

    // Y
    public static int minRightY = 0;
    public static int maxRightY = 120/*STREAM_HEIGHT*/;

    // X
    public static int minRightX = 427;
    public static int maxRightX = STREAM_WIDTH;


    // middle:

    // Y
    public static int minMiddleY = 0;
    public static int maxMiddleY = 120/*STREAM_HEIGHT*/;

    // X
    public static int minMiddleX = 213;
    public static int maxMiddleX = 427;


    // left:

    // Y
    public static int minLeftY = 0;
    public static int maxLeftY = 120/*STREAM_HEIGHT*/;

    // X
    public static int minLeftX = 0;
    public static int maxLeftX = 213;


}
