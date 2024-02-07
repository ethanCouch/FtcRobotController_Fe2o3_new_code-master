//package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.TeleOpModes;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class MecanumBase {
//
//    public final DcMotor lf;
//    public final DcMotor rf;
//    public final DcMotor lb;
//    public final DcMotor rb;
//
//    public MecanumBase(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
//        lf = leftFront;
//        rf = rightFront;
//        lb = leftBack;
//        rb = rightBack;
//    }
//
//    private void drive(double drive, double strafe, double turn, double botHeading, boolean squareInputs) {
//        if (squareInputs) {
//            drive = Math.copySign(Math.pow(drive, 2), drive);
//            strafe = Math.copySign(Math.pow(strafe, 2), strafe);
//            turn = Math.copySign(Math.pow(turn, 2), turn);
//        }
//        double oldDrive = drive;
//        drive = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
//        strafe = strafe * Math.cos(-botHeading) - oldDrive * Math.sin(-botHeading);
//
//        strafe *= 1.41;
//
//        double[] wheelSpeeds = { // Order: lf, rf, lb, rb
//                drive + strafe - turn,
//                drive - strafe + turn,
//                drive - strafe - turn,
//                drive + strafe + turn
//        };
//
//        double largest = 1.0;
//
//        for (double wheelSpeed : wheelSpeeds) {
//            if (Math.abs(wheelSpeed) > largest) {
//                largest = Math.abs(wheelSpeed);
//            }
//        }
//
//        for (int i = 0; i < wheelSpeeds.length; i++) {
//            wheelSpeeds[i] /= largest;
//        }
//
//        lf.setPower(wheelSpeeds[0]);
//        rf.setPower(wheelSpeeds[1]);
//        lb.setPower(wheelSpeeds[2]);
//        rb.setPower(wheelSpeeds[3]);
//    }
//
//    public void drive(double drive, double strafe, double turn, double botHeading) {
//        drive(drive, strafe, turn, botHeading,false);
//    }
//
//    public void drive(double drive, double strafe, double turn) {
//        drive(drive, strafe, turn, 0);
//    }
//}
