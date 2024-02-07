//package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.TeleOpModes;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.TeleOpModes.Constants;
//import org.firstinspires.ftc.teamcode.utils.PIDController;
//
//public class LinearSlide {
//
//    public final DcMotor motor;
//    private final PIDController controller;
//    private final TouchSensor limit;
//
//    private double encoderOffset;
//    private final int encoderPolarity = -1;
//
//    public LinearSlide(DcMotor motor, TouchSensor limit) {
//        this.motor = motor;
//        this.limit = limit;
//        controller = new PIDController(0.001, 0, 0);
//        setEncoderPosition(0);
//    }
//
//    public void drive(double speed) {
//        if (Double.isNaN(speed)) {
//            speed = 0;
//        }
//        /*double position = getEncoderPosition();
//        if (speed > 0 && position < Constants.Slide.maxExtensionPosition) {
//            motor.setPower(speed);
//        } else if (speed < 0) {
//            if (position < 20) {
//                motor.setPower(-0.3);
//            } else {
//                motor.setPower(speed);
//            }
//        }*/
//        motor.setPower(speed);
//    }
//
//    public void driveToPosition(int targetPosition) {
//        drive(Range.clip(controller.calculate(getEncoderPosition(), targetPosition), -1, 1));
//    }
//
//    public double getEncoderPosition() {
//        if (limit.isPressed()) {
//            setEncoderPosition(0);
//        }
//        return motor.getCurrentPosition() * encoderPolarity + encoderOffset;
//    }
//
//    private void setEncoderPosition(double position) {
//        encoderOffset = position - motor.getCurrentPosition() * encoderPolarity;
//    }
//}
