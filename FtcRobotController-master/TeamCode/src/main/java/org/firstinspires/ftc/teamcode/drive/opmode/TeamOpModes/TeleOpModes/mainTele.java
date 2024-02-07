//package org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.TeleOpModes;
//
//import com.qualcomm.hardware.bosch.BHI260IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamOpModes.TeleOpModes.MecanumBase;
//import org.firstinspires.ftc.teamcode.util.GamepadUtils;
//
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Iterative OpMode")
//
//public class mainTele extends OpMode {
//
//    private DcMotor intakeMotor = null;
//    private Servo placerServo;
//    private GamepadUtils gamepadUtils;
//
//    private LinearSlide slide;
//    private MecanumBase base;
//
//    Servo droneShooter;
//    double droneShooterPosition;
//
//    private IMU imu;
//    private double botHeading;
//
//    @Override
//    public void init() {
//        gamepadUtils = new GamepadUtils(gamepad1, gamepad2);
//        telemetry.addData("Status", "Initialized");
//
//        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor"); //EH
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        placerServo = hardwareMap.get(Servo.class, "placer"); //EH
//
//        slide = new LinearSlide(hardwareMap.get(DcMotor.class, "slideMotor"), hardwareMap.get(TouchSensor.class, "limit"));
//        slide.motor.setDirection(DcMotor.Direction.REVERSE);
//
//        droneShooter = hardwareMap.get(Servo.class, "shooter");
//
//        base = new MecanumBase(
//                hardwareMap.get(DcMotor.class, "lf"),  // CH Port: 0
//                hardwareMap.get(DcMotor.class, "rf"),  // CH Port: 1
//                hardwareMap.get(DcMotor.class, "lb"),  // CH Port: 2
//                hardwareMap.get(DcMotor.class, "rb")); // CH Port: 3
//        base.lf.setDirection(DcMotorSimple.Direction.REVERSE);
//        base.lb.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        imu = hardwareMap.get(BHI260IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
//        imu.resetYaw();
//
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void init_loop() {
//    }
//
//    @Override
//    public void start() {
//    }
//
//    @Override
//    public void loop() {
//        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        telemetry.addData("bot yaw", botHeading);
//        telemetry.addData("slide pose", slide.getEncoderPosition());
//
//        if (gamepad1.options) {
//            imu.resetYaw();
//        }
//
//        intakeMotor.setPower(gamepadUtils.getTriggerSpeed());
//        slide.drive(gamepad2.right_stick_y / Math.abs (gamepad2.right_stick_y));
//        if (gamepad2.a) {
//            placerServo.setPosition(0.7);
//        } else if (gamepad2.b) {
//            placerServo.setPosition(0.3);
//        }
//        base.drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, botHeading);
//
//        droneShooter.setPosition(gamepad1.right_trigger);
//
//    }
//
//    @Override
//    public void stop() {
//    }
//
//}
