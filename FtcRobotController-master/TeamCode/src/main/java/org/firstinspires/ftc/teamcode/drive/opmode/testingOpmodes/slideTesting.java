package org.firstinspires.ftc.teamcode.drive.opmode.testingOpmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="slideTester", group="linearOpmode")

public class slideTesting extends LinearOpMode {
    private DcMotor slide = null; //slide motor

    public void runOpMode() throws InterruptedException {

        slide = hardwareMap.get(DcMotor.class, "linear_slide" );

        int desiredPosition = 1000;
        slide.setTargetPosition(desiredPosition);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);

        waitForStart();

        while (opModeIsActive()) {
            double TPR = 537.6; // Ticks per rev //need to replace with accurate number once linear slide motor exists

            //possible to add some funky math but for now the encoder will work basically by just counting tpr
            int position = slide.getCurrentPosition();
            double revolutions = position/TPR;

            double angle = revolutions * 360;
            double angleNormalized = angle % 360;

            // show telemetry data:
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Encoder Revolutions", revolutions);
            telemetry.addData("Encoder Angle (Degrees)", angle);
            telemetry.addData("Encoder Angle -Normalized (Degrees)", angleNormalized);
            telemetry.update();

        }
    }

}
