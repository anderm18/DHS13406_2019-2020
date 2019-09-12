package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive", group="1")
public class Drive extends OpMode
{
    // Define Motors/Servos
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    //Once the START Button is pressed
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");



        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

    }

    //Code that Loops after the initialize button is pressed (generally don't touch)
    @Override
    public void init_loop() {
    }

    //Code that runs once after the start button is pressed (generally also don't touch this)
    @Override
    public void start() {
    }

    //Code that Loops (runs continuously)
    @Override
    public void loop() {


        double leftfrontPower;
        double rightfrontPower;

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftfrontPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightfrontPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Send calculated power to wheels
        leftFront.setPower(leftfrontPower);
        rightFront.setPower(rightfrontPower);

    }

    //Code that runs just once after the stop button is pressed (also generally don't touch)
    @Override
    public void stop() {
    }

}
