package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


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
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

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

        //It gets a little complex here, as we get into to Trigonometry
        double hyp = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //Above finds the Hypotenuse (the long side of a triangle) of the values of the x joystick and y joystick.
        //The Joysticks are set up on an xy plane, meaning vertical gives a y value and horizontal gives an x value
        double rAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        final double lF = hyp * Math.cos(rAngle) + rightX;
        final double rF = hyp * Math.sin(rAngle) - rightX;
        final double lB = hyp * Math.cos(rAngle) + rightX;
        final double rB = hyp * Math.sin(rAngle) - rightX;

        leftFront.setPower(lF);
        rightFront.setPower(rF);
        leftBack.setPower(lB);
        rightBack.setPower(rB);


    }

    //Code that runs just once after the stop button is pressed (also generally don't touch)
    @Override
    public void stop() {
    }

}
