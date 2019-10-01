package org.firstinspires.ftc.teamcode;
//package is where your class is stored

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import just imports other classes aka other chunks of code that already have commands/methods written so you dont have to write them

@TeleOp(name="Drive", group="1")
public class Drive extends OpMode
{
    // Define Motors/Servos
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;
    private CRServo grabber = null;


    //Once the START Button is pressed
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //Basically just says Initialize on the phone

        //define what hardware is being used on your robot and in your code (break the barrier between hardware and software
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        //set diretion of motors (if left and right always turn clockwise, then the robot will spin, not good)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        grabber.setDirection(CRServo.Direction.REVERSE);


    }

    //Code that Loops after the initialize button is pressed (generally don't touch)
    @Override
    public void init_loop() {
    }

    //Code that runs once after the start button is pressed (generally also don't touch this)
    @Override
    public void start() {

        grabber.resetDeviceConfigurationForOpMode();
    }

    //Code that Loops (runs continuously)
    @Override
    public void loop() {


        if (gamepad2.x == true){

            grabber.setPower(1);
        }

        if (gamepad2.b == true){

            grabber.setPower(-1);
        }
        else{

            grabber.setPower(0);
        }

        leftIntake.setPower(gamepad2.left_trigger);
        rightIntake.setPower(gamepad2.right_trigger);


        //It gets a little complex here, as we get into to Trigonometry
        double hyp = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //Above finds the Hypotenuse (the long side of a triangle) of the values of the x joystick and y joystick.
        //The Joysticks are set up on an xy plane, meaning vertical gives a y value and horizontal gives an x value
        double rAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        //Above is a trig idenitity. Basically, all you need to know is that it's using a mathematical law to solve the angle (theta) the robot is at with the x axis
        double turn = -gamepad1.right_stick_x;
        //Above is basically just turning, when you move x left the robot turns left, when right it turns right. No Mecanum drive in there.

        final double lF = hyp * Math.cos(rAngle) + turn;
        //Cosine (cos) is taking the angle the robot is facing (defined above), and finding the hypotanuse and adding turn (how far the x on the joystick is moved aka the left right thing)
        final double rF = hyp * Math.sin(rAngle) - turn;
        //same as cos except using the sine (sin) function. It's doing the same thing, but different math due to different angle
        final double lB = hyp * Math.sin(rAngle) + turn;
        //same as cos
        final double rB = hyp * Math.cos(rAngle) - turn;
        //same as sin


        //Below simply just sets all the calculated values to motor power aka movement
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
