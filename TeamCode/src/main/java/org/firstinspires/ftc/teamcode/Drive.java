package org.firstinspires.ftc.teamcode;
//package is where your class is stored

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;
//import just imports other classes aka other chunks of code that already have commands/methods written so you don't have to write them

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
    private Servo grabber = null;
    private DcMotor linearSlides = null;
    private DcMotor horizontalSlides = null;
    private Servo frontServosLeft = null;
    private Servo frontServosRight = null;
    //private CRServo intakeAdjust = null;
    private CRServo intakeAdjustTwo = null;
  //  private Servo grabber2 = null;
    private DistanceSensor distanceLeft = null;
    private DistanceSensor distanceRight = null;

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
        linearSlides = hardwareMap.get(DcMotor.class, "linearSlides");
        horizontalSlides = hardwareMap.get(DcMotor.class, "horizontalSlides");
        frontServosLeft = hardwareMap.get(Servo.class, "frontServosLeft");
        frontServosRight = hardwareMap.get(Servo.class, "frontServosRight");
        //intakeAdjust = hardwareMap.get(CRServo.class, "intakeAdjust");
        intakeAdjustTwo = hardwareMap.get(CRServo.class, "intakeAdjustTwo");
    //    grabber2 = hardwareMap.get(Servo.class, "grabber2");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");


        //set diretion of motors (if left and right always turn clockwise, then the robot will spin, not good)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        grabber.setDirection(Servo.Direction.FORWARD);
        frontServosLeft.setDirection(Servo.Direction.REVERSE);
        linearSlides.setDirection(DcMotor.Direction.FORWARD);


        linearSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //Code that Loops after the initialize button is pressed (generally don't touch)
    @Override
    public void init_loop() {
    }

    //Code that runs once after the start button is pressed (generally also don't touch this)
    @Override
    public void start() {

        grabber.resetDeviceConfigurationForOpMode();
        //intakeAdjust.resetDeviceConfigurationForOpMode();
        intakeAdjustTwo.resetDeviceConfigurationForOpMode();
      //  grabber2.resetDeviceConfigurationForOpMode();
        frontServosRight.resetDeviceConfigurationForOpMode();
        frontServosLeft.resetDeviceConfigurationForOpMode();

    }


    private void adjustBrakeLeft(){

        while (true){

            leftFront.setPower(-.25);
            leftBack.setPower(.2);
            rightFront.setPower(.25);
            rightBack.setPower(-.2);

            if (distanceLeft.getDistance(DistanceUnit.CM) >= 9.75 && distanceRight.getDistance(DistanceUnit.CM) >= 9.75)  break;
            if (gamepad1.x) break;

        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addLine("In Position");
        telemetry.update();




    }
    private void adjustBrakeRight(){

        while (true){

            leftFront.setPower(.25);
            leftBack.setPower(-.2);
            rightFront.setPower(-.25);
            rightBack.setPower(.2);

            if (distanceLeft.getDistance(DistanceUnit.CM) >= 10 && distanceRight.getDistance(DistanceUnit.CM) >= 10) break; //break when in position
            if (gamepad1.x) break; //emergency break


        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addLine("In Position");
        telemetry.update();



    }

    private void upOne(){

        linearSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlides.setTargetPosition(-500);

        linearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlides.setPower(-.5);

        while(linearSlides.isBusy())
        {
            //wait until target position is reached
        }

        linearSlides.setPower(0);

        linearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Code that Loops (runs continuously)
    @Override
    public void loop() {

        if (gamepad1.right_bumper == true){
            adjustBrakeRight();


        }

        if (gamepad1.left_bumper == true){
            adjustBrakeLeft();

        }

        if (gamepad2.dpad_up == true) {
            upOne();

        }

        leftIntake.setPower(-gamepad2.left_trigger);
        rightIntake.setPower(gamepad2.right_trigger);
        //intakeAdjust.setPower(gamepad2.left_trigger);
        intakeAdjustTwo.setPower(-gamepad2.right_trigger);

        horizontalSlides.setPower(gamepad2.right_stick_y*.5);
        linearSlides.setPower((gamepad2.left_stick_y*.5) - .15);

        if (gamepad2.b == true)
            grabber.setPosition(Servo.MIN_POSITION);
        else
            grabber.setPosition(0.5);

        if (gamepad2.a == true) {
            frontServosLeft.setPosition(Servo.MIN_POSITION);
            frontServosRight.setPosition(Servo.MIN_POSITION);
        }
        else if (gamepad2.y == true) {
            frontServosLeft.setPosition(.20);
            frontServosRight.setPosition(.20);
        }

        if (gamepad2.x == true) {
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
            intakeAdjustTwo.setPower(1);
            //intakeAdjust.setPower(-1);
        }



        if (gamepad1.right_bumper == false && gamepad1.left_bumper == false) {

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


    }

    //Code that runs just once after the stop button is pressed (also generally don't touch)
    @Override
    public void stop() {
    }

}