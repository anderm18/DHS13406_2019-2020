package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


@Autonomous(name= "CVREDSAFE", group="Sky autonomous")
public class CVREDSAFE extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/10f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 4f/12f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private Servo grabber = null;
    private CRServo intakeAdjustTwo = null;

    private DistanceSensor sensorDistance = null;


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.


        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        intakeAdjustTwo = hardwareMap.get(CRServo.class, "intakeAdjustTwo");
        grabber = hardwareMap.get(Servo.class, "grabber");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "nsensor");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        grabber.resetDeviceConfigurationForOpMode();

        waitForStart();
        runtime.reset();

        right(200);
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();

            leftFront.setPower(-.4);
            leftBack.setPower(-.4);
            rightFront.setPower(-.4);
            rightBack.setPower(-.4);

            if (valMid == 0){

                telemetry.addLine("SkyStone Detected!");
                telemetry.update();

                break;
            }

        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(50);

        backwards(150);
        sleep(100);

        turnRight();
        sleep(100);
        intake();
        sleep(200);

        intakeAgain();

        sleep(200);

        leftIntake.setPower(-1);
        rightIntake.setPower(1);
        sleep(500);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        backwards(1925);
        sleep(200);

        turnLeft();

        sleep(300);

        long timeStart = System.nanoTime();

        //rf - lb -
        while (opModeIsActive()){

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));

            telemetry.update();
            leftFront.setPower(.4);
            rightFront.setPower(.4);
            leftBack.setPower(.4);
            rightBack.setPower(.4);

            if (sensorDistance.getDistance(DistanceUnit.CM) <= 5.5){

                break;
            }
        }

        sleep(200);

        forwards(200);

        sleep(200);

        forwards(1000);

        sleep(200);
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
        intakeAdjustTwo.setPower(1);
        grabber.setPosition(Servo.MIN_POSITION);

        sleep(1200);

        leftIntake.setPower(0);
        rightIntake.setPower(0);
        intakeAdjustTwo.setPower(0);

        backwards(1000);


        long timeEnd = System.nanoTime();

        double time = ((timeEnd - timeStart)*.000000001);

        /*if (time >=1.5 && time<= 2.8){
            sleep(200);
            forwards(200);
            sleep(200);
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
            intakeAdjustTwo.setPower(1);
            sleep(1200);
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            intakeAdjustTwo.setPower(0);
            sleep(200);
            backwardsFast(4700);
            sleep(200);
            right(600);
            sleep(200);
            turnLeft();
            sleep(200);
            intakeAgain();
            sleep(200);
            backwards(1250);
            sleep(200);
            turnRight();
            sleep(200);
            forwards(4700);
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
            intakeAdjustTwo.setPower(1);
            sleep(1000);
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            intakeAdjustTwo.setPower(0);
            stop();
        }
/*
        while (opModeIsActive()){
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.update();
            leftFront.setPower(-.5);
            rightFront.setPower(-.5);
            leftBack.setPower(-.5);
            rightBack.setPower(-.5);
            if (sensorDistance.getDistance(DistanceUnit.CM) <= 5){
                break;
            }
        }
        stop();*/
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }



    }



    public void  left(int ticks)
    {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //left is in
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.5);
        leftBack.setPower(.5);
        rightBack.setPower(.6);
        rightFront.setPower(.5);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(.5);
        leftBack.setPower(.5);
        rightBack.setPower(-.5);
        rightFront.setPower(-.5);

        sleep(70);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }

    public void  right(int ticks)
    {
        //left is in
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

    }

    public void forwards(int ticks)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void backwards(int ticks)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(-ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turnRight(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(1550);
        rightFront.setTargetPosition(-1550);
        leftBack.setTargetPosition(1550);
        rightBack.setTargetPosition(-1550);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intake(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(2950);
        rightFront.setTargetPosition(2950);
        leftBack.setTargetPosition(2950);
        rightBack.setTargetPosition(2950);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);
        leftIntake.setPower(-1);
        rightIntake.setPower(1);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turnLeft(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(-1550);
        rightFront.setTargetPosition(1550);
        leftBack.setTargetPosition(-1550);
        rightBack.setTargetPosition(1550);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intakeAgain(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(1200);
        rightFront.setTargetPosition(1200);
        leftBack.setTargetPosition(1200);
        rightBack.setTargetPosition(1200);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);
        leftIntake.setPower(-1);
        rightIntake.setPower(1);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void backwardsFast(int ticks)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(-ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(1);
        leftBack.setPower(1);
        rightBack.setPower(1);
        rightFront.setPower(1);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void intakeShort(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(300);
        rightFront.setTargetPosition(300);
        leftBack.setTargetPosition(300);
        rightBack.setTargetPosition(300);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);
        leftIntake.setPower(-1);
        rightIntake.setPower(1);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void turnRightFar(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(1650);
        rightFront.setTargetPosition(-1650);
        leftBack.setTargetPosition(1650);
        rightBack.setTargetPosition(-1650);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            //do nothing
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}