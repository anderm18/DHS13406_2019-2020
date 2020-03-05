package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


@Autonomous(name= "AutoPlayground(TESTCLASS)", group="Sky autonomous")
public class AutoPlaygroundNEVER_RUN extends LinearOpMode {
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

    private final int rows = 1280;
    private final int cols = 720;

    //OpenCvCamera C199E06F;

    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private DcMotor linearSlides = null;

    private Servo grabber = null;
    private CRServo intakeAdjustTwo = null;

    private DistanceSensor sensorDistance = null;

    private BNO055IMU imu = null;

    private DistanceSensor distanceLeft = null;
    private DistanceSensor distanceRight = null;

    private Servo frontServosLeft = null;
    private Servo frontServosRight = null;

    private CRServo tape = null;

    Orientation angles;
    Acceleration gravity;
    private float curHeading;

    OpenCvWebcam C199E06F;


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        C199E06F = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "C199E06F"), cameraMonitorViewId);//remove this

        C199E06F.openCameraDevice();//open camera
        C199E06F.setPipeline(new StageSwitchingPipeline());//different stages
        C199E06F.startStreaming(rows, cols, OpenCvCameraRotation.UPSIDE_DOWN);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        intakeAdjustTwo = hardwareMap.get(CRServo.class, "intakeAdjustTwo");
        grabber = hardwareMap.get(Servo.class, "grabber");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "nsensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        linearSlides = hardwareMap.get(DcMotor.class, "linearSlides");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");
        frontServosLeft = hardwareMap.get(Servo.class, "frontServosLeft");
        frontServosRight = hardwareMap.get(Servo.class, "frontServosRight");
        tape = hardwareMap.get(CRServo.class, "tape");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        linearSlides.setDirection(DcMotor.Direction.FORWARD);
        frontServosLeft.setDirection(Servo.Direction.REVERSE);

        grabber.resetDeviceConfigurationForOpMode();

        frontServosRight.resetDeviceConfigurationForOpMode();
        frontServosLeft.resetDeviceConfigurationForOpMode();

        tape.resetDeviceConfigurationForOpMode();

        telemetry.addLine("opmode active");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addLine("opmode active");
        telemetry.update();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 0);

        //heres some imu test stuff

        //right -
        //left +


        frontServosLeft.setPosition(.1);
        frontServosRight.setPosition(Servo.MIN_POSITION);

        right(400, 0);

        long timeStart = System.nanoTime();

        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();

            leftFront.setPower(-.7);
            leftBack.setPower(-.7);
            rightFront.setPower(-.7);
            rightBack.setPower(-.7);

            if (valMid == 0) {

                telemetry.addLine("SkyStone Detected!");
                telemetry.update();

                break;
            }

            checkOrientation();
            if (curHeading > 0) {

                checkOrientation();
                leftFront.setPower(-.75);
                leftBack.setPower(-.75);
                rightBack.setPower(-.7);
                rightFront.setPower(-.7);

            } else if (curHeading < 0) {

                checkOrientation();
                leftFront.setPower(-.75);
                leftBack.setPower(-.75);
                rightBack.setPower(-.7);
                rightFront.setPower(-.7);
            }
            else {

                checkOrientation();
                leftFront.setPower(-.7);
                leftBack.setPower(-.7);
                rightFront.setPower(-.7);
                rightBack.setPower(-.7);
            }

        }

        forwards(700, 0);

        long timeEnd = System.nanoTime();


        double time = ((timeEnd - timeStart)*.000000001);

        turnRight();

        intake(-90);

        intakeAgain(-90);

        backwards(1900, -90);

        while(opModeIsActive()){

            checkOrientation();
            leftFront.setPower(.75);
            leftBack.setPower(.75);
            rightFront.setPower(-.75);
            rightBack.setPower(-.75);
            checkOrientation();
            if(curHeading < -83 && curHeading > -96){
                break;
            }
        }

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //left is in
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(9000);
        rightFront.setTargetPosition(9000);
        leftBack.setTargetPosition(9000);
        rightBack.setTargetPosition(9000);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){

            checkOrientation();


            if (curHeading > -90){

                checkOrientation();
                leftFront.setPower(.75);
                leftBack.setPower(1);
                rightBack.setPower(1);
                rightFront.setPower(.75);

            }
            else if (curHeading < -90){

                checkOrientation();
                leftFront.setPower(1);
                leftBack.setPower(.75);
                rightBack.setPower(.75);
                rightFront.setPower(1);

            }
            else{

                checkOrientation();
                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightBack.setPower(.75);
                rightFront.setPower(.75);

            }
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

        while (opModeIsActive()){

            checkOrientation();
            if (curHeading < -80 && curHeading > -90){
                break;
            }
            leftFront.setPower(.7);
            leftBack.setPower(.7);
            rightBack.setPower(-.7);
            rightFront.setPower(-.7);
            checkOrientation();
        }

        forwards(1800, -90);

        leftIntake.setPower(1);
        rightIntake.setPower(-1);
        intakeAdjustTwo.setPower(1);
        grabber.setPosition(Servo.MAX_POSITION);

        sleep(1200);

        leftIntake.setPower(0);
        rightIntake.setPower(0);
        intakeAdjustTwo.setPower(0);

        frontServosLeft.setPosition(.15);
        frontServosRight.setPosition(.20);

        backwards(4000, -90);

        frontServosLeft.setPosition(Servo.MIN_POSITION);
        frontServosRight.setPosition(Servo.MIN_POSITION);

        forwards(500, -90);
        backwards(200, -90);

        while(opModeIsActive()){

            checkOrientation();
            leftFront.setPower(.75);
            leftBack.setPower(.75);
            rightFront.setPower(-.75);
            rightBack.setPower(-.75);
            checkOrientation();
            if(curHeading < -170 && curHeading > -190){
                break;
            }
        }

        tape.setPower(1);



        /*leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(-1400);
        rightFront.setTargetPosition(-1400);
        leftBack.setTargetPosition(-1400);
        rightBack.setTargetPosition(-1400);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(1);
        rightFront.setPower(1);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){
            checkOrientation();

            if (curHeading > -90) {

                leftFront.setPower(-.75);
                leftBack.setPower(-.75);
                rightBack.setPower(-1);
                rightFront.setPower(-1);
                checkOrientation();
            }
            else if (curHeading < -90){

                leftFront.setPower(-1);
                leftBack.setPower(-1);
                rightFront.setPower(-.75);
                rightBack.setPower(-.75);
                checkOrientation();
            }
            else{

                leftFront.setPower(-.75);
                leftBack.setPower(-.75);
                rightBack.setPower(-.75);
                rightFront.setPower(-.75);
            }
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()){

            checkOrientation();
            if (curHeading < -80 && curHeading > -90){
                break;
            }
            leftFront.setPower(.7);
            leftBack.setPower(.7);
            rightBack.setPower(-.7);
            rightFront.setPower(-.7);
            checkOrientation();
        }

        right(10200, -90);

        intake(-90);

        backwards(1900, -90);

        left(10200);

        while (opModeIsActive()){

            checkOrientation();
            if (curHeading < -80 && curHeading > -90){
                break;
            }
            leftFront.setPower(.7);
            leftBack.setPower(.7);
            rightBack.setPower(-.7);
            rightFront.setPower(-.7);
            checkOrientation();
        }





        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();

            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }




        //backwards(100, 0);

       /*turnLeft();

       slidesUp(-1500);

       linearSlides.setPower(-.0015);

       while(opModeIsActive()){
           telemetry.addData("Distance (cm)",
                   String.format(Locale.US, "%.02f", distanceLeft.getDistance(DistanceUnit.CM)));
           telemetry.addData("Distance (cm)",
                   String.format(Locale.US, "%.02f", distanceRight.getDistance(DistanceUnit.CM)));
           telemetry.update();

           leftFront.setPower(-.75);
           leftBack.setPower(-.75);
           rightFront.setPower(-.75);
           rightBack.setPower(-.75);
           if(distanceRight.getDistance(DistanceUnit.CM) < 15 && distanceLeft.getDistance(DistanceUnit.CM) < 15){
               break;
           }
       }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);*/
    }

    private void checkOrientation() {

        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();

        curHeading = angles.firstAngle;
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


    static class StageSwitchingPipeline2 extends OpenCvPipeline
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

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_BGR2GRAY);//converts rgb to ycrcb
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


    public void slidesUp(int ticks){

        linearSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlides.setTargetPosition(ticks);

        linearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlides.setPower(-.35);

        while (linearSlides.isBusy()){}

        linearSlides.setPower(-.0015);

        linearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        leftFront.setPower(.75);
        leftBack.setPower(.75);
        rightBack.setPower(.75);
        rightFront.setPower(.75);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()){

            checkOrientation();


            if (curHeading > -90){

                checkOrientation();
                leftFront.setPower(.75);
                leftBack.setPower(1);
                rightBack.setPower(1);
                rightFront.setPower(.75);

            }
            else if (curHeading < -90){

                checkOrientation();
                leftFront.setPower(1);
                leftBack.setPower(.75);
                rightBack.setPower(.75);
                rightFront.setPower(1);

            }
            else{

                checkOrientation();
                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightBack.setPower(.75);
                rightFront.setPower(.75);

            }
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

    public void right(int ticks, double k)
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


            checkOrientation();
            if(curHeading > k) {


                leftFront.setPower(.9);
                leftBack.setPower(.9);
                rightBack.setPower(.75);
                rightFront.setPower(.75);
                checkOrientation();
            }
            else if (curHeading < k){


                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightFront.setPower(.9);
                rightBack.setPower(.9);
                checkOrientation();
            }
            else{


                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightBack.setPower(.75);
                rightFront.setPower(.75);
                checkOrientation();
            }

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

    public void forwards(int ticks, double k)
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

            checkOrientation();

            if (curHeading > k) {

                leftFront.setPower(.8);
                leftBack.setPower(.8);
                rightBack.setPower(.75);
                rightFront.setPower(.75);
                checkOrientation();
            }
            else if (curHeading < k){

                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightFront.setPower(.8);
                rightBack.setPower(.8);
                checkOrientation();
            }
            else{

                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightBack.setPower(.75);
                rightFront.setPower(.75);
            }
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

    public void backwards(int ticks, double k)
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
            checkOrientation();

            if (curHeading > k) {

                leftFront.setPower(-.75);
                leftBack.setPower(-.75);
                rightBack.setPower(-.9);
                rightFront.setPower(-.9);
                checkOrientation();
            }
            else if (curHeading < k){

                leftFront.setPower(-.9);
                leftBack.setPower(-.9);
                rightFront.setPower(-.75);
                rightBack.setPower(-.75);
                checkOrientation();
            }
            else{

                leftFront.setPower(-.75);
                leftBack.setPower(-.75);
                rightBack.setPower(-.75);
                rightFront.setPower(-.75);
            }
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

    public void intake(double k){

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
            checkOrientation();

            if (curHeading > k) {

                leftFront.setPower(.8);
                leftBack.setPower(.8);
                rightBack.setPower(.75);
                rightFront.setPower(.75);
                checkOrientation();
            }
            else if (curHeading < k){

                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightFront.setPower(.8);
                rightBack.setPower(.8);
                checkOrientation();
            }
            else{

                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightBack.setPower(.75);
                rightFront.setPower(.75);
            }
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

    public void intakeAgain(double k){

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

            checkOrientation();

            if (curHeading > k) {

                leftFront.setPower(.8);
                leftBack.setPower(.8);
                rightBack.setPower(.75);
                rightFront.setPower(.75);
                checkOrientation();
            }
            else if (curHeading < k){

                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightFront.setPower(.8);
                rightBack.setPower(.8);
                checkOrientation();
            }
            else{

                leftFront.setPower(.75);
                leftBack.setPower(.75);
                rightBack.setPower(.75);
                rightFront.setPower(.75);
            }
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