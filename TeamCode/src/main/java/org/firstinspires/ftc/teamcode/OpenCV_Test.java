package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.highgui.Highgui;


/**
 * Created by riseo on 7/1/2018.
 */

@Autonomous(name= "OpenCV_Test", group = "OpenCV")
@Disabled

public abstract class OpenCV_Test extends OpMode{
//    public int n_rows = 100;
//    public int n_cols = 100;
//
//
//
//    public void loop() {
//
//        //Load native library
//        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
////Mat object used to host the image
//        Mat imageArray;
////Read image file from vile system
//        imageArray = Mat.zeros(n_rows, n_cols, CvType.CV_8UC1);


    public void init(String[] args) {
        String default_file = "Internal storage/DCIM/Camera/image.jpg";
        String filename = ((args.length > 0) ? args[0] : default_file);
        // Load an image
        Mat src = Imgcodecs.imread(filename, Imgcodecs.IMREAD_COLOR);
        // Check if image is loaded fine
        if( src.empty() ) {
            telemetry.addLine("Error opening image!");
            telemetry.addData("Program Arguments: [image_name -- default "
                    ,default_file +"] \n");
            telemetry.update();
            System.exit(-1);
        }

        Mat gray = new Mat();
        Imgproc.cvtColor(src, gray, Imgproc.COLOR_BGR2GRAY);

        Imgproc.medianBlur(gray, gray, 5);

        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                25, // change this value to detect circles with different distances to each other
                100.0, 30.0, 1, 0); // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            Point center = new Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
            Imgproc.circle(src, center, 1, new Scalar(0,100,100), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(src, center, radius, new Scalar(255,0,255), 3, 8, 0 );
        }

//        imshow("detected circles", src);
//        HighGui.waitKey();

        Imgcodecs.imwrite("Internal storage/DCIM/Camera/image2.jpg", src);

    }
    public void imageOperations(String[] args){
        String default_file = "Internal storage/DCIM/Camera/image.jpg";
        String filename = ((args.length > 0) ? args[0] : default_file);
        // Load an image
        Mat src = Imgcodecs.imread(filename, Imgcodecs.IMREAD_COLOR);
        // Check if image is loaded fine
        if( src.empty() ) {
            telemetry.addLine("Error opening image!");
            telemetry.addData("Program Arguments: [image_name -- default "
                    ,default_file +"] \n");
            telemetry.update();
            System.exit(-1);
        }

        Mat gray = new Mat();
        Imgproc.cvtColor(src, gray, Imgproc.COLOR_BGR2GRAY);

        Imgproc.medianBlur(gray, gray, 5);

        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                25, // change this value to detect circles with different distances to each other
                100.0, 30.0, 1, 0); // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            Point center = new Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
            Imgproc.circle(src, center, 1, new Scalar(0,100,100), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(src, center, radius, new Scalar(255,0,255), 3, 8, 0 );
        }

//        imshow("detected circles", src);
//        HighGui.waitKey();

        Imgcodecs.imwrite("Internal storage/DCIM/Camera/image2.jpg", src);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        int x = 1;
        telemetry.addLine("Loop");
        telemetry.update();
        while (x > 0){
            
        }
    }

    @Override
    public void stop(){}
}
//public class HoughCircles {
//    public static void main(String[] args) {
//        // Load the native library.
//        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//        new HoughCirclesRun().run(args);
//    }
//
//
//
//}
