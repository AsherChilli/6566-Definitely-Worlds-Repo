package org.firstinspires.ftc.teamcode.OpenCV.Processors;

import android.graphics.Canvas;
import android.graphics.Paint;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
@Config
public class sampleProcessor implements VisionProcessor {


    public static Scalar lowBlue = new Scalar(0, 80, 60);
    public static Scalar highBlue = new Scalar(20, 255, 255);
    public static Scalar lowRed1 = new Scalar(20, 40, 100);
    public static Scalar highRed1 = new Scalar(110, 130, 100);
    public static Scalar lowRed2 = new Scalar(140, 255, 255);
    public static Scalar highRed2 = new Scalar(30, 255, 255);
    public static Scalar lowerYellow = new Scalar(90, 60, 60);
    public static Scalar upperYellow = new Scalar(105, 255, 255);



    private int width;
    private int height;
    private Position position;
    double cX = 0;
    double cY = 0;
    public static final double objectWidthInRealWorldUnits = 1.5;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1430;

    enum Position {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }

    enum Color {
        RED,
        BLUE,
        YELLOW


    }
    public static Color color = Color.YELLOW;



    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Mat processFrame(Mat input, long time) {
        // Preprocess the frame to detect yellow regions
        Mat mask = preprocessFrame(input);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest yellow contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
            // Calculate the width of the bounding box
            width = (int) calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

        }

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat yellowMask = new Mat();

        if(color == Color.YELLOW){
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        }
        if(color == Color.RED){
            Core.inRange(hsvFrame, lowRed1, highRed1, yellowMask);

        }
        if(color == Color.BLUE){
            Core.inRange(hsvFrame, lowBlue, highBlue, yellowMask);
        }


        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }
    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {



    }

    public Position getPosition(){
        return position;
    }

    public static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    public static void setColor(Color color1){
        color = color1;
    }
}
