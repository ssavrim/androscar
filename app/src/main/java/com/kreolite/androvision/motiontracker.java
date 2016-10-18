package com.kreolite.androvision;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.WindowManager;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class motiontracker extends AppCompatActivity implements CvCameraViewListener2 {

    private static final String    TAG = "androVision::Motion";
    private static int SENSITIVITY_VALUE = 30;
    private static int BLUR_SIZE = 50;
    private CameraBridgeViewBase mOpenCvCameraView;
    private Mat                    mRgba;
    private Mat                    mGrayLast;
    private Mat                    mGrayCurrent;
    private Mat                    mDiffImg;
    private Mat                    mThresholdImage;
    private Mat                    mEdges;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public motiontracker() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.activity_motiontracker);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.activity_motiontracker);
        mOpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
    }
    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }
    public void onCameraViewStarted(int width, int height) {
        Log.d(TAG, "onCameraViewStarted: " + width + "*" + height);
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mGrayLast = new Mat(height, width, CvType.CV_8UC1);
        mGrayCurrent = new Mat(height, width, CvType.CV_8UC1);
        mDiffImg = new Mat(height, width, CvType.CV_8UC1);
        mThresholdImage = new Mat(height, width, CvType.CV_8UC1);
        mEdges = new Mat(height, width, CvType.CV_8UC1);
    }

    public void onCameraViewStopped() {
        mRgba.release();
        mGrayLast.release();
        mGrayCurrent.release();
        mDiffImg.release();
        mThresholdImage.release();
        mEdges.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        if (mGrayLast == null) {
            mGrayLast = inputFrame.gray();
            Imgproc.resize(inputFrame.gray(), mGrayLast, new Size(640, 480));
        } else {
            mRgba = inputFrame.rgba();
            mGrayCurrent = inputFrame.gray();

            /*Core.absdiff(mGrayLast, mGrayCurrent, mDiffImg);
            Imgproc.threshold(mDiffImg, mThresholdImage,
                    SENSITIVITY_VALUE, 255, Imgproc.THRESH_BINARY);
            Imgproc.blur(mThresholdImage, mThresholdImage, new Size(BLUR_SIZE, BLUR_SIZE));
            Imgproc.threshold(mThresholdImage, mThresholdImage,
                    SENSITIVITY_VALUE, 255, Imgproc.THRESH_BINARY);
            searchForMovement(mThresholdImage, mRgba);*/
            searchForCircles();

            // Register current gray frame
            mGrayLast = inputFrame.gray();
        }
        return mRgba;
    }

    private void searchForCircles()
    {
        long start_time = System.nanoTime();
        Mat circles = new Mat();
        int iCannyUpperThreshold = 100;
        int iMinRadius = 20;
        int iMaxRadius = 400;
        int iAccumulator = 300;
        int iLineThickness = 5;

        //Imgproc.Canny(mGrayCurrent, mThresholdImage, 80, 120);
        Imgproc.medianBlur(mGrayCurrent, mThresholdImage, 5);
        Imgproc.HoughCircles(mThresholdImage, circles, Imgproc.CV_HOUGH_GRADIENT,
                2.0, mThresholdImage.rows() / 8, iCannyUpperThreshold, iAccumulator,
                iMinRadius, iMaxRadius);

        if (circles.cols() > 0)
            Log.i(TAG, "found circles :  " +  circles.cols());
            for (int x = 0; x < circles.cols(); x++)
            {
                double vCircle[] = circles.get(0,x);

                if (vCircle == null)
                    break;

                Point pt = new Point(Math.round(vCircle[0]), Math.round(vCircle[1]));
                int radius = (int)Math.round(vCircle[2]);

                // draw the found circle
                Imgproc.circle(mRgba, pt, radius, new Scalar(0,255,0), iLineThickness);
                Imgproc.circle(mRgba, pt, 3, new Scalar(0,0,255), iLineThickness);
            }

        long end_time = System.nanoTime();
        long duration = (end_time - start_time)/1000000;  //divide by 1000000 to get milliseconds.
        Log.i(TAG, "duration :  " + duration * 0.001 + " s");
    }

    private void searchForMovement(Mat thresholdImage, Mat frame)
    {

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholdImage, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect objectBoundingRectangle = new Rect(0, 0, 0, 0);
        for (int i = 0; i < contours.size(); i++)
        {
            objectBoundingRectangle = Imgproc.boundingRect(contours.get(i));
            if(objectBoundingRectangle.area()>500)
                /*Imgproc.putText(frame,
                        "object(" + objectBoundingRectangle.size() + ")",
                        new Point(objectBoundingRectangle.x-20, objectBoundingRectangle.y-10),
                        Core.FONT_HERSHEY_SIMPLEX,
                        1,
                        new Scalar(255,255,255));*/
                Imgproc.rectangle(frame, objectBoundingRectangle.tl(), objectBoundingRectangle.br(), new Scalar(0,255,0));
        }

    }
}
