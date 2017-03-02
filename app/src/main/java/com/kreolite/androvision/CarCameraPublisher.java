package com.kreolite.androvision;

import android.content.Context;
import android.util.AttributeSet;
import android.util.Log;

import com.kreolite.androvision.cv_bridge.CvImage;
import com.kreolite.androvision.cv_bridge.Format;
import com.kreolite.androvision.cv_bridge.ImageEncodings;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;

import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;


/**
 * Created by ssavrim on 11/17/2016.
 */

public class CarCameraPublisher extends AbstractNodeMain {
    private static final String TAG = "CarCameraPublisher";
    private Publisher<sensor_msgs.CompressedImage> imagePublisher;
    private Publisher<sensor_msgs.CameraInfo> cameraInfoPublisher;
    public static final String CAMERA_INFO_TOPIC = "camera/camera_info";
    public static final String CAMERA_IMAGE_TOPIC = "camera/image/compressed";

    private ConnectedNode mConnectedNode;
    private CameraBridgeViewBase   mOpenCvCameraView;
    private Mat                    mRgba;
    private Mat                    mRgbaF;
    private Mat                    mRgbaT;

    public GraphName getDefaultNodeName() {
        return GraphName.of("car_camera/publisher");
    }

    public CarCameraPublisher(CameraBridgeViewBase openCvCameraView) {
        super();
        mOpenCvCameraView = openCvCameraView;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        mConnectedNode = connectedNode;
        imagePublisher = connectedNode.newPublisher(CAMERA_IMAGE_TOPIC, sensor_msgs.CompressedImage._TYPE);
        cameraInfoPublisher = connectedNode.newPublisher(CAMERA_INFO_TOPIC, sensor_msgs.CameraInfo._TYPE);

    }

    @Override
    public void onShutdown(Node arg0) {
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        super.onShutdown(arg0);
    }

    public void releaseCamera() {
        if(mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }
    public void setCamera(int cameraIndex) {
        if(mOpenCvCameraView != null) {
            mOpenCvCameraView.setCameraIndex(cameraIndex);
            mOpenCvCameraView.enableView();
        }
    }
    public void publishImage(Mat inputFrame, Size screenSize){
        try {
            Time currentTime = mConnectedNode.getCurrentTime();
            String frameId = "camera";
            sensor_msgs.CompressedImage image = imagePublisher.newMessage();
            image.setFormat("jpeg");
            image.getHeader().setStamp(currentTime);
            image.getHeader().setFrameId(frameId);

            CvImage cvImage = new CvImage(image.getHeader(), ImageEncodings.RGB8);
            cvImage.image = inputFrame;
            imagePublisher.publish(cvImage.toCompressedImageMsg(image, Format.JPG));

            sensor_msgs.CameraInfo cameraInfo = cameraInfoPublisher.newMessage();
            cameraInfo.getHeader().setStamp(currentTime);
            cameraInfo.getHeader().setFrameId(frameId);
            cameraInfo.setWidth((int) screenSize.width);
            cameraInfo.setHeight((int) screenSize.height);
            cameraInfoPublisher.publish(cameraInfo);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage());
            return;
        }
    }
}
