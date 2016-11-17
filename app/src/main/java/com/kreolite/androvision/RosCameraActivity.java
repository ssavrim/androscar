package com.kreolite.androvision;

import android.hardware.Camera;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;

import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;

public class RosCameraActivity extends RosActivity {
    private static final String _TAG = "RosCamera";
    private int cameraId;
    private CarCommandListener carCommand;
    private CameraPublisher carCamera;

    public RosCameraActivity() {
        super(_TAG, _TAG);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_ros_camera);
        carCamera = (CameraPublisher) findViewById(R.id.ros_camera_preview_view);
        carCommand = new CarCommandListener(this, carCamera, "command");
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_UP) {
            int numberOfCameras = Camera.getNumberOfCameras();
            final Toast toast;
            if (numberOfCameras > 1) {
                cameraId = (cameraId + 1) % numberOfCameras;
                carCamera.releaseCamera();
                carCamera.setCamera(Camera.open(cameraId));
                toast = Toast.makeText(this, "Switching cameras.", Toast.LENGTH_SHORT);
            } else {
                toast = Toast.makeText(this, "No alternative cameras to switch to.", Toast.LENGTH_SHORT);
            }
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    toast.show();
                }
            });
        }
        return true;
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        cameraId = 0;

        carCamera.setCamera(Camera.open(cameraId));
        try {
            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration =
                    NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
            nodeMainExecutor.execute(carCamera, nodeConfiguration);
            nodeMainExecutor.execute(carCommand, nodeConfiguration);
        } catch (IOException e) {
            // Socket problem
            Log.e(_TAG, "socket error trying to get networking information from the master uri");
        }

    }
}