package com.kreolite.androvision;

import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.hardware.Camera;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.util.Log;
import android.view.MotionEvent;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.lang.ref.WeakReference;
import java.net.URI;
import java.util.Set;

public class RosCameraGPIOActivity extends RosActivity {
    private static final String _TAG = "RosCamera";
    private int cameraId=0;
    public CarCommandListener carCommand;
    public CarCameraPublisher carCamera;
    public CarSensorPublisher carSensor;
    private SensorManager mSensorManager;
    private NsdHelper mNsdHelper;

    public RosCameraGPIOActivity() {
        super(_TAG, _TAG);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_ros_camera);
        carCamera = (CarCameraPublisher) findViewById(R.id.ros_camera_preview_view);
        carCommand = new CarCommandListener(new CarCommandGPIOBackend(), carCamera);
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        carSensor = new CarSensorPublisher(mSensorManager);
        mNsdHelper = new NsdHelper(this);
    }
    public void startMasterChooser() {
        nodeMainExecutorService.startMaster(false);
        init(nodeMainExecutorService);
    }

    @Override
    public void onResume() {
        super.onResume();
        mNsdHelper.initializeNsd();
    }
    @Override
    public void onPause() {
        super.onPause();
        mNsdHelper.tearDown();
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        URI masterUri = getMasterUri();
        String hostAddress = InetAddressFactory.newNonLoopback().getHostAddress();
        mNsdHelper.registerService(masterUri.getPort());
        NodeConfiguration nodeConfiguration =  NodeConfiguration.newPublic(hostAddress, masterUri);
        nodeMainExecutor.execute(carCamera, nodeConfiguration);
        nodeMainExecutor.execute(carCommand, nodeConfiguration);
        nodeMainExecutor.execute(carSensor, nodeConfiguration);
    }

}