package com.kreolite.androvision;

import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.Toast;

import org.json.JSONException;
import org.json.JSONObject;
import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.android.view.RosTextView;
import org.ros.android.view.VirtualJoystickView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;


public class RosRemoteControlActivity extends RosActivity {
    private static final String _TAG = "RosRemoteControl";
    public static final String MASTER_URI_EXTRA = "MASTER_URI";
    private RosImageView<sensor_msgs.CompressedImage> rosImageView;
    private RosTextView<sensor_msgs.Range> rosDistanceView;
    private VirtualJoystickView rosJoystick;
    private final CarCommandPublisher carCommand;
    private JSONObject mFrontRange;

    public RosRemoteControlActivity() {
        super(_TAG, _TAG);
        carCommand = new CarCommandPublisher();
    }
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_ros_remote_control);
        setup();
        mFrontRange = new JSONObject();
    }
    @Override
    protected void onPause() {
        RosRemoteControlActivity.this.nodeMainExecutorService.forceShutdown();
        super.onPause();
    }
    public void startMasterChooser() {
        URI masterUri = null;
        Bundle bundle = getIntent().getBundleExtra(MASTER_URI_EXTRA);
        if (bundle != null) {
            if (bundle.getString(MASTER_URI_EXTRA) != null){
                try {
                    masterUri = new URI(bundle.getString(MASTER_URI_EXTRA));
                    nodeMainExecutorService.setMasterUri(masterUri);
                } catch (URISyntaxException e) { }
            }
        }
        if (masterUri == null ){
            super.startMasterChooser();
        } else {
            RosRemoteControlActivity.this.init();
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_UP) {
            final Toast toast = Toast.makeText(this, "Switching cameras.", Toast.LENGTH_SHORT);
            carCommand.publish(CarCommandPublisher.SWITCH_CAMERA);
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
        URI masterUri = getMasterUri();
        String hostAddress = InetAddressFactory.newNonLoopback().getHostAddress();
        NodeConfiguration nodeConfiguration =  NodeConfiguration.newPublic(hostAddress, masterUri);
        nodeMainExecutor.execute(rosImageView, nodeConfiguration);
        nodeMainExecutor.execute(rosDistanceView, nodeConfiguration);
        nodeMainExecutor.execute(rosJoystick, nodeConfiguration);
        nodeMainExecutor.execute(carCommand, nodeConfiguration);
    }
    private void setup() {
        // Initialize view of the car camera
        rosImageView = (RosImageView<sensor_msgs.CompressedImage>)findViewById(R.id.rosImageView);
        rosImageView.setTopicName("/camera/image/compressed");
        rosImageView.setMessageType(sensor_msgs.CompressedImage._TYPE);
        rosImageView.setMessageToBitmapCallable(new BitmapFromCompressedImage());

        // Initialize view of the ultrasonic value
        rosDistanceView = (RosTextView<sensor_msgs.Range>) findViewById(R.id.distanceView);
        rosDistanceView.setTopicName(CarSensorPublisher.FRONT_RANGE_TOPIC);
        rosDistanceView.setMessageType(sensor_msgs.Range._TYPE);
        rosDistanceView.setMessageToStringCallable(new MessageCallable<String, sensor_msgs.Range>() {
            @Override
            public String call(sensor_msgs.Range message) {
                try {
                    if (message.getRange() > 0) {
                        mFrontRange.put(message.getHeader().getFrameId(), Float.toString(message.getRange()) + " cm");
                    }
                    return "left: " + mFrontRange.get("/left") + " - " + "right: " + mFrontRange.get("/right");
                } catch(JSONException e) {
                    return "---";
                }
            }
        });

        // Initialize simple actions to send to the camera
        Button btnUp = (Button)findViewById(R.id.btnForward);
        btnUp.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish(CarCommandPublisher.FORWARD);
            }
        });
        Button btnDown = (Button)findViewById(R.id.btnReverse);
        btnDown.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish(CarCommandPublisher.REVERSE);
            }
        });
        Button btnLeft = (Button)findViewById(R.id.btnLeft);
        btnLeft.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish(CarCommandPublisher.LEFT);
            }
        });
        Button btnRight = (Button)findViewById(R.id.btnRight);
        btnRight.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish(CarCommandPublisher.RIGHT);
            }
        });
        Button btnStop = (Button)findViewById(R.id.btnStop);
        btnStop.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish(CarCommandPublisher.STOP);
            }
        });

        // Initialize simple actions to send to the camera
        rosJoystick = (VirtualJoystickView) findViewById(R.id.virtualJoystick);
        rosJoystick.setTopicName(CarCommandPublisher.VELOCITY_ACTION_TOPIC);
    }
}
