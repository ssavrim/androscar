package com.kreolite.androvision;

import android.hardware.Camera;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.Toast;

import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;

public class RosRemoteControlActivity extends RosActivity {
    private static final String _TAG = "RosRemoteControl";
    private RosImageView<sensor_msgs.CompressedImage> rosImageView;
    private final CarCommandPublisher carCommand;

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
        rosImageView = (RosImageView<sensor_msgs.CompressedImage>)findViewById(R.id.rosImageView);
        rosImageView.setTopicName("/camera/image/compressed");
        rosImageView.setMessageType(sensor_msgs.CompressedImage._TYPE);
        rosImageView.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        setup();
    }
    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_UP) {
            final Toast toast = Toast.makeText(this, "Switching cameras.", Toast.LENGTH_SHORT);
            carCommand.publish("10");
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
        try {
            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration =
                    NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
            nodeMainExecutor.execute(rosImageView, nodeConfiguration);
            nodeMainExecutor.execute(carCommand, nodeConfiguration);
        } catch (IOException e) {
            // Socket problem
            Log.e(_TAG, "socket error trying to get networking information from the master uri");
        }

    }
    private void setup() {
        Button btnUp = (Button)findViewById(R.id.btnForward);
        btnUp.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish("8");
            }
        });
        Button btnDown = (Button)findViewById(R.id.btnReverse);
        btnDown.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish("2");
            }
        });
        Button btnLeft = (Button)findViewById(R.id.btnLeft);
        btnLeft.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish("4");
            }
        });
        Button btnRight = (Button)findViewById(R.id.btnRight);
        btnRight.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish("6");
            }
        });
        Button btnStop = (Button)findViewById(R.id.btnStop);
        btnStop.setOnClickListener(new View.OnClickListener(){
            public void onClick(View v){
                carCommand.publish("0");
            }
        });
    }
}
