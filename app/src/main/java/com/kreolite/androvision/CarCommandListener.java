package com.kreolite.androvision;

import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.widget.Toast;

import org.json.JSONException;
import org.json.JSONObject;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import java.util.Arrays;
import java.util.List;
import java.util.Set;

import static java.lang.Math.abs;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 *
 * @author damonkohler@google.com (Damon Kohler)
 */
public class CarCommandListener extends AbstractNodeMain {
    private Context mContext;
    private CameraPublisher mCamera;
    private String mTopicName;
    private int cameraId;
    private int lastLinear = 0;
    private int lastAngular = 0;

    public CarCommandListener(Context context, CameraPublisher camera, String topicName) {
        super();
        mContext = context;
        mCamera = camera;
        mTopicName = topicName;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_command/listener");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        cameraId = 0;
        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(mTopicName, std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                Log.i(getDefaultNodeName().toString(), "Received command:\"" + message.getData() + "\"");
                String command = message.getData();
                if (command.equals("10")) {
                    // Switch camera
                    int numberOfCameras = Camera.getNumberOfCameras();
                    final Toast toast;
                    if (numberOfCameras > 1) {
                        cameraId = (cameraId + 1) % numberOfCameras;
                        mCamera.releaseCamera();
                        mCamera.setCamera(Camera.open(cameraId));
                    }
                }
                if (usbService != null) {
                    usbService.write(command.getBytes());
                }
            }
        });
        Subscriber<geometry_msgs.Twist> cmdVelSubscriber = connectedNode.newSubscriber("car_command/cmd_vel", geometry_msgs.Twist._TYPE);
        cmdVelSubscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
            @Override
            public void onNewMessage(geometry_msgs.Twist message) {
                try {
                    Double linearRatio = message.getLinear().getX();
                    Double angularRatio = message.getAngular().getZ();
                    int pin1 = 0;
                    int pin2 = 0;
                    int pin3 = 0;
                    int pin4 = 0;
                    int linear = 0;
                    if (linearRatio != 0) {
                        linear = (int) abs(255 * linearRatio);
                        if (linearRatio > 0) {
                            pin1 = pin4 = 0;
                            pin2 = pin3 = linear >= 150 ? linear : 150;
                        } else {
                            pin1 = pin4 = linear >= 150 ? linear : 150;
                            pin2 = pin3 = 0;
                        }
                    }

                    JSONObject jsonObj = new JSONObject();
                    jsonObj.put("pin1", pin1);
                    jsonObj.put("pin2", pin2);
                    jsonObj.put("pin3", pin3);
                    jsonObj.put("pin4", pin4);

                    if (usbService != null) {
                        if (lastLinear != linear) {
                            Log.i(getDefaultNodeName().toString(), jsonObj.toString());
                            usbService.write(jsonObj.toString().getBytes());
                            lastLinear = linear;
                        }

                    }
                } catch (JSONException e) {
                    Log.e(getDefaultNodeName().toString(), e.getMessage());
                }
            }
        });
        setFilters();  // Start listening notifications from UsbService
        startService(UsbService.class, usbConnection, null); // Start UsbService(if it was not started before) and Bind it
    }

    @Override
    public void onShutdown(Node node) {
        if (mContext != null) {
            mContext.unregisterReceiver(mUsbReceiver);
            mContext.unbindService(usbConnection);
        }
    }

    /*
     * Notifications from UsbService will be received here.
     */
    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            switch (intent.getAction()) {
                case UsbService.ACTION_USB_PERMISSION_GRANTED: // USB PERMISSION GRANTED
                    Log.i(getDefaultNodeName().toString(), "USB Ready");
                    break;
                case UsbService.ACTION_USB_PERMISSION_NOT_GRANTED: // USB PERMISSION NOT GRANTED
                    Log.i(getDefaultNodeName().toString(), "USB Permission not granted");
                    break;
                case UsbService.ACTION_NO_USB: // NO USB CONNECTED
                    Log.i(getDefaultNodeName().toString(), "No USB connected");
                    break;
                case UsbService.ACTION_USB_DISCONNECTED: // USB DISCONNECTED
                    Log.i(getDefaultNodeName().toString(), "USB disconnected");
                    break;
                case UsbService.ACTION_USB_NOT_SUPPORTED: // USB NOT SUPPORTED
                    Log.i(getDefaultNodeName().toString(), "USB device not supported");
                    break;
            }
        }
    };
    private UsbService usbService;
    private final ServiceConnection usbConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName arg0, IBinder arg1) {
            usbService = ((UsbService.UsbBinder) arg1).getService();
        }

        @Override
        public void onServiceDisconnected(ComponentName arg0) {
            usbService = null;
        }
    };

    private void startService(Class<?> service, ServiceConnection serviceConnection, Bundle extras) {
        if (!UsbService.SERVICE_CONNECTED) {
            Intent startServiceIntent = new Intent(mContext, service);
            if (extras != null && !extras.isEmpty()) {
                Set<String> keys = extras.keySet();
                for (String key : keys) {
                    String extra = extras.getString(key);
                    startServiceIntent.putExtra(key, extra);
                }
            }
            mContext.startService(startServiceIntent);
        }
        Intent bindingIntent = new Intent(mContext, service);
        mContext.bindService(bindingIntent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    private void setFilters() {
        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbService.ACTION_USB_PERMISSION_GRANTED);
        filter.addAction(UsbService.ACTION_NO_USB);
        filter.addAction(UsbService.ACTION_USB_DISCONNECTED);
        filter.addAction(UsbService.ACTION_USB_NOT_SUPPORTED);
        filter.addAction(UsbService.ACTION_USB_PERMISSION_NOT_GRANTED);
        mContext.registerReceiver(mUsbReceiver, filter);
    }
}