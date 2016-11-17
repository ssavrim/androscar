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

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import java.util.Set;

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