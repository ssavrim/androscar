package com.kreolite.androvision;

import android.hardware.Camera;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import static java.lang.Math.abs;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 *
 * @author damonkohler@google.com (Damon Kohler)
 */
public class CarCommandListener extends AbstractNodeMain {
    private UsbService mUsbService;
    private CarCameraPublisher mCamera;
    private int cameraId;
    private int lastLinear = -1;
    private int lastAngular = -1;

    public CarCommandListener(UsbService usbService, CarCameraPublisher camera) {
        super();
        mUsbService = usbService;
        mCamera = camera;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_command/listener");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        cameraId = 0;
        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
                CarCommandPublisher.SIMPLE_ACTION_TOPIC, std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                try {
                    String command = message.getData();
                    JSONObject jsonObj = new JSONObject();
                    if (command.equals(CarCommandPublisher.SWITCH_CAMERA)) {
                        // Switch camera
                        int numberOfCameras = Camera.getNumberOfCameras();
                        if (numberOfCameras > 1) {
                            cameraId = (cameraId + 1) % numberOfCameras;
                            mCamera.releaseCamera();
                            mCamera.setCamera(Camera.open(cameraId));
                        }
                    } else if(command.equals(CarCommandPublisher.STOP)) {
                        jsonObj.put("pin1", 0);
                        jsonObj.put("pin2", 0);
                        jsonObj.put("pin3", 0);
                        jsonObj.put("pin4", 0);
                    }
                    if (mUsbService != null && jsonObj != null) {
                        mUsbService.write(jsonObj.toString().getBytes());
                    }
                } catch (JSONException e) {
                    Log.e(getDefaultNodeName().toString(), e.getMessage());
                }
            }
        });
        Subscriber<geometry_msgs.Twist> cmdVelSubscriber = connectedNode.newSubscriber(
                CarCommandPublisher.VELOCITY_ACTION_TOPIC, geometry_msgs.Twist._TYPE);
        cmdVelSubscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
            @Override
            public void onNewMessage(geometry_msgs.Twist message) {
                try {
                    double linearRatio = Math.round(message.getLinear().getX() * 10.0) / 10.0;
                    double angularRatio = Math.round(message.getAngular().getZ() * 10.0) / 10.0;
                    int pin1 = 0;
                    int pin2 = 0;
                    int pin3 = 0;
                    int pin4 = 0;
                    int currentLinear = 0;
                    int currentAngular = 0;
                    if (linearRatio != 0) {
                        //int linearValue = (int) abs(255 * linearRatio);
                        //currentLinear = linearValue >= 150 ? linearValue : 150;
                        currentLinear = 255;
                        currentAngular = currentLinear - (int) abs(currentLinear * angularRatio);
                        currentAngular = currentAngular >= 100 ? currentAngular : 100;
                        if (linearRatio > 0) {
                            pin1 = pin4 = 0;
                            pin2 = pin3 = currentLinear;
                            if (angularRatio < 0) {
                                pin2 = currentAngular;
                            } else {
                                pin3 = currentAngular;
                            }
                        } else {
                            pin1 = pin4 = currentLinear;
                            pin2 = pin3 = 0;
                            if (angularRatio < 0) {
                                pin1 = currentAngular;
                            } else {
                                pin4 = currentAngular;
                            }
                        }
                    }

                    JSONObject jsonObj = new JSONObject();
                    jsonObj.put("pin1", pin1);
                    jsonObj.put("pin2", pin2);
                    jsonObj.put("pin3", pin3);
                    jsonObj.put("pin4", pin4);

                    if (mUsbService != null) {
                        if (lastAngular != currentAngular) {
                            Log.i(getDefaultNodeName().toString(), jsonObj.toString());
                            mUsbService.write(jsonObj.toString().getBytes());
                        }
                    }
                    lastLinear = currentLinear;
                    lastAngular = currentAngular;
                } catch (JSONException e) {
                    Log.e(getDefaultNodeName().toString(), e.getMessage());
                }
            }
        });
    }
}