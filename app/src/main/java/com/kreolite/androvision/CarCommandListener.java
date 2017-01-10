package com.kreolite.androvision;

import android.hardware.Camera;
import android.util.Log;

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

    private static final String TAG = "CarCommandListener";
    private final CarCommandBackend mCarCommandBackend;
    private CarCameraPublisher mCamera;
    private int cameraId;
    private String lastPinValues = "";

    public CarCommandListener(CarCommandBackend carCommandBackend, CarCameraPublisher camera) {
        super();
        mCarCommandBackend = carCommandBackend;
        mCamera = camera;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_command/listener");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        cameraId = -1;
        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
                CarCommandPublisher.SIMPLE_ACTION_TOPIC, std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                try {
                    String command = message.getData();
                    JSONObject jsonObj = new JSONObject();
                    String currentPinValues = "";
                    if (command.equals(CarCommandPublisher.SWITCH_CAMERA)) {
                        // Switch camera or disable it
                        int numberOfCameras = Camera.getNumberOfCameras();
                        if (numberOfCameras > 1) {
                            cameraId = (cameraId + 1) % (numberOfCameras + 1);
                            mCamera.releaseCamera();
                            if (cameraId != numberOfCameras) {
                                mCamera.setCamera(Camera.open(cameraId));
                                Log.d(TAG, "setCamera getPreviewSize: " + mCamera.getPreviewSize().height + "x" + mCamera.getPreviewSize().width);
                            }
                        }
                    } else if(command.equals(CarCommandPublisher.STOP)) {
                        mCarCommandBackend.setMotorActions(0, 0, 0, 0);
                    } else if (command.equals(CarCommandPublisher.FORWARD)) {
                        mCarCommandBackend.setMotorActions(0, 255, 255, 0);
                    } else if (command.equals(CarCommandPublisher.REVERSE)) {
                        mCarCommandBackend.setMotorActions(255, 0, 0, 255);
                    } else if (command.equals(CarCommandPublisher.LEFT)) {
                        mCarCommandBackend.setMotorActions(0, 180, 0, 180);
                    } else if (command.equals(CarCommandPublisher.RIGHT)) {
                        mCarCommandBackend.setMotorActions(180, 0, 180, 0);
                    }
                } catch (Exception e) {
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

                    if (linearRatio != 0) {
                        int currentLinear = (int) (255 * abs(linearRatio));
                        int currentAngular = currentLinear - (int) (currentLinear * abs(angularRatio));
                        //currentAngular = currentAngular >= 100 ? currentAngular : 100;
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
                    Log.i(TAG, "Command actions: " + pin1 + " - " + pin2 + " - " + pin3 + " - " + pin4);
                    mCarCommandBackend.setMotorActions(pin1, pin2, pin3, pin4);
                } catch (Exception e) {
                    Log.e(TAG, e.getMessage());
                }
            }
        });
    }
}