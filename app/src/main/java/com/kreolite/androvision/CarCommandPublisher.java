package com.kreolite.androvision;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 *
 * @author ssavrimo@gmail.com
 */

public class CarCommandPublisher extends AbstractNodeMain {
    private static final String TAG = "CarCameraPublisher";
    private Publisher<std_msgs.String> publisherCmdSimple;
    private Publisher<geometry_msgs.Twist> publisherCmdVelocity;

    public static final String SIMPLE_ACTION_TOPIC = "car_command/cmd_simple";
    public static final String VELOCITY_ACTION_TOPIC = "car_command/cmd_vel";

    public static final String SWITCH_CAMERA = "switch_camera";
    public static final String FORWARD = "forward";
    public static final String REVERSE = "reverse";
    public static final String LEFT = "left";
    public static final String RIGHT = "right";
    public static final String STOP = "stop";

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_command/publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        publisherCmdVelocity = connectedNode.newPublisher(VELOCITY_ACTION_TOPIC, geometry_msgs.Twist._TYPE);
        publisherCmdSimple = connectedNode.newPublisher(SIMPLE_ACTION_TOPIC, std_msgs.String._TYPE);
    }

    @Override
    public void onShutdown(Node arg0) {
        publisherCmdSimple.shutdown();
        publisherCmdVelocity.shutdown();
        super.onShutdown(arg0);
    }

    public void publishCmdSimple(String command) {
        if (publisherCmdSimple != null) {
            Log.i(TAG, command);
            std_msgs.String message = publisherCmdSimple.newMessage();
            message.setData(command);
            publisherCmdSimple.publish(message);
        }
    }

    public void publishCmdVelocity(double linearVelocityX, double angularVelocityZ) {
        if (publisherCmdVelocity != null) {
            geometry_msgs.Twist message = publisherCmdVelocity.newMessage();
            message.getLinear().setX(linearVelocityX);
            message.getLinear().setY(0.0D);
            message.getLinear().setZ(0.0D);
            message.getAngular().setX(0.0D);
            message.getAngular().setY(0.0D);
            message.getAngular().setZ(angularVelocityZ);
            publisherCmdVelocity.publish(message);
        }
    }
}
