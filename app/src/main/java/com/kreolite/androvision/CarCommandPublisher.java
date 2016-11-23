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
    private Publisher<std_msgs.String> publisher;

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
    public void onStart(final ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(SIMPLE_ACTION_TOPIC, std_msgs.String._TYPE);
    }

    @Override
    public void onShutdown(Node arg0) {
        publisher.shutdown();
    }

    public void publish(String command) {
        if (publisher != null) {
            Log.i("CarCommand", command);
            std_msgs.String message = publisher.newMessage();
            message.setData(command);
            publisher.publish(message);
        }
    }
}
