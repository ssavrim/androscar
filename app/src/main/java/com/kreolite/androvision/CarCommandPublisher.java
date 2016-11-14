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
    private String topicName;
    private Publisher<std_msgs.String> publisher;

    public CarCommandPublisher() {
        topicName = "command";
    }

    public CarCommandPublisher(String topic) {
        topicName = topic;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_command/publisher");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(topicName, std_msgs.String._TYPE);
    }

    @Override
    public void onShutdown(Node arg0) {
        publisher = null;
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
