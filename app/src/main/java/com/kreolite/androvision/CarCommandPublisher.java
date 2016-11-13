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
    private String topic_name;
    private Publisher<std_msgs.String> publisher;

    public CarCommandPublisher() {
        topic_name = "command";
    }

    public CarCommandPublisher(String topic) {
        topic_name = topic;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car/command");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(topic_name, std_msgs.String._TYPE);
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
