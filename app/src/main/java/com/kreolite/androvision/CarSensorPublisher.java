package com.kreolite.androvision;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

/**
 * Created by kreolite on 21/11/16.
 */

public class CarSensorPublisher extends AbstractNodeMain {
    public static final String FRONT_RANGE_TOPIC = "car_sensor/front_range";

    private Publisher<sensor_msgs.Range> rangePublisher;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_sensor/publisher");
    }
    @Override
    public void onStart(final ConnectedNode connectedNode) {
        rangePublisher = connectedNode.newPublisher(FRONT_RANGE_TOPIC, sensor_msgs.Range._TYPE);
    }

    @Override
    public void onShutdown(Node arg0) {
        rangePublisher = null;
    }
    public void publishRange(String command) {

        try {
            if (rangePublisher != null && !command.isEmpty()) {
                Log.i("CarSensor", "Range: " + command + " cm");
                sensor_msgs.Range message = rangePublisher.newMessage();
                message.getHeader().setFrameId("front_range");
                message.setRadiationType(sensor_msgs.Range.ULTRASOUND);
                message.setRange(Float.parseFloat(command));
                message.setMinRange(0);
                message.setMaxRange(450);
                rangePublisher.publish(message);
            }
        } catch (NumberFormatException e) {
            Log.e("CarSensor", "Wrong range: " + e);
        }
    }
}
