package com.kreolite.androvision;

import android.util.Log;

import org.ros.message.Time;
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
    private ConnectedNode mConnectedNode = null;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_sensor/publisher");
    }
    @Override
    public void onStart(final ConnectedNode connectedNode) {
        mConnectedNode = connectedNode;
        rangePublisher = connectedNode.newPublisher(FRONT_RANGE_TOPIC, sensor_msgs.Range._TYPE);
    }

    @Override
    public void onShutdown(Node arg0) {
        rangePublisher = null;
    }

    public void publishRange(String message) {
        try {
            if (rangePublisher != null && !message.isEmpty()) {
                Log.i("CarSensor", message);
                String[] messageSplit = message.split(",");
                Time currentTime = mConnectedNode.getCurrentTime();
                sensor_msgs.Range range = rangePublisher.newMessage();
                range.getHeader().setFrameId("front_range");
                range.getHeader().setStamp(currentTime);
                range.setRadiationType(sensor_msgs.Range.ULTRASOUND);
                range.setFieldOfView(0.1f);
                range.setMinRange(Float.parseFloat(messageSplit[0]));
                range.setMaxRange(Float.parseFloat(messageSplit[1]));
                range.setRange(Float.parseFloat(messageSplit[2]));
                rangePublisher.publish(range);
            }
        } catch (NumberFormatException | ArrayIndexOutOfBoundsException e) {
            Log.e("CarSensor", "Wrong range: " + e);
        }
    }
}
