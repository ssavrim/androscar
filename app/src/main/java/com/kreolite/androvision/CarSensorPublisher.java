package com.kreolite.androvision;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
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
    public static final String ORIENTATION_TOPIC = "car_sensor/orientation";

    private ConnectedNode mConnectedNode = null;
    private Publisher<sensor_msgs.Range> rangePublisher;
    private Publisher<geometry_msgs.PoseStamped> orientationPublisher;
    private final SensorManager mSensorManager;
    private CarSensorPublisher.MySensorListener mSensorListener;

    public CarSensorPublisher(SensorManager sensorManager) {
        mSensorManager = sensorManager;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_sensor/publisher");
    }
    @Override
    public void onStart(final ConnectedNode connectedNode) {
        mConnectedNode = connectedNode;
        try {
            // Initialize publisher for ultrasound
            rangePublisher = connectedNode.newPublisher(FRONT_RANGE_TOPIC, sensor_msgs.Range._TYPE);

            // Initialize publisher for orientation
            orientationPublisher = connectedNode.newPublisher(ORIENTATION_TOPIC, geometry_msgs.PoseStamped._TYPE);
            mSensorListener = new CarSensorPublisher.MySensorListener(orientationPublisher);
            Sensor sensorRotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
            //Sensor sensorAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            //Sensor sensorGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            //Sensor sensorAmbientTemperature = mSensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
            mSensorManager.registerListener(mSensorListener, sensorRotation, SensorManager.SENSOR_DELAY_NORMAL);
            //mSensorManager.registerListener(mOrientationListener, sensorAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
            //mSensorManager.registerListener(mOrientationListener, sensorGyroscope, SensorManager.SENSOR_DELAY_NORMAL);
            //mSensorManager.registerListener(mOrientationListener, sensorAmbientTemperature, SensorManager.SENSOR_DELAY_NORMAL);
        } catch (Exception var4) {
            connectedNode.getLog().fatal(var4);
        }
    }
    @Override
    public void onShutdown(Node arg0) {
        rangePublisher = null;
        if (mSensorManager != null) {
            mSensorManager.unregisterListener(mSensorListener);
        }
    }

    public void publishRange(String message) {
        try {
            if (rangePublisher != null && !message.isEmpty()) {
                Log.d("CarSensor", "Publish front range: " + message);
                String[] messageSplit = message.split(",");
                Time currentTime = mConnectedNode.getCurrentTime();
                sensor_msgs.Range range = rangePublisher.newMessage();
                range.getHeader().setFrameId("/front_range");
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

    private final class MySensorListener implements SensorEventListener {
        private final Publisher<geometry_msgs.PoseStamped> publisher;

        private MySensorListener(Publisher<geometry_msgs.PoseStamped> var1) {
            this.publisher = var1;
        }
        public void onSensorChanged(SensorEvent event) {
            String sensorName = event.sensor.getName();
            Log.d("CarSensor", sensorName + ": X: " + event.values[0] + "; Y: " + event.values[1] + "; Z: " + event.values[2] + ";");
            if(event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                Time currentTime = mConnectedNode.getCurrentTime();
                float[] quaternion = new float[4];
                SensorManager.getQuaternionFromVector(quaternion, event.values);
                geometry_msgs.PoseStamped pose = this.publisher.newMessage();
                pose.getHeader().setFrameId("/map");
                pose.getHeader().setStamp(currentTime);
                pose.getPose().getOrientation().setW((double)quaternion[0]);
                pose.getPose().getOrientation().setX((double)quaternion[1]);
                pose.getPose().getOrientation().setY((double)quaternion[2]);
                pose.getPose().getOrientation().setZ((double)quaternion[3]);
                this.publisher.publish(pose);
            }
        }
        public void onAccuracyChanged(Sensor sensor, int accuracy) {}
    }
}
