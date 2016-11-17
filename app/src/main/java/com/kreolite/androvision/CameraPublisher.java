package com.kreolite.androvision;

import android.content.Context;
import android.util.AttributeSet;

import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.namespace.GraphName;

/**
 * Created by ssavrim on 11/17/2016.
 */

public class CameraPublisher extends RosCameraPreviewView {
    public CameraPublisher(Context context) {
        super(context);
    }

    public CameraPublisher(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public CameraPublisher(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }
    public GraphName getDefaultNodeName() {
        return GraphName.of("car_camera/publisher");
    }
}
