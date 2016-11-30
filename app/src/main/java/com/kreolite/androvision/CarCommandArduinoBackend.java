package com.kreolite.androvision;

import android.util.Log;

/**
 * Created by ptardy on 11/29/16.
 */

public class CarCommandArduinoBackend implements CarCommandBackend {
    private final UsbService mUsbService;
    private static final String TAG = "CarCmdArduinoBackend";
    private String lastPinValues="";

    public CarCommandArduinoBackend(UsbService usbService){
        mUsbService = usbService;
    }
    @Override
    public void setMotorActions(int motor1, int motor1Rev, int motor2, int motor2Rev) {
        String currentPinValues = String.format("%02X", motor1);
        currentPinValues += String.format("%02X", motor1Rev);
        currentPinValues += String.format("%02X", motor2);
        currentPinValues += String.format("%02X", motor2Rev);

        if (!lastPinValues.equals(currentPinValues)) {
            Log.i(TAG, currentPinValues);
            if (mUsbService != null) {
                mUsbService.write(currentPinValues.getBytes());
            }
            lastPinValues = currentPinValues;
        }
    }
}
