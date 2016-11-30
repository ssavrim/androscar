package com.kreolite.androvision;

import android.annotation.SuppressLint;
import android.os.AsyncTask;
import android.util.Log;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.RandomAccessFile;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.Scanner;

/**
 * Created by ptardy on 11/29/16.
 */
public class CarCommandGPIOBackend implements CarCommandBackend {
    private static final String TAG = "CarCmdGPIOBackend";
    private int gpioMotor1 = 0;
    private int gpioMotor1Rev;
    private int gpioMotor2;
    private int gpioMotor2Rev;
    private int pwnValue;
    Socket mSocket;

    public CarCommandGPIOBackend() {
        try {
            String content = (new RandomAccessFile("/data/androvision_gpio_config.txt", "r")).readLine();
            Scanner sc = new Scanner(content);
            gpioMotor1 = sc.nextInt();
            gpioMotor1Rev = sc.nextInt();
            gpioMotor2 = sc.nextInt();
            gpioMotor2Rev = sc.nextInt();
            Log.d(TAG, "CarCommandGPIOBackend: " + gpioMotor1 + " "+ gpioMotor1Rev + " " + gpioMotor2 + " "+ gpioMotor2Rev);

        } catch (IOException e) {
            e.printStackTrace();
        }
        (new AsyncTask<Void, Void, Void>() {

            @Override
            protected Void doInBackground(Void[] _) {
                reconnect();
                return null;
            }
        }
        ).execute();
    }
    private void reconnect() {
        if (mSocket != null) {
            try {
                mSocket.close();
            } catch (IOException e) {
            }
        }

        try {
            mSocket = new Socket("localhost", 8888);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    @SuppressLint("DefaultLocale")
    void setGpio(int gpio, int value){
        byte msg[] = new byte[2];
        if(value > 0) {
            pwnValue = value;
            value = 1;
        }
        msg[0] = (byte)gpio;
        msg[1] = (byte)value;
        try {
            mSocket.getOutputStream().write(msg);
        } catch (IOException e) {
            e.printStackTrace();
            reconnect();
        }
    }
    @Override
    public void setMotorActions(int motor1, int motor1Rev, int motor2, int motor2Rev) {
        if (gpioMotor1 == 0) {
            return;
        }
        setGpio(gpioMotor1, motor1);
        setGpio(gpioMotor1Rev, motor1Rev);
        setGpio(gpioMotor2, motor2);
        setGpio(gpioMotor2Rev, motor2Rev);
    }
}
