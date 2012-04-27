package org.aus10.iohannes;

import android.app.Application;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;

public class IohannesApp extends Application {
    private MotorService motorService_;

    private ServiceConnection serviceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName componentName, IBinder iBinder) {
            motorService_ = ((MotorService.MotorServiceBinder) iBinder).getMotorService();
        }

        public void onServiceDisconnected(ComponentName componentName) { }
    };

    public void onCreate() {
        super.onCreate();
        startService(new Intent(this, MotorService.class));
        bindService(new Intent(this, MotorService.class), serviceConnection, BIND_AUTO_CREATE);
    }
}
