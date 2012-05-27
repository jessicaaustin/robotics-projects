package org.aus10.iohannes_ros;

import android.app.Application;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;

public class IohannesApp extends Application {
    private IOIOService ioioService;

    private ServiceConnection serviceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName componentName, IBinder iBinder) {
            ioioService = ((IOIOService.MotorServiceBinder) iBinder).getMotorService();
        }

        public void onServiceDisconnected(ComponentName componentName) { }
    };

    public void onCreate() {
        super.onCreate();
        startService(new Intent(this, IOIOService.class));
        bindService(new Intent(this, IOIOService.class), serviceConnection, BIND_AUTO_CREATE);
    }
}
