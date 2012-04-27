package org.aus10.iohannes;


import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;
import ioio.lib.api.IOIO;
import ioio.lib.api.IOIOFactory;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;

/**
 * This is the thread that does the IOIO interaction.
 * <p/>
 * It first creates a IOIO instance and wait for a connection to be
 * established. Then it starts doing the main work of opening the LED pin
 * and constantly updating it to match the toggle button's state.
 * <p/>
 * Whenever a connection drops, it tries to reconnect, unless this is a
 * result of abort().
 */
public class MotorService extends Service {

    private final IOIOThread ioioThread = new IOIOThread();

    public static final String INTENT_MOTORS = "motorControl";
    public static final String INTENT_PARAM_MOTORS = "manualDirection";

    public enum MotorControl {
        FORWARD, REVERSE, LEFT, RIGHT, STOP;
    }

    private MotorControl currentDirection;

    private final IntentFilter intentFilter = new IntentFilter(INTENT_MOTORS);
    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            MotorControl direction = (MotorControl) intent.getSerializableExtra(INTENT_PARAM_MOTORS);
            currentDirection = direction;
        }
    };

    public static Intent createIntentToMoveMotor(MotorService.MotorControl direction) {
        return new Intent(MotorService.INTENT_MOTORS).putExtra(MotorService.INTENT_PARAM_MOTORS, direction);
    }

    public void onCreate() {
        super.onCreate();
        currentDirection = MotorControl.STOP;
        registerReceiver(broadcastReceiver, intentFilter);
        ioioThread.start();
    }

    public void onDestroy() {
        super.onDestroy();
        unregisterReceiver(broadcastReceiver);
    }

    private class IOIOThread extends Thread {

        /**
         * The pins we're using on the board.
         */
        private static final int MOTORS_M1A_PIN = 3;
        private static final int MOTORS_M1B_PIN = 4;
        private static final int MOTORS_M2A_PIN = 5;
        private static final int MOTORS_M2B_PIN = 6;
        private IOIO ioio_;
        private boolean abort_ = false;
        private PwmOutput motorsM1A;
        private PwmOutput motorsM1B;
        private PwmOutput motorsM2A;
        private PwmOutput motorsM2B;

        public void run() {
            while (true) {
                synchronized (this) {
                    if (abort_) {
                        break;
                    }
                    ioio_ = IOIOFactory.create();
                }
                try {

                    notifyActivityWithMessage("IOIO Connecting...");
                    ioio_.waitForConnect();
                    notifyActivityWithMessage("IOIO Connected");
                    motorsM1A = ioio_.openPwmOutput(MOTORS_M1A_PIN, 30);
                    motorsM1B = ioio_.openPwmOutput(MOTORS_M1B_PIN, 30);
                    motorsM2A = ioio_.openPwmOutput(MOTORS_M2A_PIN, 30);
                    motorsM2B = ioio_.openPwmOutput(MOTORS_M2B_PIN, 30);

                    while (!abort_) {
                        controlMotors(.5f);
                        Thread.sleep(10);
                    }

                } catch (ConnectionLostException e) {
                } catch (Exception e) {
                    Log.e("HelloIOIOPower", "Unexpected exception caught", e);
                    ioio_.disconnect();
                    notifyActivityWithMessage("IOIO Disconnected... due to " + e);
                    break;
                } finally {
                    try {
                        ioio_.waitForDisconnect();
                        notifyActivityWithMessage("IOIO Disconnected... Reconnecting...");
                    } catch (InterruptedException e) {
                    }
                }
            }
        }

        private void controlMotors(final float speed) throws ConnectionLostException {
            switch (currentDirection) {
                case FORWARD:
                    controlMotor(0, speed, 0, speed);
                    break;
                case REVERSE:
                    controlMotor(speed, 0, speed, 0);
                    break;
                case LEFT:
                    controlMotor(speed, 0, 0, speed);
                    break;
                case RIGHT:
                    controlMotor(0, speed, speed, 0);
                    break;
                default:
                    controlMotor(0, 0, 0, 0);
                    break;
            }
        }

        /**
         * State      A  B
         * FWD        0  1
         * REV        1  0
         * STOP       0  0
         * short      1  1
         * circuit!
         */
        private void controlMotor(float m1a, float m1b, float m2a, float m2b) throws ConnectionLostException {
            motorsM1A.setDutyCycle(m1a);
            motorsM1B.setDutyCycle(m1b);
            motorsM2A.setDutyCycle(m2a);
            motorsM2B.setDutyCycle(m2b);
        }

        /**
         * Abort the connection.
         * <p/>
         * This is a little tricky synchronization-wise: we need to be handle
         * the case of abortion happening before the IOIO instance is created or
         * during its creation.
         */
        synchronized public void abort() {
            abort_ = true;
            if (ioio_ != null) {
                ioio_.disconnect();
            }
        }
    }

    private void notifyActivityWithMessage(final String message) {
        final Intent ioioConnected = new Intent(MainActivity.INTENT_CONNECTED).putExtra(MainActivity.INTENT_PARAM_CONNECTED, message);
        sendBroadcast(ioioConnected);
    }

    private final IBinder binder = new MotorServiceBinder();

    public IBinder onBind(Intent arg0) {
        return binder;
    }

    public class MotorServiceBinder extends Binder {
        public MotorService getMotorService() {
            return MotorService.this;
        }
    }

}
