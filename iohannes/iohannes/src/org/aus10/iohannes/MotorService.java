package org.aus10.iohannes;


import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;
import ioio.lib.api.DigitalOutput;
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

    public void onCreate() {
        super.onCreate();
        ioioThread.start();
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

        private boolean motorsForward = false;
        private boolean motorsReverse = false;
        private boolean motorsLeft = false;
        private boolean motorsRight = false;

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
                    PwmOutput motorsM1A = ioio_.openPwmOutput(MOTORS_M1A_PIN, 30);
                    PwmOutput motorsM1B = ioio_.openPwmOutput(MOTORS_M1B_PIN, 30);
                    PwmOutput motorsM2A = ioio_.openPwmOutput(MOTORS_M2A_PIN, 30);
                    PwmOutput motorsM2B = ioio_.openPwmOutput(MOTORS_M2B_PIN, 30);

                    while (!abort_) {
                        motorsForward = true;
                        controlMotors(.5f, motorsM1A, motorsM1B, motorsM2A, motorsM2B);
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

        /**
         * State      A  B
         * FWD        0  1
         * REV        1  0
         * STOP       0  0
         * short      1  1
         * circuit!
         */
        private void controlMotors(final float speed,
                                   PwmOutput motorsM1A,
                                   PwmOutput motorsM1B,
                                   PwmOutput motorsM2A,
                                   PwmOutput motorsM2B) throws ConnectionLostException {
            if (motorsForward) {
                motorsM1A.setDutyCycle(0);
                motorsM1B.setDutyCycle(speed);
                motorsM2A.setDutyCycle(0);
                motorsM2B.setDutyCycle(speed);
            } else if (motorsReverse) {
                motorsM1A.setDutyCycle(speed);
                motorsM1B.setDutyCycle(0);
                motorsM2A.setDutyCycle(speed);
                motorsM2B.setDutyCycle(0);
            } else if (motorsLeft) {
                motorsM1A.setDutyCycle(speed);
                motorsM1B.setDutyCycle(0);
                motorsM2A.setDutyCycle(0);
                motorsM2B.setDutyCycle(speed);
            } else if (motorsRight) {
                motorsM1A.setDutyCycle(0);
                motorsM1B.setDutyCycle(speed);
                motorsM2A.setDutyCycle(speed);
                motorsM2B.setDutyCycle(0);
            } else {
                motorsM1A.setDutyCycle(0);
                motorsM1B.setDutyCycle(0);
                motorsM2A.setDutyCycle(0);
                motorsM2B.setDutyCycle(0);
            }
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
