package org.aus10.iohannes;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.ToggleButton;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.IOIOFactory;
import ioio.lib.api.exception.ConnectionLostException;

/**
 * This is the main activity of the iohannes application.
 */
public class MainActivity extends Activity {
    /**
     * The text displayed at the top of the page.
     */
    private TextView title_;
    /**
     * Our buttons.
     */
    private ToggleButton status_button_;
    private ToggleButton motors_forward_button_;
    private ToggleButton motors_reverse_button_;
    private ToggleButton motors_left_button_;
    private ToggleButton motors_right_button_;
    /**
     * The thread that interacts with the IOIO.
     */
    private IOIOThread ioio_thread_;
    /**
     * The pins we're using on the board.
     */
    private static final int STATUS_LED_PIN = 0;
    private static final int MOTORS_M1A_PIN = 3;
    private static final int MOTORS_M1B_PIN = 4;
    private static final int MOTORS_M2A_PIN = 5;
    private static final int MOTORS_M2B_PIN = 6;

    /**
     * Called when the activity is first created. Here we normally initialize
     * our GUI.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        title_ = (TextView) findViewById(R.id.title);
        status_button_ = (ToggleButton) findViewById(R.id.status_test);
        motors_forward_button_ = (ToggleButton) findViewById(R.id.motor_forward);
        motors_reverse_button_ = (ToggleButton) findViewById(R.id.motor_backward);
        motors_left_button_ = (ToggleButton) findViewById(R.id.motor_left);
        motors_right_button_ = (ToggleButton) findViewById(R.id.motor_right);
        setupButtonClickListeners();
    }

    /**
     * Called when the application is resumed (also when first started). Here is
     * where we'll create our IOIO thread.
     */
    @Override
    protected void onResume() {
        super.onResume();
        ioio_thread_ = new IOIOThread();
        ioio_thread_.start();
    }

    /**
     * Called when the application is paused. We want to disconnect with the
     * IOIO at this point, as the user is no longer interacting with our
     * application.
     */
    @Override
    protected void onPause() {
        super.onPause();
        ioio_thread_.abort();
        try {
            ioio_thread_.join();
        } catch (InterruptedException e) {
        }
    }

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
    class IOIOThread extends Thread {
        private IOIO ioio_;
        private boolean abort_ = false;

        /**
         * Thread body.
         */
        @Override
        public void run() {
            super.run();
            while (true) {
                synchronized (this) {
                    if (abort_) {
                        break;
                    }
                    ioio_ = IOIOFactory.create();
                }
                try {
                    setText(R.string.wait_ioio);
                    ioio_.waitForConnect();
                    setText(R.string.ioio_connected);
                    DigitalOutput led = ioio_.openDigitalOutput(STATUS_LED_PIN, true);
                    DigitalOutput motorsM1A = ioio_.openDigitalOutput(MOTORS_M1A_PIN, true);
                    DigitalOutput motorsM1B = ioio_.openDigitalOutput(MOTORS_M1B_PIN, true);
                    DigitalOutput motorsM2A = ioio_.openDigitalOutput(MOTORS_M2A_PIN, true);
                    DigitalOutput motorsM2B = ioio_.openDigitalOutput(MOTORS_M2B_PIN, true);
                    while (true) {
                        updateStatusButton(led);
                        controlMotors(motorsM1A, motorsM1B, motorsM2A, motorsM2B);
                        sleep(10);
                    }
                } catch (ConnectionLostException e) {
                } catch (Exception e) {
                    Log.e("HelloIOIOPower", "Unexpected exception caught", e);
                    ioio_.disconnect();
                    break;
                } finally {
                    try {
                        ioio_.waitForDisconnect();
                    } catch (InterruptedException e) {
                    }
                }
            }
        }

        private void updateStatusButton(DigitalOutput led) throws ConnectionLostException {
            led.write(!status_button_.isChecked());
        }

        private void controlMotors(DigitalOutput motorsM1A, DigitalOutput motorsM1B, DigitalOutput motorsM2A, DigitalOutput motorsM2B) throws ConnectionLostException {
            if (motors_forward_button_.isChecked()) {
                motorsM1A.write(true);
                motorsM1B.write(false);
                motorsM2A.write(false);
                motorsM2B.write(true);
            } else if (motors_reverse_button_.isChecked()) {
                motorsM1A.write(false);
                motorsM1B.write(true);
                motorsM2A.write(true);
                motorsM2B.write(false);
            } else if (motors_left_button_.isChecked()) {
                motorsM1A.write(false);
                motorsM1B.write(true);
                motorsM2A.write(false);
                motorsM2B.write(true);
            } else if (motors_right_button_.isChecked()) {
                motorsM1A.write(true);
                motorsM1B.write(false);
                motorsM2A.write(true);
                motorsM2B.write(false);
            } else {
                motorsM1A.write(false);
                motorsM1B.write(false);
                motorsM2A.write(false);
                motorsM2B.write(false);
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

        /**
         * Set the text line on top of the screen.
         *
         * @param id The string ID of the message to present.
         */
        private void setText(final int id) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    title_.setText(getString(id));
                }
            });
        }
    }

    void setupButtonClickListeners() {
        motors_forward_button_.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                motors_reverse_button_.setChecked(false);
                motors_left_button_.setChecked(false);
                motors_right_button_.setChecked(false);
            }
        });
        motors_reverse_button_.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                motors_forward_button_.setChecked(false);
                motors_left_button_.setChecked(false);
                motors_right_button_.setChecked(false);
            }
        });
        motors_left_button_.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                motors_forward_button_.setChecked(false);
                motors_reverse_button_.setChecked(false);
                motors_right_button_.setChecked(false);
            }
        });
        motors_right_button_.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View arg0) {
                motors_forward_button_.setChecked(false);
                motors_reverse_button_.setChecked(false);
                motors_left_button_.setChecked(false);
            }
        });
    }

}
