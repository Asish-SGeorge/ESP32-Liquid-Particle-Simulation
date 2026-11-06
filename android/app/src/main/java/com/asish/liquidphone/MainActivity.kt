package com.asish.liquidphone

import android.annotation.SuppressLint
import android.app.Activity
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.view.Surface
import android.view.WindowManager
import android.webkit.JavascriptInterface
import android.webkit.WebChromeClient
import android.webkit.WebView
import android.webkit.WebViewClient
import java.util.Locale
import kotlin.math.atan2
import kotlin.math.hypot

class MainActivity : Activity(), SensorEventListener {
    private lateinit var webView: WebView
    private lateinit var sensorManager: SensorManager
    private var rotationSensor: Sensor? = null
    private var sensorRunning = false
    private var motionRequested = false
    private val rotationMatrix = FloatArray(9)
    private val remappedMatrix = FloatArray(9)

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        // Game rotation vector is fused from gyro + accelerometer and does not
        // need compass north. Fall back to the standard rotation vector.
        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        webView = WebView(this)
        webView.setBackgroundColor(android.graphics.Color.BLACK)
        webView.settings.javaScriptEnabled = true
        webView.settings.domStorageEnabled = true
        webView.settings.allowFileAccess = false
        webView.settings.allowContentAccess = false
        webView.settings.javaScriptCanOpenWindowsAutomatically = false
        webView.webViewClient = object : WebViewClient() {
            override fun onPageFinished(view: WebView?, url: String?) {
                super.onPageFinished(view, url)
                if (rotationSensor == null) {
                    sendStatus("This Android phone does not expose a rotation-vector sensor.", true)
                } else {
                    sendStatus("Android companion connected. Hold the phone upright, then enable left/right tilt.", false)
                }
            }
        }
        webView.webChromeClient = WebChromeClient()
        webView.addJavascriptInterface(NativeMotionBridge(), "AndroidMotionBridge")
        setContentView(webView)
        webView.loadUrl("http://192.168.4.1/")
    }

    inner class NativeMotionBridge {
        @JavascriptInterface
        fun startMotion() {
            runOnUiThread {
                val sensor = rotationSensor
                if (sensor == null) {
                    sendStatus("No compatible rotation-vector sensor was found on this phone.", true)
                    return@runOnUiThread
                }
                motionRequested = true
                registerMotionSensor()
                if (sensorRunning) {
                    sendStatus("Android one-axis motion sensor started. Hold phone upright and tap CALIBRATE PHONE.", false)
                } else {
                    sendStatus("Android could not start the motion sensor.", true)
                }
            }
        }

        @JavascriptInterface
        fun stopMotion() {
            runOnUiThread { stopSensors() }
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (!sensorRunning || event.sensor.type != Sensor.TYPE_GAME_ROTATION_VECTOR &&
            event.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return

        SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
        val displayRotation = if (android.os.Build.VERSION.SDK_INT >= 30) {
            display?.rotation ?: Surface.ROTATION_0
        } else {
            @Suppress("DEPRECATION")
            (windowManager.defaultDisplay.rotation)
        }

        // Remap the sensor coordinate frame so +X is screen-right and +Y is
        // screen-up, including when Android rotates the display.
        val axes = when (displayRotation) {
            Surface.ROTATION_90 -> Pair(SensorManager.AXIS_Y, SensorManager.AXIS_MINUS_X)
            Surface.ROTATION_180 -> Pair(SensorManager.AXIS_MINUS_X, SensorManager.AXIS_MINUS_Y)
            Surface.ROTATION_270 -> Pair(SensorManager.AXIS_MINUS_Y, SensorManager.AXIS_X)
            else -> Pair(SensorManager.AXIS_X, SensorManager.AXIS_Y)
        }
        SensorManager.remapCoordinateSystem(rotationMatrix, axes.first, axes.second, remappedMatrix)

        // The rotation matrix maps device axes into world axes. Its third row
        // is the world's up direction expressed in the remapped device frame.
        // In screen coordinates, gravity-down is (-upX, +upY). Project it
        // into the portrait display plane, then reduce it to one left/right
        // tilt angle. The perpendicular tilt component is not used.
        val upX = remappedMatrix[6].toDouble()
        val upY = remappedMatrix[7].toDouble()
        var gravityX = -upX
        var gravityY = upY
        val magnitude = hypot(gravityX, gravityY)
        if (magnitude < 0.10) {
            runOnUiThread {
                sendStatus("Hold the phone vertically so gravity lies in the screen plane.", true)
            }
            return
        }
        gravityX /= magnitude
        gravityY /= magnitude
        if (!gravityX.isFinite() || !gravityY.isFinite()) return

        // Convert the in-plane gravity projection into ONE left/right tilt angle.
        // The web simulation reconstructs its 2D gravity vector from this angle;
        // no independent forward/backward axis is transmitted.
        val tiltDegrees = Math.toDegrees(atan2(gravityX, gravityY))
        if (!tiltDegrees.isFinite()) return
        val js = String.format(Locale.US, "window.receiveNativeTilt(%.4f);", tiltDegrees)
        runOnUiThread { webView.evaluateJavascript(js, null) }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun sendStatus(message: String, warning: Boolean) {
        val escaped = message.replace("\\", "\\\\").replace("'", "\\'").replace("\n", " ")
        webView.evaluateJavascript("window.receiveNativeMotionStatus('$escaped',$warning);", null)
    }

    private fun registerMotionSensor() {
        val sensor = rotationSensor ?: return
        if (!sensorRunning && motionRequested) {
            sensorRunning = sensorManager.registerListener(
                this,
                sensor,
                SensorManager.SENSOR_DELAY_GAME
            )
        }
    }

    private fun stopSensors(clearRequest: Boolean = false) {
        if (clearRequest) motionRequested = false
        if (sensorRunning) {
            sensorManager.unregisterListener(this)
            sensorRunning = false
        }
    }

    override fun onPause() {
        // Pause callbacks in the background, then resume automatically when
        // the user returns to the app if motion had been enabled.
        stopSensors(clearRequest = false)
        super.onPause()
    }

    override fun onResume() {
        super.onResume()
        registerMotionSensor()
    }

    @Deprecated("Deprecated in Android")
    override fun onBackPressed() {
        if (webView.canGoBack()) webView.goBack() else super.onBackPressed()
    }

    override fun onDestroy() {
        stopSensors(clearRequest = true)
        webView.removeJavascriptInterface("AndroidMotionBridge")
        webView.destroy()
        super.onDestroy()
    }
}
