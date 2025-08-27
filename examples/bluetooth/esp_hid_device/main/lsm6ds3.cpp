#include <cstring>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;


static const char *TAG = "IMU";

// I2Cdev and LSM303DLHC must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "LSM6DS3.h"

LSM6DS3 imu(CONFIG_I2C_ADDR);

// Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533
#define MOUSE_SCALE (100)

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

POSE_t pose; // exposed via .h

void _getMotion6(double *_ax, double *_ay, double *_az, double *_gx, double *_gy, double *_gz) {
	float ax=0.0, ay=0.0, az=0.0;
	float gx=0.0, gy=0.0, gz=0.0;
#if 0
	if (imu.accelerationAvailable()) {
		imu.readAcceleration(ax, ay, az);
	}
	if (imu.gyroscopeAvailable()) {
		imu.readGyroscope(gx, gy, gz);
	}
#endif
	while(1) {
		if (imu.accelerationAvailable()) break;
		vTaskDelay(1);
	}
	imu.readAcceleration(ax, ay, az);
	while(1) {
		if (imu.gyroscopeAvailable()) break;
		vTaskDelay(1);
	}
	imu.readGyroscope(gx, gy, gz);

	*_ax = ax;
	*_ay = ay;
	*_az = az;
	*_gx = gx;
	*_gy = gy;
	*_gz = gz;
}

void getRollPitch(double accX, double accY, double accZ, double *roll, double *pitch) {
	// atan2 outputs the value of - to	(radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	*roll = atan2(accY, accZ) * RAD_TO_DEG;
	*pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	*roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	*pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

extern "C" {

extern void send_mouse(uint8_t buttons, char dx, char dy, char wheel);

void lsm6ds3(void *pvParameters){

    // Initialize device
    if (imu.begin() == 0) {
        ESP_LOGE(TAG, "Connection fail");
        vTaskDelete(NULL);
    }

    double accXl = 0, accYl = 0, accZl = 0; // initial gravity vector
    double grvXl = 0, grvYl = 0, grvZl = 0;
    double worldX = 0, worldY = 0, worldZ = 0;  // accumulated world position

    double accX, accY, accZ;
    double grvX, grvY, grvZ;

    double roll, pitch; // Roll and pitch are calculated using the accelerometer

    _getMotion6(&accX, &accY, &accZ, &grvX, &grvY, &grvZ);

    // Set Kalman and gyro starting angle
    getRollPitch(accX, accY, accZ, &roll, &pitch);

    uint32_t timer = micros();

    bool initialized = false;

    while(1){

        _getMotion6(&accX, &accY, &accZ, &grvX, &grvY, &grvZ);

//      double mX = (accX-accXl)*MOUSE_SCALE, mY = (accY-accYl)*MOUSE_SCALE;

        getRollPitch(accX, accY, accZ, &roll, &pitch);

        // Set the first data
        if (!initialized) {
            initialized = true;
            accXl = accX;
            accYl = accY;
            accZl = accZ;
            grvXl = grvX;
            grvYl = grvY;
            grvZl = grvZ;
        }

        double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
        timer = micros();

        accX *= dt; accY *= dt; accZ *= dt;

        const double M = accX*accX + accY*accY + accZ*accZ;

        // offset and scale by 1/T
        accX = (accX - accXl) / M;
        accY = (accY - accYl) / M;
        accZ = (accZ - accZl) / M;

        const double orientX = grvXl - grvX;
        const double orientY = grvYl - grvY;
        const double orientZ = grvZl - grvZ;

        double Xv = (accX)*orientX + accY*orientX + accZ*orientX;
        double Yv = (-accX)*orientY + accY*orientY + accZ*orientY;
        double Zv = (-accX)*orientZ - accY*orientZ - accZ*orientZ;

        worldX += Xv;
        worldY += Yv;
        worldZ += Zv;
#define COORDFMT "%0.02f"

        printf("Position: "COORDFMT"(X) "COORDFMT"(Y) "COORDFMT"(Z))\n", worldX, worldY, worldZ);
        //printf("Delt: "COORDFMT"(dx)" COORDFMT"(dy) "COORDFMT"(dz)\n",  Xv, Yv, Zv);

        /* Roll and pitch estimation */
        double gyroXrate = grvX;
        double gyroYrate = grvY;

        pose.aX = Xv;
        pose.aY = Yv; // flip Y?
        pose.aZ = Zv;

        vTaskDelay(1);
    }

    // Never reach here
    vTaskDelete(NULL);
}

}
