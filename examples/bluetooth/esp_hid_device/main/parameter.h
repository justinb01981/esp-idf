
#define CONFIG_GPIO_SDA 17
#define CONFIG_GPIO_SCL 18
#define PIN_LEFTMOUSE 5 /*d2*/
#define PIN_RIGHTMOUSE 7 /*d4*/

#define IMU_POLL_INTERVAL_MS 1
#define IMU_ROTATE_MULTIPLIER 3.5f


#define CONFIG_I2C_ADDR 0x6A // HACK should come from project config

typedef struct {
	uint16_t port;
	char ipv4[20]; // xxx.xxx.xxx.xxx
} PARAMETER_t;


typedef struct {
	float quatx;
	float quaty;
	float quatz;
	float quatw;
	float roll;
	float pitch;
	float yaw;
    float aX, aY, aZ;
} POSE_t;

