#include "hFramework.h"
#include "wheel.h"
#include "Adafruit_LSM9DS1.h"
#include <stddef.h>
#include <stdio.h>
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/JointState.h"
#include <math.h>

using namespace hFramework;

const float robotWidth = 0.4;
const float wheelRadius = 0.06;
const float encoderResolution = 1632.67;

ros::NodeHandle nh;
sensor_msgs::BatteryState battery;
ros::Publisher *battery_pub;
std_msgs::Float32 leftPid;
ros::Publisher *leftPid_pub;
std_msgs::Float32 rightPid;
ros::Publisher *rightPid_pub;
sensor_msgs::Imu imuMsg;
ros::Publisher *imu_pub;
geometry_msgs::PoseStamped pose;
ros::Publisher *pose_pub;
geometry_msgs::TransformStamped robot_tf;
tf::TransformBroadcaster broadcaster;

Wheel *leftWheel = new Wheel(hMotD, 0, 8000);
Wheel *rightWheel = new Wheel(hMotC, 0, 8000);

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t a, m, g, temp;
float quater[4] = {0,0,0,0};

bool batteryLow = false;
bool batteryCritical = false;
bool motorStalled = false;
int motorStalledCounter = 0;


void imuUpdater() {
	bool imuOk = true;
	uint32_t t = sys.getRefTime();
	long dt = 50;
	float SamplePeriod = 0.01;
	float Beta = 1;

	hSens1.selectI2C();
	if(!lsm.begin()) {
		printf("IMU not connected\n");
		imuOk = false;
		return;
	}
	printf("IMU connected\n");
	lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
	lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
	lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
	//filter.begin(100);

	while (imuOk) {
		lsm.read();
		lsm.getEvent(&a, &m, &g, &temp); 

		// Invert the Y axis to get the system right handed
		a.acceleration.y = -a.acceleration.y;
		g.gyro.y = -g.gyro.y;
		m.magnetic.y = -m.magnetic.y;

		// Convert gyro values from deg/s to rad/s 
		g.gyro.x *= 3.14159f / 180.0f;
		g.gyro.y *= 3.14159f / 180.0f;
		g.gyro.z *= 3.14159f / 180.0f;

		// Magdwick algorithm
		// https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU/blob/master/x-IMU%20IMU%20and%20AHRS%20Algorithms/x-IMU%20IMU%20and%20AHRS%20Algorithms/AHRS/MadgwickAHRS.cs

		float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
		float gx = g.gyro.x, gy = g.gyro.y, gz = g.gyro.z;
		float mx = m.magnetic.x, my = m.magnetic.y, mz = m.magnetic.z;
		float q1 = quater[0], q2 = quater[1], q3 = quater[2], q4 = quater[3];   // short name local variable for readability
		float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx;
		float _2q1my;
		float _2q1mz;
		float _2q2mx;
		float _4bx;
		float _4bz;
		float _2q1 = 2 * q1;
		float _2q2 = 2 * q2;
		float _2q3 = 2 * q3;
		float _2q4 = 2 * q4;
		float _2q1q3 = 2 * q1 * q3;
		float _2q3q4 = 2 * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = (float)sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0) return; // handle NaN
		norm = 1 / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = (float)sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0) return; // handle NaN
		norm = 1 / norm;        // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		_2q1mx = 2 * q1 * mx;
		_2q1my = 2 * q1 * my;
		_2q1mz = 2 * q1 * mz;
		_2q2mx = 2 * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = (float)sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2 * _2bx;
		_4bz = 2 * _2bz;

		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = 1 / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;

		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

		// Integrate to yield quaternion
		q1 += qDot1 * SamplePeriod;
		q2 += qDot2 * SamplePeriod;
		q3 += qDot3 * SamplePeriod;
		q4 += qDot4 * SamplePeriod;
		norm = 1 / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		quater[0] = q1 * norm;
		quater[1] = q2 * norm;
		quater[2] = q3 * norm;
		quater[3] = q4 * norm;

		// filter.update(
		// 	g.gyro.x,
		// 	g.gyro.y,
		// 	g.gyro.z,
		// 	a.acceleration.x,
		// 	a.acceleration.y,
		// 	a.acceleration.z,
		// 	m.magnetic.x,
		// 	m.magnetic.y,
		// 	m.magnetic.z
		// );
		// filter.getQuaternions(quater);

		sys.delaySync(t, dt);
	}
}

void wheelOdomUpdater() {
	uint32_t t = sys.getRefTime();
	long dt = 10;

	float leftAngularPosition = 0, rightAngularPosition = 0;
	float leftAngularVelocity = 0, rightAngularVelocity = 0;
	float angularPosition = 0, angularVelocity = 0;
	float xPosition = 0, xVelocity = 0;
	float yPosition = 0, yVelocity = 0;

	while(1) {
		int32_t leftEncoder = leftWheel->getDistance();
		int32_t rightEncoder = rightWheel->getDistance();

		leftAngularVelocity = ((2 * 3.14 * leftEncoder / encoderResolution) - leftAngularPosition) / dt;
        rightAngularVelocity = ((2 * 3.14 * rightEncoder / encoderResolution) - rightAngularPosition) / dt;

        leftAngularPosition = 2 * 3.14 * leftEncoder / encoderResolution;
        rightAngularPosition = 2 * 3.14 * rightEncoder / encoderResolution;

		angularVelocity = (((rightAngularPosition - leftAngularPosition) * wheelRadius / robotWidth) - angularPosition) / dt;
        angularPosition = (rightAngularPosition - leftAngularPosition) * wheelRadius / robotWidth;
        xVelocity = (leftAngularVelocity * wheelRadius + angularVelocity * robotWidth / 2) * cos(angularPosition);
        yVelocity = (leftAngularVelocity * wheelRadius + angularVelocity * robotWidth / 2) * sin(angularPosition);
        xPosition = xPosition + xVelocity * dt;
        yPosition = yPosition + yVelocity * dt;

		pose.pose.position.x = xPosition;
		pose.pose.position.y = yPosition;
		pose.pose.orientation = tf::createQuaternionFromYaw(angularPosition);

		sys.delaySync(t, dt);
	}
}

void wheelUpdater() {
    uint32_t t = sys.getRefTime();
    long dt = 10;
    for (;;)
    {
        leftWheel->update(dt);
		rightWheel->update(dt);
        sys.delaySync(t, dt);
    }
}

void batteryMonitor() {
	uint32_t t = sys.getRefTime();
	int dt = 250;
    int batteryLowCounter = 0;
	int batteryCriticalCounter = 0;

    while (1) {
        if(sys.getSupplyVoltage() > 10) batteryLowCounter--;
        else batteryLowCounter++;

        if(batteryLowCounter > 50){
            batteryLow = true;
            batteryLowCounter = 50;
        }
        if(batteryLowCounter < -50) {
            batteryLow = false;
            batteryLowCounter = -50;
        }

		if(sys.getSupplyVoltage() > 9.6) batteryCriticalCounter--;
		else batteryCriticalCounter++;

		if(batteryCriticalCounter > 50) {
			batteryCritical = true;
			batteryCritical = 50;
		}
		if(batteryCriticalCounter < 0) {
			batteryCriticalCounter = 0;
		}
        sys.delaySync(t, dt);
    }
}

void setSpeed(float targetLinearSpeed, float targetAngularSpeed) {
	float leftWheelLinearSpeed = targetLinearSpeed - (targetAngularSpeed * robotWidth / 2);
	float rightWheelLinearSpeed = targetLinearSpeed + (targetAngularSpeed * robotWidth / 2);
	float leftWheelAngularVelocity = leftWheelLinearSpeed / wheelRadius;
	float rightWheelAngularVelocity = rightWheelLinearSpeed / wheelRadius;
	float leftEncoderSpeed = encoderResolution * leftWheelAngularVelocity / (2 * M_PI);
	float rightEncoderSpeed = encoderResolution * rightWheelAngularVelocity / (2 * M_PI);


	leftWheel->setSpeed(leftEncoderSpeed);
	rightWheel->setSpeed(rightEncoderSpeed);
}

void cmdVelCallback(const geometry_msgs::Twist &twist) {
	if(!twist.linear.x && !twist.angular.z) {
		leftWheel->turnOn();
		rightWheel->turnOn();
		motorStalledCounter = 0;
		motorStalled = false;
	}
	if(motorStalled) setSpeed(0,0);
	else setSpeed(twist.linear.x, twist.angular.z);
}

void initCmdVelSubscriber() {
   ros::Subscriber<geometry_msgs::Twist> *cmd_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", &cmdVelCallback);
   nh.subscribe(*cmd_sub);
}

void initBatteryPublisher() {
   battery_pub = new ros::Publisher("/battery", &battery);
   nh.advertise(*battery_pub);
}

void initImuPublisher() {
	imuMsg.header.frame_id = "odom"; //odom
	imu_pub = new ros::Publisher("/imu", &imuMsg);
	nh.advertise(*imu_pub);
}

void initPosePublisher() {
	pose.header.frame_id = "odom"; //odom
	pose.pose.orientation = tf::createQuaternionFromYaw(0);
	pose_pub = new ros::Publisher("/wheel_pose", &pose);
	nh.advertise(*pose_pub);
}

void initTfPublisher() {
	robot_tf.header.frame_id = "odom"; //frame=odom
	robot_tf.child_frame_id = "base_link";
	robot_tf.transform.translation.x = 0.0;
	robot_tf.transform.translation.y = 0.0;
	robot_tf.transform.translation.z = 0.0;
	robot_tf.transform.rotation.x = 0.0;
	robot_tf.transform.rotation.y = 0.0;
	robot_tf.transform.rotation.z = 0.0;
	robot_tf.transform.rotation.w = 1.0;
	broadcaster.init(nh);
}


void hMain()
{
	RPi.init(230400);
	nh.getHardware()->initWithDevice(&RPi);
    nh.initNode();
	initCmdVelSubscriber();
	initBatteryPublisher();
	initImuPublisher();
	initTfPublisher();
	initPosePublisher();

	leftWheel->begin();
	rightWheel->begin();
	leftWheel->reset();
	rightWheel->reset();
	sys.taskCreate(std::bind(&wheelUpdater));

	sys.taskCreate(std::bind(&imuUpdater));
	sys.taskCreate(std::bind(&wheelOdomUpdater));
	sys.taskCreate(std::bind(&batteryMonitor));

	int publishCounter = 0;
	int serialCounter = 0;
	int ledCounter = 0;

	while(1) {
		nh.spinOnce();

		if(publishCounter > 2) {
			// Battery publisher
			battery.voltage = sys.getSupplyVoltage();
			battery_pub->publish(&battery);

			// IMU publisher
			imuMsg.linear_acceleration.x = a.acceleration.x;
			imuMsg.linear_acceleration.y = a.acceleration.y;
			imuMsg.linear_acceleration.z = a.acceleration.z;
			imuMsg.angular_velocity.x = g.gyro.x;
			imuMsg.angular_velocity.y = g.gyro.y;
			imuMsg.angular_velocity.z = g.gyro.z;
			imuMsg.orientation.x = quater[1];
			imuMsg.orientation.y = quater[2];
			imuMsg.orientation.z = quater[3];
			imuMsg.orientation.w = quater[0];
			imu_pub->publish(&imuMsg);

			Wheel pose publisher
			pose_pub->publish(&pose);

			// Tf publisher
			robot_tf.header.stamp = nh.now();
			robot_tf.transform.translation.x = pose.pose.position.x;
			robot_tf.transform.translation.y = pose.pose.position.y;
			robot_tf.transform.rotation.x = pose.pose.orientation.x;
			robot_tf.transform.rotation.y = pose.pose.orientation.y;
			robot_tf.transform.rotation.z = pose.pose.orientation.z;
			robot_tf.transform.rotation.w = pose.pose.orientation.w;
         	// publish tf
         	broadcaster.sendTransform(robot_tf);

			publishCounter = 0;
		}	

		if(ledCounter > 25) {
			if(batteryLow) LED2.toggle();
			else LED2.off();

			if(batteryCritical) LED1.toggle();

			if(motorStalled) LED3.on();
			else LED3.off();

			ledCounter = 0;
		}

		// Check if the battery level is critical
		if(batteryCritical) {
			leftWheel->turnOff();
			rightWheel->turnOff();
		}

		// Check if a motor is stalled
		// if((leftWheel->getPidOut() && !leftWheel->getSpeed()) || (rightWheel->getPidOut() && !rightWheel->getSpeed())){
		// 	motorStalledCounter++;
		// 	if(motorStalledCounter > 50){
		// 		leftWheel->turnOff();
		// 		rightWheel->turnOff();
		// 		// leftWheel->reset();
		// 		// rightWheel->reset();
		// 		motorStalledCounter = 50;
		// 		motorStalled = true;
		// 	}
		// }

		publishCounter++;
		ledCounter++;
		serialCounter++;
		sys.delay(10);
	}
}
