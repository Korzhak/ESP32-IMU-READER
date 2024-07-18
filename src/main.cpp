#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "quaternion.h"

using namespace imu;

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

uint32_t lastSendTime = 0;

double deltaX, deltaY, deltaZ;
double angleX, angleY, angleZ;

Quaternion orientation, qDeviation;
Vector<3> eulerAngles;

void setup()
{
	// Init Serial Monitor
	Serial.begin(115200);

	if (!mpu.begin())
	{
		Serial.println("Failed to find MPU6050 chip");
		while (1)
		{
			delay(10);
		}
	}

	mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
	mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
	delay(100);

	// Розрахуємо відхилення за допомогою знаходження середнього значення (500 ітерацій)
	for (size_t i = 0; i < 500; i++)
	{
		// Читаємо дані з давача
		mpu.getEvent(&a, &g, &temp);
		deltaX = deltaX + g.gyro.x; //
		deltaY = deltaY + g.gyro.y; // сумуємо для кожної з осей значення від давачів
		deltaZ = deltaZ + g.gyro.z; //
	}
	
	deltaX = deltaX / 500; //
	deltaY = deltaY / 500; // Ділимо кожне значення на кількість ітерацій
	deltaZ = deltaZ / 500; // і знаходимо середнє
}

void loop()
{	
	if((millis() - lastSendTime) >= 10) {
	  // Отримуємо дані від давача
		mpu.getEvent(&a, &g, &temp);

		// Створення кватерніона з кутових швидкостей (віднімаємо біаси)
		Quaternion qW(0, g.gyro.x - deltaX, g.gyro.y - deltaY, g.gyro.z - deltaZ);

		// Розраховуємо диференційне рівнняння орієнтації
		//       1
		// q' = --- * q * w
		//       2 
		// q - кватерніон орієнтації
		// w - кватерніон кутових швидкостей
		qDeviation = orientation * qW * 0.5;


		// Інтегруємо диференційне рівняння та отримуємо нову орієнтацію
		orientation = orientation + qDeviation * 0.01;
		//                 ^             ^        ^
        //             попередня     диференц.  проміжок
		//            орієнтація     рівняння    часу


		// Нормалізуємо кватерніон (довжина вектора кватерніона має бути рівна 1)
		// q = sqrt(w^2 + x^2 + y^2 + z^2)
		orientation.normalize();
		

		// Перетворення кватерніона в кути Ейлера
		eulerAngles = orientation.toEuler();

		
		Serial.print(orientation.w()); //
		Serial.print("\t"); //
		Serial.print(orientation.x()); // Виводимо значення кватерніона в послідовний порт
		Serial.print("\t"); //
		Serial.print(orientation.y()); //
		Serial.print("\t"); //
		Serial.print(orientation.z()); //

		Serial.print("\t\t"); //
		Serial.print(eulerAngles.x()); // Виводимо значення кутів Ейлера в послідовний порт
		Serial.print("\t"); //
		Serial.print(eulerAngles.y()); //
		Serial.print("\t"); //
		Serial.print(eulerAngles.z()); //
		Serial.println();     //
		
		lastSendTime = millis();
	}
}