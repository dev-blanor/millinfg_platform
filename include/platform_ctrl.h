#include <Arduino.h>
#include <Wire.h>

// Serial2 - для соединения по bluetooth
// Serial1 - для соединения с УСЛП

#define NUM_DRIVERS 3 //кол-во драйверов

#define DRIVER1_I2C_ADDR 11 // адрес 1-го драйвера на шине i2c
#define DRIVER2_I2C_ADDR 12 // адрес 2-го драйвера на шине i2c
#define DRIVER3_I2C_ADDR 13 // адрес 3-го драйвера на шине i2c

const int DRIVERS_I2C_ADDR[] = {11, 12, 13}; // массив адресов
float data_for_driver[NUM_DRIVERS];          // массив данных для отправки драйверам (скорости вращения колёс рад/с)

// Информация от УСЛП // (*пока только координаты в ЛСК УСЛП)
float theta;                           // угол поворота робота (угол поворота ЛСК по отношению к БСК), радианы
float position[2] = {123.4f, -456.5f}; // координаты центра робота, мм (X Y)
float posTR[2];                        // координаты правого излучателя
float posTL[2];                        // координаты левого излучателя

// КИНЕМАТИКА //
float Uc[3];                   // вектор локальных скоростей (Vx Vy theta)
const float K_matrix[3][3] = { // матрица кинематики
    {33.333333333333336f, -33.333333333333330f, -2.745190528383288f},
    {12.200846792814621f, 45.534180126147960f, -2.745190528383290f},
    {-12.200846792814621f, 45.534180126147960f, -10.245190528383290f}};
float fi_vel[3]; // вектор угловых скоростей, рад/с

// Прототипы функций //
void TransiveFloat_i2c(float x);
void TransiveFloat_i2c_CS(float x);
void send_data_to_drivers();
void clear_buffer();
void inv_kin_local_maket();

//////////////
// FUNCTION //
//////////////

// расчёт угловых скоростей, рад/с
// по формуле K_matrix * Ur
void inv_kin_local_maket()
{
    for (int i = 0; i < 3; i++)
    {
        fi_vel[i] = K_matrix[i][0] * Uc[0] + K_matrix[i][1] * Uc[1] + K_matrix[i][2] * Uc[2];
    }
}

// отправка данных драйверам по i2c
void send_data_to_drivers()
{
    for (int i = 0; i < NUM_DRIVERS; i++)
    {
        Wire.beginTransmission(DRIVERS_I2C_ADDR[i]); // transmit to device #8
        TransiveFloat_i2c_CS(data_for_driver[i]);
        Wire.endTransmission(); // stop transmitting
    }
}

// передача числа типа float по байтно по интерфейсу i2c
void TransiveFloat_i2c(float x)
{
    char float_bytes[sizeof(float)];
    memcpy(float_bytes, &x, sizeof(float)); // разбивка на байты

    for (uint8_t j = 0; j < sizeof(float); j++) // отправка
    {
        Wire.write(float_bytes[j]);
    }
}

// + контрольная сумма
// Драйверу отправляется 5 байт: 4 байта числа float и 1 байт контр. суммы.
// [byte1 ... byte4 CS]
void TransiveFloat_i2c_CS(float x)
{
    char float_bytes[sizeof(float)];
    uint16_t check_sum_sum = 0; // для суммы байт
    uint8_t check_sum;          // для контрольной суммы

    memcpy(float_bytes, &x, sizeof(float)); // разбивка на байты

    for (uint8_t j = 0; j < sizeof(float); j++) // отправка
    {
        Wire.write(float_bytes[j]);
        check_sum_sum += float_bytes[j];
    }
    check_sum = (uint8_t)(check_sum_sum & 0xFF); // взятие младшего байта
    //Serial2.println(check_sum);
    Wire.write(check_sum); // отправка последнего байта контрольной суммы
}

// очистка буфера Serial0
void clear_buffer()
{
    for (int i = 0; i < Serial2.available(); i++)
    {
        Serial.read();
    }
}