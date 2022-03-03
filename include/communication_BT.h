/*
    Обеспечение связи с роботом. Протокол описан в тетради.
    Связь осуществляется по bluetooth. Под эти цели выделен порт Serial2 
*/

#include <Arduino.h>

// дескрипторы
#define FI_VEL 'D'         // скорости колёс непосредственно
#define LOC_VECTOR 'U'     // вектор локальных скоростей
#define SET_BEACON_POS 'B' // настройка координат маяков
#define GET_POS 'P'        // обратная связь по положению и ориентации
#define FROM_TERMINAL 't'  // сообщение от терминала
#define FROM_HIGH_LVL 'h'  // сообщение от верхнего уровня (matlab)

#define WAIT_DATA_BT 100 //время ожидания данных, мс

bool flag_data_for_driver = false; // готовность данных для отправки драйверам
bool flag_data_Ur = false;         // данные по вектор скорости обновлены

uint8_t bytes_FI_VEL[13];     // скорости колёс 3*4=12 байт данных и 1 байт контрольной суммы (CS)
uint8_t bytes_LOC_VECTOR[13]; // 12 байт данных и 1 байт CS
uint8_t BB_set_bytes[17];     // коордианты маяков 4*4=16 байт данных и 1 байт контрольной суммы (CS)

void init_communicate_BT();
void chek_data_from_BT();
bool recieve_data_HLVL(char type_data);
void send_data_HLVL();
float convert_bytes2float(uint8_t float_bytes[]);
void convert_FI_VEL();
void convert_LOC_VECTOR();
void convert_BB(float *bb_set);
void clear_bufferBT();

//////////////
// FUNCTION //
//////////////

// инициализация соединения
void init_communicate_BT()
{
    Serial2.begin(38400);
    Serial2.setTimeout(WAIT_DATA_BT);
}

// проверка наличия данных
void chek_data_from_BT()
{
    while (Serial2.available() != 0) // пока есть данные в буфере
    {
        char descriptor = Serial2.read(); //читаем дескриптор
        char refine[1];
        Serial2.readBytes(refine, 1); // читаем последующий байт для уточнения

        bool cs_status; // статус контрольной суммы совпадение/несовпадение

        Serial.print(descriptor);  // на экран
        Serial.println(refine[0]); // на экран

        switch (descriptor)
        {
        case FI_VEL:                        // пришли сорости колёс
            if (refine[0] == FROM_TERMINAL) // если сообщение с терминала -> символьный формат
            {
                flag_data_for_driver = true;
                for (int i = 0; i < NUM_DRIVERS; i++)
                {
                    data_for_driver[i] = Serial2.parseFloat();
                    Serial2.read();
                    //Serial2.println(data_for_driver[i]);
                }
                Serial2.println("OK"); // ответ в терминал
            }
            else if (refine[0] == FROM_HIGH_LVL)
            {
                ////////////////////////////////////////////
                // принимаем байты и проверяем целостность
                cs_status = recieve_data_HLVL(FI_VEL);
                //
                Serial.print("CS_status\t");
                Serial.println(cs_status);

                if (cs_status)
                {
                    convert_FI_VEL();
                    flag_data_for_driver = true;
                }
                //for (uint8_t i = 0; i < sizeof(bytes_FI_VEL); i++)
                //{
                //    Serial.println(bytes_FI_VEL[i]);
                //}
                ///////////////////////////////////////////
            }
            else
                clear_bufferBT();
            break;

        case LOC_VECTOR: // локальный вектор
            if (refine[0] == FROM_TERMINAL)
            {
                flag_data_Ur = true;
                for (int i = 0; i < 3; i++)
                {
                    Uc[i] = Serial2.parseFloat();
                    Serial2.read();
                    Serial2.println(Uc[i]); // эхо
                }
            }
            else if (refine[0] == FROM_HIGH_LVL)
            {
                cs_status = recieve_data_HLVL(LOC_VECTOR);

                Serial.print("CS_status\t");
                Serial.println(cs_status);

                if (cs_status)
                {
                    convert_LOC_VECTOR();
                    flag_data_Ur = true;
                }

                //for (uint8_t i = 0; i < sizeof(bytes_LOC_VECTOR); i++)
                //{
                //    Serial.println(bytes_LOC_VECTOR[i]);
                //}
            }
            else
                clear_bufferBT();
            break;

        case GET_POS:                       // запрос на получение координат
            if (refine[0] == FROM_TERMINAL) // терминала
            {
                // ответ
                Serial2.write(GET_POS);
                Serial2.write(FROM_TERMINAL);
                Serial2.print(position[0]);
                Serial2.print(';');
                Serial2.print(position[1]);
                Serial2.print(';');
                Serial2.print(theta);
                Serial2.println(';');
            }
            else if (refine[0] == FROM_HIGH_LVL) // с matlab
            {
                // ответ
                Serial2.write(GET_POS);
                Serial2.write(FROM_HIGH_LVL);
                send_data_HLVL();
            }
            else
                clear_bufferBT();

            break;
        case SET_BEACON_POS: // команда с верхнего уровня: "Bh"[x1][y1][x2][y2][CS-1 byte] (type float)
            if (refine[0] == FROM_HIGH_LVL)
            {
                //Serial.println('1');
                float BB_coordinate[4];
                if (recieve_data_HLVL(SET_BEACON_POS))
                {
                    convert_BB(BB_coordinate);
                    // отправка на УСЛП
                    Serial1.write(SET_BEACON_POS);
                    Serial1.print(BB_coordinate[0]); // x1
                    Serial1.print(';');
                    Serial1.print(BB_coordinate[1]); // y1
                    Serial1.print(';');
                    Serial1.print(BB_coordinate[2]); // x2
                    Serial1.print(';');
                    Serial1.print(BB_coordinate[3]); // y2
                    Serial1.print(';');
                    //set_BB_coordinates(BB_coordinate);
                }
            }
            break;
        default:
            clear_bufferBT();
            break;
        }
    }
}

/* 
    Отправка бинарных данных верхнему уровню
    массив из трёх чисел (координаты и угол) типа float преобразуем в массив
    12 байт. После считаем контрольную сумму, добавляем к массиву и отправляем 
*/
void send_data_HLVL()
{
    float data_float[] = {position[0], position[1], theta}; // формируме массив данных [X Y theta]
    uint8_t data_bytes[12];                                 // массив байт 3*4 = 12 байт
    memcpy(&data_bytes, data_float, sizeof(float) * 3);     // конвертация в массив байтов

    uint16_t check_sum_sum = 0; // для суммы байт
    uint8_t check_sum;          // для контрольной суммы
    for (uint8_t i = 0; i < sizeof(data_bytes); i++)
    {
        check_sum_sum += data_bytes[i];
    }
    check_sum = (uint8_t)(check_sum_sum & 0xFF); // взятие младшего байта

    // отправка массива байт
    for (uint8_t i = 0; i < sizeof(data_bytes); i++)
    {
        Serial2.write(data_bytes[i]);
        //
        Serial.println(data_bytes[i]); // на экран
        //
    }
    // и контр. сумма
    Serial2.write(check_sum);
    //
    Serial.println(check_sum); // на экран
}

// приём данных с верхнего уровня
bool recieve_data_HLVL(char type_msg)
{
    uint16_t check_sum_sum = 0; // для суммы байт
    uint8_t check_sum;          // для контрольной суммы

    if (type_msg == FI_VEL) // если следует принять скорости колёс
    {
        Serial2.readBytes(bytes_FI_VEL, sizeof(bytes_FI_VEL)); // читаем байты

        // считаем контрольную сумму
        for (uint8_t i = 0; i < (sizeof(bytes_FI_VEL) - 1); i++)
        {
            check_sum_sum += bytes_FI_VEL[i];
        }
        check_sum = (uint8_t)(check_sum_sum & 0xFF);

        //Serial.print("CS\t");
        //Serial.println(check_sum);

        if (check_sum == bytes_FI_VEL[sizeof(bytes_FI_VEL) - 1]) // если контрольная сумма совпадает
            return true;
        else
            return false;
    }
    else if (type_msg == LOC_VECTOR) // если следует принять локальный вектор
    {
        Serial2.readBytes(bytes_LOC_VECTOR, sizeof(bytes_LOC_VECTOR)); // читаем байты
        // считаем контрольную сумму
        for (uint8_t i = 0; i < (sizeof(bytes_LOC_VECTOR) - 1); i++)
        {
            check_sum_sum += bytes_LOC_VECTOR[i];
        }
        check_sum = (uint8_t)(check_sum_sum & 0xFF);

        //Serial.print("CS\t");
        //Serial.println(check_sum);

        if (check_sum == bytes_LOC_VECTOR[sizeof(bytes_LOC_VECTOR) - 1])
            return true;
        else
            return false;
    }
    else if (type_msg == SET_BEACON_POS) // координаты маяков
    {
        Serial2.readBytes(BB_set_bytes, sizeof(BB_set_bytes)); // читаем байты

        // считаем контрольную сумму
        check_sum_sum = 0;
        for (uint8_t i = 0; i < (sizeof(BB_set_bytes) - 1); i++)
        {
            check_sum_sum += BB_set_bytes[i];
        }
        check_sum = (uint8_t)(check_sum_sum & 0xFF);

        //Serial.print("CS\t");
        //Serial.println(check_sum);

        if (check_sum == BB_set_bytes[sizeof(BB_set_bytes) - 1]) // если контрольная сумма совпадает
            return true;
        else
            return false;
    }
    else
        return false;
}

// конвертировать байты в массив скоростей для отправки драйверам
void convert_FI_VEL()
{
    uint8_t sizeData = (sizeof(bytes_FI_VEL) - 1) / 4; // количество чисел float
    uint8_t buf[4];                                    // промежуточный массив, содержащий байты числа

    Serial.println("driverFI");

    for (uint8_t i = 0; i < sizeData; i++)
    {
        for (uint8_t j = 0; j < 4; j++)
        {
            buf[j] = bytes_FI_VEL[i * 4 + j]; // определяем массив из 4х байт для числа
        }

        memcpy(&data_for_driver[i], buf, sizeof(float)); // собираем число

        //data_for_driver[i] = convert_bytes2float(buf);
        Serial.println(data_for_driver[i]);
    }
}

// коневертировать байты в коордианты для устанвоки маяков
void convert_BB(float *bb_set)
{
    uint8_t sizeData = (sizeof(BB_set_bytes) - 1) / 4; // количество чисел float
    //uint8_t sizeData = 4;
    uint8_t buf[4]; // промежуточный массив, содержащий байты числа

    Serial.println("BB set");

    for (uint8_t i = 0; i < sizeData; i++)
    {
        for (uint8_t j = 0; j < 4; j++)
        {
            buf[j] = BB_set_bytes[i * 4 + j]; // определяем массив из 4х байт для числа
        }

        memcpy(&bb_set[i], buf, sizeof(float)); // собираем число

        //data_for_driver[i] = convert_bytes2float(buf);
        Serial.println(bb_set[i]);
    }
}

// конвертация массива байтов в значения локальных скоростей
void convert_LOC_VECTOR()
{
    uint8_t sizeData = (sizeof(bytes_LOC_VECTOR) - 1) / 4; // кол-во чисел float, зависит от кол-ва байт
    uint8_t buf[4];                                        // промежуточный массив, содержащий байты числа

    Serial.println("driverLOC");

    for (uint8_t i = 0; i < sizeData; i++)
    {
        for (uint8_t j = 0; j < 4; j++)
        {
            buf[j] = bytes_LOC_VECTOR[i * 4 + j]; // определяем массив из 4х байт для числа
        }

        memcpy(&Uc[i], buf, sizeof(float)); // собираем число

        //data_for_driver[i] = convert_bytes2float(buf);
        Serial.println(Uc[i]);
    }
}

// конвертировать байты в float
float convert_bytes2float(uint8_t float_bytes[])
{
    float f_data;
    memcpy(&f_data, float_bytes, sizeof(f_data));
    return f_data;
}

//очистка буфера Serial2
void clear_bufferBT()
{
    for (int i = 0; i < Serial2.available(); i++)
    {
        Serial2.read();
    }
}