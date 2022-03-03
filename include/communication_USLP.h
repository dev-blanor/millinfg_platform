/*
    *** ПРОТОКОЛ ОБМЕНА ДАННЫМИ С УСЛП (VER 1) ***
    *** Настройка системы и получение координат ***

    С УСЛП связь происходит по UART через Serial1 порт контроллера. 

    Скорость соединения равна 38400. 
    (подробное описание протокола в тетради)
    Все команды и данные отправляются в символьном формате (т.е. можно
    общаться через терминальные программы). Ответ может быть как в символьном, так и в бинарном виде.

    1) Настройка частоты
        команда: "Fxxx;" F - дескриптор, xxx - данные (частота)
        ответ: "F0" - успешно, "F1" - ошибка

    2) Настройка координат расположения маяков
        команда: "Bxxx1;yyy1;xxx2;yyy2;"
                  |___#1___| |___#2___|
        ответ: "B0" - успешно, "B1" - ошибка

    3) Запрос коордиант
        3.1) координаты каждого излучателя в ЛСК и БСК
            команда: "P[L/R][L/G][a/b]"
                данные: 'L' - левый излучатель, 'R' - правый излучатель
                        'L' - локальные кордианты, 'G' - глобальные
                        'a' - формат данных ASCII, 'b' - формат данных BIN
                например:  "PLLa" - левый излучатель в ЛСК, символьный формат
                            "PRGb" - правый излучатель в БСК, бинарный формат
            ответ в виде: [заголовок][данные]  
                    заголовок = запросу
                    данные:
                    в случае 'a' - "P[L/R][L/G][a/b]xxx;yyy\n" , xxx - x-координата, yyy - y-координата
                    в случае 'b' - "P[L/R][L/G][a/b]"[0bxx1 ... 0bxx4][0byy1 ... 0byy4][CS]
                                                      |  4 байта x  | |  4 байта y  | | контр. сумма |
        3.2*) координаты средней точки между излучателями и угол наклона вектора,
        проведенного через координаты излучателей, к оси X
            команда: "P[C][L/G][a/b]"
                данные: 'C' - координаты и угол
                        'L' - локальные кордианты, 'G' - глобальные
                        'a' - формат данных ASCII, 'b' - формат данных BIN
                например:  "PCLb" - координата и угол в ЛСК, бинарный формат
            ответ в виде: [заголовок][данные]  
                    заголовок = запросу
                    данные:
                    в случае 'a' - "P[С][L/G][a/b]xxx;yyy;aaa\n" , xxx - x-координата, yyy - y-координата, 
                    aаа - угол                    
                    в случае 'b' 
                    "P[C][L/G][a/b]"[0bxx1 ... 0bxx4][0byy1 ... 0byy4][0byy1 ... 0byy4][CS]
                                     |  4 байта x  |    |  4 байта y  |   |  4 байта а  | | контр. сумма |
*/

#include <Arduino.h>

void communication_USLP_init();
void chek_data_from_USLP();
bool receive_L_or_R_TR_data(char type_data);
bool receive_POS_and_ANGLE_data(char type_data);
void clear_buffer_ser1();
void request_USLP_data(char req, char type_data);
void request_USLP();
void set_BB_coordinates(float *bb_coordinates);

//ДЕСКРИПТОРЫ
//настройки
#define SET_FREQ 'F'       //настройка частоты
#define SET_BEACON_POS 'B' //изменение координат маяков
//запрос
//****** запрос координат ******
#define REQ_POS 'P'
#define LEFT_TR 'L'       // левый излучатель
#define RIGHT_TR 'R'      // правый излучатель
#define POS_AND_ANGLE 'C' // координаты и угол

#define LOCAL_POS 'L'       //коордианты в ЛСК (отн. маяков)
#define GLOBAL_POS 'G'      //координаты глобальные
#define TYPE_DATA_ASCII 'a' //типа данных - символьный
#define TYPE_DATA_BIN 'b'   //тип данных - бинарный

#define WAIT_DATA_USLP 100 //время ожидания данных, мс

float data_float_LR[2]; // промежуточный массив для хранения значений координат излучателей
float data_float_C[3];  // промежуточный массив для хранения значений координат центра и угла [X, Y, THETA]

bool data_uslp_ready = false; // флаг готовности данных от УСЛП

#define DATA_NOTINPUT 0 // 0 - данных нет/не принято
#define DATA_INPUT_OK 1 // 1 - данные приняты без ошибок
#define DATA_INPUT_ER 2 // 2 - данные приняты с ошибкой контрольной суммы
// статус данных с УСЛП
int data_uslp_status = DATA_NOTINPUT;

#define request_FREQ 8       // частота опроса УСЛП, Гц
bool request_iSsend = false; // флаг статуса запроса (false - неотправлен, true - запос отправлен)
uint32_t ts_request = 0;     // для таймера ожидания разрешение на запрос
int count_er_response = 0;   // счётчик тактов ожидания ответа

//инициализация
void communication_USLP_init()
{
    Serial1.begin(38400);
    Serial1.setTimeout(WAIT_DATA_USLP); //время ожидания данных
}

//мониторинг запросов
void chek_data_from_USLP()
{
    while (Serial1.available() != 0) //пока есть данные в буфере
    {
        char descriptor = Serial1.read(); //читаем дескриптор P
        char buf[3];                      //заголовка ответа [L/R/C][L/G][a/b]
                                          //                    0     1    2
        data_uslp_status = DATA_INPUT_ER; //по умолчанию, если что-то пришло
        if (descriptor == REQ_POS)
        {
            if (Serial1.readBytes(buf, 3) == 3) // читаем заголовок ответа,в котором параметры данных
            {
                switch (buf[0])
                {
                case LEFT_TR:                           // координаты левого излучателя
                    if (receive_L_or_R_TR_data(buf[2])) //если данные корректны
                    {
                        posTL[0] = data_float_LR[0];
                        posTL[1] = data_float_LR[1];
                        data_uslp_status = DATA_INPUT_OK;
                        data_uslp_ready = true;
                    }
                    break;

                case RIGHT_TR: // координаты правого излучателя
                    if (receive_L_or_R_TR_data(buf[2]))
                    {
                        posTR[0] = data_float_LR[0];
                        posTR[1] = data_float_LR[1];
                        data_uslp_status = DATA_INPUT_OK;
                        data_uslp_ready = true;
                    }
                    break;

                case POS_AND_ANGLE: // координата и угол
                    if (receive_POS_and_ANGLE_data(buf[2]))
                    {
                        position[0] = data_float_C[0];
                        position[1] = data_float_C[1];
                        theta = data_float_C[2];
                        data_uslp_status = DATA_INPUT_OK;
                        data_uslp_ready = true;
                    }
                    break;

                default:
                    clear_buffer_ser1();
                    break;
                }
            }
            else
                clear_buffer_ser1();
        }
        /////////////////////////////*******************////////////////
        else if (descriptor == SET_BEACON_POS) // пришёл отвте от УСЛП на команду изменить координаты маяков
        {
            Serial1.read();
            Serial.println("B0");
        }
        ///////////////////////////************************//////////////
        else
            clear_buffer_ser1();
    }
}

// получение координат левого или правого излучателя
bool receive_L_or_R_TR_data(char type_data)
{
    uint8_t bytes[9]; // массив байт 4 * 2 + 1 байт контр. суммы
    uint8_t _bytes_[8];
    uint16_t check_sum_sum = 0; // для суммы байт
    uint8_t check_sum;          // для контрольной суммы
    bool flag = false;

    if (type_data == TYPE_DATA_ASCII) // символьный формат
    {
        data_float_LR[0] = Serial1.parseFloat();
        Serial.read();
        data_float_LR[1] = Serial1.parseFloat();
        Serial.read();
        flag = true;
    }
    else if (type_data == TYPE_DATA_BIN)
    {
        Serial1.readBytes(bytes, sizeof(bytes));
        // считаем контрольную сумму байт (без последнего )
        for (uint8_t i = 0; i < (sizeof(bytes) - 1); i++)
        {
            check_sum_sum += bytes[i];
            _bytes_[i] = bytes[i]; // сохраняем байты данных в массив, без контрольной суммы
            // его используем для конвертации
        }
        check_sum = (uint8_t)(check_sum_sum & 0xFF); // берём младший байт

        if (check_sum == bytes[sizeof(bytes) - 1])            // если контрольная сумма совпадает
        {                                                     // сумма совпала -> данные не потеряны
            memcpy(&data_float_LR, _bytes_, sizeof(_bytes_)); // собираем массив данных
            flag = true;                                      // поднимаем флаг
        }
        else
            return false;
    }
    else
        return false;

    return flag;
}

// получение середины отрезка и угла наклона вектора
bool receive_POS_and_ANGLE_data(char type_data)
{
    uint8_t bytes[13]; // массив байт 4 * 3 + 1 байт контр. суммы 13
    uint8_t _bytes_[12];
    uint16_t check_sum_sum = 0; // для суммы байт
    uint8_t check_sum;          // для контрольной суммы
    bool flag = false;

    if (type_data == TYPE_DATA_ASCII) // символьный формат
    {
        data_float_C[0] = Serial1.parseFloat();
        Serial.read();
        data_float_C[1] = Serial1.parseFloat();
        Serial.read();
        data_float_C[2] = Serial1.parseFloat();
        Serial.read();
        flag = true;
    }
    else if (type_data == TYPE_DATA_BIN)
    {
        Serial1.readBytes(bytes, sizeof(bytes));
        // считаем контрольную сумму байт (без последнего )
        for (uint8_t i = 0; i < (sizeof(bytes) - 1); i++)
        {
            check_sum_sum += bytes[i];
            _bytes_[i] = bytes[i]; // сохраняем байты данных в массив, без контрольной суммы
            // его используем для конвертации
        }
        check_sum = (uint8_t)(check_sum_sum & 0xFF); // берём младший байт

        if (check_sum == bytes[sizeof(bytes) - 1])           // если контрольная сумма совпадает
        {                                                    // сумма совпала -> данные не потеряны
            memcpy(&data_float_C, _bytes_, sizeof(_bytes_)); // собираем массив данных
            flag = true;                                     // поднимаем флаг
        }
        else
            return false;
    }
    else
        return false;

    return flag;
}

// отправка запроса на данные от УСЛП.
// В аргументе указываем запрос на что и в каком формате, ascii или bin
// (пока только локальные координаты)
void request_USLP_data(char req, char type_data)
{
    switch (req)
    {
    case POS_AND_ANGLE: // P[C][L][a/b]
        Serial1.write(REQ_POS);
        Serial1.write(POS_AND_ANGLE);
        Serial1.write(LOCAL_POS);
        Serial1.write(type_data);
        break;
    case LEFT_TR: // P[L][L][a/b]
        Serial1.write(REQ_POS);
        Serial1.write(LEFT_TR);
        Serial1.write(LOCAL_POS);
        Serial1.write(type_data);
        break;
    case RIGHT_TR: // P[R][L][a/b]
        Serial1.write(REQ_POS);
        Serial1.write(RIGHT_TR);
        Serial1.write(LOCAL_POS);
        Serial1.write(type_data);
        break;

    default:
        break;
    }
}

// установка координат маяков
void set_BB_coordinates(float *bb_coordinates)
{
    Serial1.write(SET_BEACON_POS);
    Serial1.print(bb_coordinates[0]); // x1
    Serial1.print(';');
    Serial1.print(bb_coordinates[0]); // x1
    Serial1.print(';');
    Serial1.print(bb_coordinates[1]); // y1
    Serial1.print(';');
    Serial1.print(bb_coordinates[2]); // x2
    Serial1.print(';');
    Serial1.print(bb_coordinates[3]); // y2
    Serial1.print(';');
}
// отправка запроса УСЛП. Запросы формируются с некторой частотой request_FREQ.
// При задержки или отсуствия ответа запоросы отправляется после переполнения отсчёта,
// когда он равен "request_FREQ". Если ответ пришёл, с ошибкой или без, всё равно отправак запроса
void request_USLP()
{
    if (!request_iSsend)
    {                                                    // если запрос НЕ был отправлен
        request_USLP_data(POS_AND_ANGLE, TYPE_DATA_BIN); // запрос
        request_iSsend = true;                           // запрос отправлен
        count_er_response = 0;
        //ts_request = millis();
        //Serial.println(millis());
    }
    else if ((millis() - ts_request) >= 1000 / request_FREQ)
    {
        ts_request = millis();
        // данные пришли, с ошибкой или без, или число тактов ожидания превысило максимальное...
        if (data_uslp_status == DATA_INPUT_OK || data_uslp_status == DATA_INPUT_ER || count_er_response >= request_FREQ)
        {
            //Serial.println(data_uslp_status);
            request_iSsend = false;           // даём разрешение на запрос
            data_uslp_status = DATA_NOTINPUT; //сброс статуса данных с УСЛП в состояние "нет данных"
            data_uslp_ready = false;          // сброс флага принятия данных (не обязательно)
            count_er_response = 0;            // обнуляем счётчик
        }
        else // значит данные не пришли, ждём-с... кол-во тактов = частоте опросов
        {
            count_er_response++;
        }
    }
}

//очистка буфера
void clear_buffer_ser1()
{
    for (int i = 0; i < Serial1.available(); i++)
    {
        Serial1.read();
    }
}