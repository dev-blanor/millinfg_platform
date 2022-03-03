#include <platform_ctrl.h>
#include <communication_BT.h>
#include <communication_USLP.h>

void setup()
{
    Serial.begin(38400);
    Wire.begin();
    init_communicate_BT();
    communication_USLP_init();
}

void loop()
{
    request_USLP();
    // соединение через bluetooth
    chek_data_from_BT();
    // соединения с УСЛП
    chek_data_from_USLP();

    if (flag_data_Ur) // данные по локальным скоростям обновлены
    {
        flag_data_Ur = false;
        inv_kin_local_maket();
        for (int i = 0; i < NUM_DRIVERS; i++)
        {
            data_for_driver[i] = fi_vel[i];
        }
        flag_data_for_driver = true; // можно отправить драйверам
    }

    if (flag_data_for_driver) // данные отправки драйверам обновлены
    {
        flag_data_for_driver = false;
        send_data_to_drivers();
    }
}