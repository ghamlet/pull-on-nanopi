//Правка под версию WorkBench 1.4

/* Ищем dynamixel_item.cpp
   Внутри описаны все названия из таблицы протокола, которые сейчас поддерживаются в WorkBench. Так, P_gain, I_gain, D_gain, а также Present_Temperature не поддерживаются.
   TODO: добвить эти таблицы в dynamixel_item.h оффициально в git репозиторий. Что сделано сейчас: добавлено локально в файл dynamixel_item необходимые куски таблицы для использования в Workbench
   Для этого заходим в dynamixel_item.cpp, ищем нужный тип движка (setMXItem (MX28), setExtMXItem(MX64), setAXItem(AX18/12)) и в самый конец таблицы для OPENCM добавляем новый
   итем с нужными характеристиками. Если посмотреть в dynamixel_tool.cpp метод DynamixelTool::getControlItem, то увидем сравнение строк по названию => более нам ничего добавлять не нужно.
   Не забываем увеличить размерность массива the_number_of_item.
   TODO: почему-то с MX28 считать температуру все-равно не выходит. С других движков норм.
   Если поставить для MX28 строку температуры после "present_Load" (40, далее 43 как раз), то все работает.
   Изучить этот момент более детально. Мб мы теряем Goal_Acceleration? А поч на других движках работает?
*/
/* В DW добавили кастомно параметры P,I,D, Present_Tempreture. Важно чтобы они в массиве шли последовательно согласно битовой позиции!!! */
#include "arm_IK_lib.h"


#define MOVE_STEP_TIME 15
#define MOVE_STEP_LEN 0.5
#define MAX_FI 150.
#define MIN_FI 0.
int32_t data_packet[DXL_CNT] = {0, 0, 0, 0, 0, 0};

// Это переменные нужны при анализе пакета позиционирования и ориентации, приходящего с сервера: p:rotation:distance:DOWN/UP:OPEN/CLOSE
float rotation = 0;
float distance = 0;
float z;
float fi = PI;

// Эта переменная нужна для засекания времени при отправке пакетов
long packet_timer = 0;

// Эта переменная нужна для выбора фазы отсылки данных: 0,2,4 - позиция; 1,3 - нагрузка; 5 - температура.
int packet_phase = 0;

// Определяет будет ли отправляться данные  робота
int SendMode = 0;

// Эта переменная нужна для планирования траектории
float ex_pos_x = 0;
float ex_pos_z = 0;
float ex_angle = 0;

// Эта переменная нужна для разрешения планирования траектории
int allow_trajectory = 0;
int steps = 0;
float delta_x ;
float delta_z ;
float delta_angle = 0;
float dxl_speed_med = 0.;
int wait_time = 70;
float step_len = 1.;
float planed_x, planed_z, planed_angle;
long move_timer = 0;

int first_move = 1;

int moving = 0;

const uint8_t handler_index_pos = 0;
const uint8_t handler_index_torque = 1;
const uint8_t handler_index_speed = 2;



void setup()
{
  Serial.begin(9600); // Скорость отправки и приема данных через последовательный порт.
  while (!Serial); // TODO: если вам не нужен запуск руки после открытия последовательного порта, то эту строчку нужно убрать: она стопорит программу до этого момента.
 
 //Для логирования и контроля ошибок
const char *log;
bool result = false;

  //выполняем инициализацию DXL шины на заданной скорости
  Serial.println("System Boot");
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);  
  }
  
//инициализируем динамиксели и устанавливаем им режим работы в Joint
  Serial.println("Setting DXLs");
  dxls_init();


  //Инициализируем синхронную запись
  Serial.println("Setting SyncWrite Goal_Position");
 
 //Настройка синхронной записи Goal_Position - handler_index_pos = 0
  result = dxl_wb.addSyncWriteHandler(1, "Goal_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
  }
    Serial.println("Setting SyncWrite Torque_Enable");

    
//Настройка синхронной записи Torque_Enable - handler_index_torque = 1
 result = dxl_wb.addSyncWriteHandler(1, "Torque_Enable", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
  }
  
  Serial.println("Setting SyncWrite Moving_Speed");
//Настройка синхронной записи Moving_Speed handler_index_speed = 2
   result = dxl_wb.addSyncWriteHandler(1, "Moving_Speed", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
  }
  
   
// Сразу ограничим скорость движения элементов руки
  result = dxl_wb.syncWrite(handler_index_speed, goal_speed, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync goal_speed");
  }


//где это используется?
  calc_ik_angular_offsets();  // Считаем угловые смещения всех двигателей


  Serial.println("Ready");
  RedrawMenu();

  allow_trajectory = 0; //Изначально планирования траектории нет
  distance = 0.;
  rotation = 0.;
  z = 0.;
  planed_x = 0.;
  planed_z = 0.;
  planed_angle = 0.;
  packet_timer = millis();
}


void loop()
{
//Для логирования и контроля ошибок
const char *log;
bool result = false;
  

  //Блок отсылки данных по фазам
  if ((millis() - packet_timer >= PACKET_DELAY) && SendMode)
  {
    packet_timer = millis();
    Serial.print(String(is_moving()) + ":");
    switch (packet_phase % 6)
    {
      case 1: case 3:
        get_data_load(data_packet, "Present_Load");
        Serial.print("L:");
        break;
      case 5:
        get_data_temp(data_packet, "Present_Temperature");
        Serial.print("T:");
        break;
      default:
        get_data_pos(data_packet, "Present_Position");
        Serial.print("M:");
    }
    //  Serial.println(dxl_wb.itemRead(dxl_id[0], "Present_Temperature"));
    print_packet(data_packet);
    packet_phase++;
  }

  /***************НАЧАЛО БЛОКА ЧТЕНИЯ ПОСЛЕДОВАТЕЛЬНОГО ПОРТА*******************************/
  if (Serial.available())
  {
    float buf_distance = 0.;
    float buf_rotation = 0.;
    float buf_z = 0.;
    float buf_fi = 0.;

    char c = Serial.read(); //читаем первый байт

    //Начинаем парсинг пакета
    switch (c)
    {
      case '1':
        MoveHome();
        Serial.println("\nMoving to start position");
        break;

      case '2':
        MoveCenter();
        Serial.println("\nMoving to center position");
        break;

      case '3':
        RelaxServos();
        Serial.println("\nMoving to safe position and relaxing");
        break;

      case 'h':
        RedrawMenu();
        break;

      case '\n':    //Игнорим символ новой строки
        break;

      case 'r':
        SendMode = 1;
        Serial.println("\nSending activated.");
        RedrawMenu();
        break;

      case 's':
        SendMode = 0;
        Serial.println("\nSending stoped.");
        RedrawMenu();
        break;

      case 'g':
        /***************НАЧАЛО ПАРСЕРА*******************************/

        if ( (c = Serial.read()) == ':' )
        {
          while ((c = Serial.read()) != ':')
          {
            buf_rotation *= 10;
            buf_rotation += (int)(c - '0');
            if (buf_rotation > 1000)
              break;
          }
          if ((buf_rotation > MAX_ROT) || (buf_rotation < MIN_ROT))
          {
            Serial.println("\nRotation out of bounds. Max: " + String(MAX_ROT) + " Min: " + String(MIN_ROT));
            // Очищаем буфер
            while (Serial.available() > 0)
              Serial.read();
          }
          else
          {
            while ((c = Serial.read()) != ':' )
            {
              buf_distance *= 10;
              buf_distance += (int)(c - '0');
              if ( buf_distance > 1000)
                break;
            }


            if ((buf_distance > MAX_DIST) || (buf_distance < MIN_DIST))
            {
              Serial.println("\nDistance out of bounds. Max: " + String(MAX_DIST) + " Min: " + String(MIN_DIST));
              while (Serial.available() > 0)
                Serial.read();
            }
            else
            {
              int sign = 1;
              if ((c = Serial.read()) == '-')
                sign = -1;
              else
              {
                buf_fi *= 10;
                buf_fi += (int)(c - '0');
              }
              while ((c = Serial.read()) != ':' )
              {
                buf_fi *= 10;
                buf_fi += (int)(c - '0');
                if ( buf_fi > 1000)
                  break;
              }
              buf_fi = sign * buf_fi;
              if ((fabs(buf_fi) > MAX_FI) || (fabs(buf_fi) < MIN_FI))
              {
                Serial.println("\nEnd-effector rotation out of bounds. Max module: " + String(MAX_FI) + " Min: " + String(MIN_FI));
                while (Serial.available() > 0)
                  Serial.read();
              }
              else
              {
                if ((c = Serial.read()) == '1')
                  buf_z = CARGO_POS_Z;
                else if ( c == '0')
                {
                  buf_z = ABOVE_CARGO_Z;
                }
                else if ( c == '2')
                {
                  buf_rotation = 140;
                  buf_distance = 180;
                  buf_z = ABOVE_CARGO_Z;
                }
                else if ( c == '3')
                {
                  buf_rotation = 180;
                  buf_z = (CARGO_POS_Z + ABOVE_CARGO_Z) / 2 ;
                  buf_distance = 180.;
                }
                if ((c = Serial.read()) == ':')
                {
                  if ((c = Serial.read()) == '0')
                    goal_position[sync_order(GRIPPER)] = END_EFFECTOR_OPEN;
                  else if (c == '1')
                    goal_position[sync_order(GRIPPER)] = END_EFFECTOR_CLOSE;
                  if ((c = Serial.read()) == '#')
                  {
                    // Движение возможно только после полностью корректного пакета
                    Serial.println("\nExecuting");
                    fi = (buf_fi + 180.) / 180. * PI;
                    goal_position[4] = angle_to_pos(fi, 'A');
                    //  Serial.println(String(dxl_id[GRIPPER - 1]) + ": " + String(goal_position[sync_order(GRIPPER - 1)]));
                    dxl_wb.itemWrite(5, "Goal_Position",  goal_position[4]);
                    dxl_wb.itemWrite(6, "Goal_Position", goal_position[sync_order(GRIPPER)]);

                    allow_trajectory = 1; //Если нам нужно разрешить планирование траектории, то измените это значение на 1.
                    buf_rotation = buf_rotation / 180. * PI;

                    rotation = buf_rotation;
                    distance = buf_distance;
                    z = buf_z;
                  }
                  else
                  {
                    Serial.println("\nIncorrect packet: No # symbol");
                    while (Serial.available() > 0)
                      Serial.read();
                  }
                  break;
                }
                else
                {
                  Serial.println("\nIncorrect packet: No gripper command");
                  while (Serial.available() > 0)
                    Serial.read();
                }
              }
            }
          }
        }
        else
        {
          Serial.println("\nIncorrect packet");
          while (Serial.available() > 0)
            Serial.read();
        }
        break;
      /***************КОНЕЦ ПАРСЕРА*******************************/

      default:
        Serial.println("\nIncorrect packet");
        while (Serial.available() > 0)
          Serial.read();
    }
  }
  /***************КОНЕЦ БЛОКА ЧТЕНИЯ ПОСЛЕДОВАТЕЛЬНОГО ПОРТА*******************************/
  //Serial.println(goal_position[4]);
  if ((distance != 0) && (z != 0) && (rotation != 0))
    if ( millis() - move_timer > MOVE_STEP_TIME)
    {
      move_timer = millis();
      TrajectoryPlanning(distance, z, rotation);
    }
}



void TrajectoryPlanning(float x, float z, float angle)
{
  // Serial.println(String(steps) + ": " + String(ex_pos_x) + " : " + String(x) + " : " + String(ex_angle));
 // Serial.println(dxl_wb.itemRead(1, "Moving"));

 //Для логирования и контроля ошибок
const char *log;
bool result = false;

  if (first_move)
  {

    inverse_kinematics(x, z, angle);
 //   dxl_wb.syncWrite("Goal_Position", goal_position);
  result = dxl_wb.syncWrite(handler_index_pos, goal_position, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync goal_position");
  }

 
    delay(100);
    // moving = 0;
    if (!is_moving())
    {

      ex_pos_x = x;
      planed_x = x;
      ex_pos_z = z;
      planed_z = z;
      ex_angle = angle;
      planed_angle = angle;
      first_move = 0;
      moving = 0;
      Serial.println("first_move_ended");
    }
  }
  else if ((allow_trajectory) && ((x != ex_pos_x) || (z != ex_pos_z) || angle != ex_angle))
  {
    moving = 1;
    if ((planed_x != x) || (planed_z != z) || (planed_angle != angle))
    {
      planed_x = x;
      planed_z = z;
      planed_angle = angle;
      delta_x = fabs(x - ex_pos_x);
      delta_z = fabs(z - ex_pos_z);
      delta_angle = fabs(angle - ex_angle);

      if (delta_x > delta_z)
        steps = delta_x / MOVE_STEP_LEN;
      else if (delta_x < delta_z)
        steps = delta_z / MOVE_STEP_LEN;
      else if (delta_x == 0 && delta_z == 0)
        steps = delta_angle / MOVE_STEP_LEN;
      // Расчет дельт шага
      delta_x = (x - ex_pos_x) / (double)steps;
      delta_z = (z - ex_pos_z) / (double)steps;
      delta_angle = (angle - ex_angle) / (double)steps;

      //adapt_vel(ex_pos_x, ex_pos_z, ex_angle, ex_pos_x + delta_x, ex_pos_z + delta_z, ex_angle + delta_angle);

    }
   /* if ( millis() - move_timer > MOVE_STEP_TIME)
    {
      move_timer = millis();*/
      steps--;
      //перемещение дискретное
      ex_pos_x += delta_x;
      ex_pos_z += delta_z;
      ex_angle += delta_angle;
      //   Serial.println(String(steps) + ": " + String(ex_pos_x) + " : " + String(x) + " : " + String(ex_angle));

      inverse_kinematics(ex_pos_x, ex_pos_z, ex_angle);
//      dxl_wb.syncWrite("Goal_Position", goal_position);
  result = dxl_wb.syncWrite(handler_index_pos, goal_position, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync goal_position");
  }


      if (steps <= 1) //На всякий случай дополнительный довод
      {
        ex_pos_x = x;
        planed_x = x;
        ex_pos_z = z;
        planed_z = z;
        ex_angle = angle;
        planed_angle = angle;
        inverse_kinematics(x, z, angle);
//        dxl_wb.syncWrite("Goal_Position", goal_position);
   result = dxl_wb.syncWrite(handler_index_pos, goal_position, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync goal_position");
  }

        moving = 0;
      //}
    }
  }
  else
    moving = 0;
}


void MoveHome()
{
  Serial.println("\nMoving to center position");
  allow_trajectory = 1;
  goal_position[5] = END_EFFECTOR_CLOSE;
  distance = L2;
  z = L1 + BASE_HEIGHT - L3;
  rotation = PI;
}


void MoveCenter()
{
  Serial.println("\nMoving to center position");
  allow_trajectory = 1;
  goal_position[5] = END_EFFECTOR_CLOSE;
  rotation = 230 / 180. * PI;
  distance = 220.;
  z = (CARGO_POS_Z + ABOVE_CARGO_Z) / 2;
}


void RelaxServos()
{
//Для логирования и контроля ошибок
const char *log;
bool result = false;
  
  allow_trajectory = 0;
  first_move = 1;
  distance = 0.;
  rotation = 0.;
  z = 0.;
  
//  dxl_wb.syncWrite("Torque_Enable", torque_off);
  result = dxl_wb.syncWrite(handler_index_torque, torque_off, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync torque_off");
  }

  
  
}


void RedrawMenu()
{
  Serial.println("\n################# g:240:220:0:0:1###################");
  Serial.println("# Please enter option 1-7 to run");
  Serial.println("# (h) Redraw Menu");
  Serial.println("# (r) Run Sending Data");
  Serial.println("# (s) Stop Sending Data");
  Serial.println("# (p) Move RoboArm to position p:angle:distance:state:take");
  Serial.println("# (1) Move to Start Position");
  Serial.println("# (2) Move to Center");
  Serial.println("# (3) Relax servos");
  Serial.println("###################################");
}