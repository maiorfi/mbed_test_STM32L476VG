#include "mbed.h"

#include "LCD_DISCO_L476VG.h"
#include "GYRO_DISCO_L476VG.h"
#include "COMPASS_DISCO_L476VG.h"
#include "QSPI_DISCO_L476VG.h"

static DigitalOut led_green(LED1);
static DigitalOut led_red(LED2);

static DigitalIn joy_btn(JOYSTICK_CENTER);

static DigitalIn joy_up(JOYSTICK_UP);
static DigitalIn joy_down(JOYSTICK_DOWN);
static DigitalIn joy_left(JOYSTICK_LEFT);
static DigitalIn joy_right(JOYSTICK_RIGHT);

static LCD_DISCO_L476VG lcd;
static GYRO_DISCO_L476VG gyr;
static COMPASS_DISCO_L476VG compass;
static QSPI_DISCO_L476VG qspi;

static Thread s_thread_trace;
static Thread s_thread_sensors;
static Thread s_thread_ui_out;
static Thread s_thread_ui_in;

static EventQueue s_eq_trace;
static EventQueue s_eq_sensors;
static EventQueue s_eq_ui_out;
static EventQueue s_eq_ui_in;

static Mutex s_mutex_gyr_data;
static Mutex s_mutex_mag_data;
static Mutex s_mutex_acc_data;

static float s_gyr_data[3];
static int16_t s_mag_data[3];
static int16_t s_acc_data[3];

static Timer s_timer_1;

typedef enum _ui_out_state_type
{
    UI_OUT_STATE_BEGIN,

    UI_OUT_STATE_GYR,
    UI_OUT_STATE_MAG,
    UI_OUT_STATE_ACC,
    
    UI_OUT_STATE_END

} ui_out_state_type;

typedef enum _ui_out_axis_type
{
    UI_OUT_AXIS_BEGIN,

    UI_OUT_AXIS_X,
    UI_OUT_AXIS_Y,
    UI_OUT_AXIS_Z,
    
    UI_OUT_AXIS_END

} ui_out_axis_type;

static ui_out_state_type ui_out_state=UI_OUT_STATE_GYR;
static Mutex s_mutex_ui_out_state;

static ui_out_axis_type ui_out_axis=UI_OUT_AXIS_X;
static Mutex s_mutex_ui_out_axis;

#define QSPI_BUFFER_SIZE ((uint32_t)0x0100)

static uint8_t s_from_qspi_buffer[QSPI_BUFFER_SIZE];
static uint8_t s_to_qspi_buffer[QSPI_BUFFER_SIZE];

bool qspi_erase()
{
    return (qspi.Erase_Block(0) == QSPI_OK);
}

bool qspi_read()
{
    return (qspi.Read(s_from_qspi_buffer, 0, QSPI_BUFFER_SIZE) == QSPI_OK);
}

bool qspi_write()
{
    return (qspi.Write(s_to_qspi_buffer, 0, QSPI_BUFFER_SIZE) == QSPI_OK);
}

bool save_state()
{
    memcpy(s_to_qspi_buffer,&ui_out_state,sizeof(ui_out_state));
    memcpy(s_to_qspi_buffer+sizeof(ui_out_state),&ui_out_axis,sizeof(ui_out_axis));

    if(!qspi_erase()) return false;
    return qspi_write();
}

bool load_state()
{
    if(!qspi_read()) return false;

    memcpy(&ui_out_state,s_from_qspi_buffer,sizeof(ui_out_state));
    memcpy(&ui_out_axis,s_from_qspi_buffer+sizeof(ui_out_state),sizeof(ui_out_axis));

    if(ui_out_state<=UI_OUT_STATE_BEGIN || ui_out_state>=UI_OUT_STATE_END) ui_out_state=UI_OUT_STATE_GYR;
    if(ui_out_axis<=UI_OUT_AXIS_BEGIN || ui_out_axis>=UI_OUT_AXIS_END) ui_out_axis=UI_OUT_AXIS_X;

    return true;
}

void event_proc_ui_in()
{
    static char* debug_string_state_name;

    static int latest_joy_button_state=-1;

    static int latest_joy_up_state=-1;
    static int latest_joy_down_state=-1;
    static int latest_joy_left_state=-1;
    static int latest_joy_right_state=-1;

    int joy_button_state = joy_btn.read();

    int joy_up_state = joy_up.read();
    int joy_down_state = joy_down.read();
    int joy_left_state = joy_left.read();
    int joy_right_state = joy_right.read();

    if(latest_joy_button_state==-1) latest_joy_button_state=joy_button_state;

    if(latest_joy_up_state==-1) latest_joy_up_state=joy_up_state;
    if(latest_joy_down_state==-1) latest_joy_down_state=joy_down_state;
    if(latest_joy_left_state==-1) latest_joy_left_state=joy_left_state;
    if(latest_joy_right_state==-1) latest_joy_right_state=joy_right_state;

    if (joy_button_state != latest_joy_button_state)
    {
        printf("[TASK - UI INPUT] Pulsante Jostick : %s\n", joy_button_state ? "PUSHED" : "RELEASED");

        if(!joy_button_state)
        {
            ui_out_state_type new_state;

            s_mutex_ui_out_state.lock();
            ui_out_state=static_cast<ui_out_state_type>(ui_out_state+1);
            if(ui_out_state==UI_OUT_STATE_END) ui_out_state=static_cast<ui_out_state_type>(UI_OUT_STATE_BEGIN+1);
            new_state=ui_out_state;
            s_mutex_ui_out_state.unlock();

            switch(new_state)
            {
                case UI_OUT_STATE_GYR:
                debug_string_state_name=const_cast<char*>("GYR");
                break;

                case UI_OUT_STATE_MAG:
                debug_string_state_name=const_cast<char*>("MAG");
                break;

                case UI_OUT_STATE_ACC:
                debug_string_state_name=const_cast<char*>("ACC");
                break;

                default:
                debug_string_state_name=const_cast<char*>("???");
                break;
            }

            printf("[TASK - UI INPUT] Nuovo stato UI : %s\n", debug_string_state_name);

            if(!save_state()) printf("Salvataggio stato fallito!");
        }

        latest_joy_button_state = joy_button_state;
    }

    if (joy_up_state != latest_joy_up_state)
    {
        printf("[TASK - UI INPUT] Jostick UP : %s\n", joy_up_state ? "PUSHED" : "RELEASED");

        if(joy_up_state)
        {
            ui_out_axis_type new_axis;

            s_mutex_ui_out_axis.lock();
            ui_out_axis=static_cast<ui_out_axis_type>(ui_out_axis+1);
            if(ui_out_axis==UI_OUT_AXIS_END) ui_out_axis=static_cast<ui_out_axis_type>(UI_OUT_AXIS_BEGIN+1);
            new_axis=ui_out_axis;
            s_mutex_ui_out_axis.unlock();

            printf("[TASK - UI INPUT] Nuovo asse UI : %c\n", 'X'+static_cast<int>(new_axis)-1);

            if(!save_state()) printf("Salvataggio stato fallito!");
        }

        latest_joy_up_state = joy_up_state;
    }
    else if (joy_down_state != latest_joy_down_state)
    {
        printf("[TASK - UI INPUT] Jostick DOWN : %s\n", joy_down_state ? "PUSHED" : "RELEASED");

        if(joy_down_state)
        {
            ui_out_axis_type new_axis;

            s_mutex_ui_out_axis.lock();
            ui_out_axis=static_cast<ui_out_axis_type>(ui_out_axis-1);
            if(ui_out_axis==UI_OUT_AXIS_BEGIN) ui_out_axis=static_cast<ui_out_axis_type>(UI_OUT_AXIS_END-1);
            new_axis=ui_out_axis;
            s_mutex_ui_out_axis.unlock();

            printf("[TASK - UI INPUT] Nuovo asse UI : %c\n", 'X'+static_cast<int>(new_axis)-1);

            if(!save_state()) printf("Salvataggio stato fallito!");
        }

        latest_joy_down_state = joy_down_state;
    }
}

void manage_ui_state_gyr(const ui_out_axis_type axis, const char* buffer)
{
    static float gyr_data_ui_buffer[3];

    s_mutex_gyr_data.lock();
    gyr_data_ui_buffer[0]=s_gyr_data[0];
    gyr_data_ui_buffer[1]=s_gyr_data[1];
    gyr_data_ui_buffer[2]=s_gyr_data[2];
    s_mutex_gyr_data.unlock();

    sprintf(const_cast<char*>(buffer),"G%c%4d",'X'+static_cast<int>(axis)-1,static_cast<int>(gyr_data_ui_buffer[static_cast<int>(axis)-1]/1000.0));

    led_red.write(gyr_data_ui_buffer[static_cast<int>(axis)-1]>50000.0 || gyr_data_ui_buffer[static_cast<int>(axis)-1]<-50000.0);
}

void manage_ui_state_mag(const ui_out_axis_type axis, const char* buffer)
{
    static int16_t mag_data_ui_buffer[3];

    s_mutex_mag_data.lock();
    mag_data_ui_buffer[0]=s_mag_data[0];
    mag_data_ui_buffer[1]=s_mag_data[1];
    mag_data_ui_buffer[2]=s_mag_data[2];
    s_mutex_mag_data.unlock();

    sprintf(const_cast<char*>(buffer),"M%c%4d",'X'+static_cast<int>(axis)-1,static_cast<int>(mag_data_ui_buffer[static_cast<int>(axis)-1]/2));

    led_red.write(mag_data_ui_buffer[static_cast<int>(axis)-1]<30 && mag_data_ui_buffer[static_cast<int>(axis)-1]>-30);
}

void manage_ui_state_acc(const ui_out_axis_type axis, const char* buffer)
{
    static int16_t acc_data_ui_buffer[3];

    s_mutex_acc_data.lock();
    acc_data_ui_buffer[0]=s_acc_data[0];
    acc_data_ui_buffer[1]=s_acc_data[1];
    acc_data_ui_buffer[2]=s_acc_data[2];
    s_mutex_acc_data.unlock();

    sprintf(const_cast<char*>(buffer),"A%c%4d",'X'+static_cast<int>(axis)-1,static_cast<int>(acc_data_ui_buffer[static_cast<int>(axis)-1]/16));

    led_red.write(acc_data_ui_buffer[static_cast<int>(axis)-1]<1050*16 && acc_data_ui_buffer[static_cast<int>(axis)-1]>950*16);
}

void event_proc_ui_out()
{
    static char buffer[16];
    
    ui_out_state_type current_state;
    ui_out_axis_type current_axis;

    s_mutex_ui_out_state.lock();
    current_state=ui_out_state;
    s_mutex_ui_out_state.unlock();

    s_mutex_ui_out_axis.lock();
    current_axis=ui_out_axis;
    s_mutex_ui_out_axis.unlock();

    switch(current_state)
    {
        case UI_OUT_STATE_GYR:
        manage_ui_state_gyr(current_axis,buffer);
        break;

        case UI_OUT_STATE_MAG:
        manage_ui_state_mag(current_axis,buffer);
        break;

        case UI_OUT_STATE_ACC:
        manage_ui_state_acc(current_axis,buffer);
        break;

        default:
        break;
    }

    lcd.Clear();
    lcd.DisplayString(reinterpret_cast<uint8_t*>(buffer));
}

void event_proc_trace()
{
    led_green.write(!led_green.read());
}

void event_proc_sensors()
{
    static float gyr_buffer[3];

    gyr.GetXYZ(gyr_buffer);

    s_mutex_gyr_data.lock();
    s_gyr_data[0]=gyr_buffer[0];
    s_gyr_data[1]=gyr_buffer[1];
    s_gyr_data[2]=gyr_buffer[2];
    s_mutex_gyr_data.unlock();

    static int16_t mag_buffer[3];

    compass.MagGetXYZ(mag_buffer);

    s_mutex_mag_data.lock();
    s_mag_data[0]=mag_buffer[0];
    s_mag_data[1]=mag_buffer[1];
    s_mag_data[2]=mag_buffer[2];
    s_mutex_mag_data.unlock();

    static int16_t acc_buffer[3];
    
    compass.AccGetXYZ(acc_buffer);

    s_mutex_acc_data.lock();
    s_acc_data[0]=acc_buffer[0];
    s_acc_data[1]=acc_buffer[1];
    s_acc_data[2]=acc_buffer[2];
    s_mutex_acc_data.unlock();
}

bool initialize_hardware()
{
    // GPIO
    joy_up.mode(PullDown);
    joy_down.mode(PullDown);
    joy_left.mode(PullDown);
    joy_right.mode(PullDown);

    // QSPI
    QSPI_Info pQSPI_Info;
    
    // Check initialization
    if (qspi.Init() != QSPI_OK)
    {
      printf("QSPI Initialization FAILED\n");
      return false;
    }
    
    // Check memory informations
    qspi.GetInfo(&pQSPI_Info);

    if ((pQSPI_Info.FlashSize          != N25Q128A_FLASH_SIZE) ||
        (pQSPI_Info.EraseSectorSize    != N25Q128A_SUBSECTOR_SIZE) || 
        (pQSPI_Info.ProgPageSize       != N25Q128A_PAGE_SIZE) ||
        (pQSPI_Info.EraseSectorsNumber != N25Q128A_SUBSECTOR_SIZE) ||
        (pQSPI_Info.ProgPagesNumber    != N25Q128A_SECTOR_SIZE))
    {
      printf("QSPI informations FAILED\n");
      return false;
    }
    
    return true;
}

void manage_perstisted_state()
{
    if(joy_btn.read())
    {
        printf("...cancellazione flash QSPI...");
        
        if(qspi_erase()) printf("...fatto\n");
        else printf("...fallito!\n");

        return;
    }

    if(!load_state()) printf("Ripristino stato iniziale fallito!");
}

/******************************/

int main()
{
    printf("[MAIN] Started...\n");

    if(!initialize_hardware())
    {
        led_red.write(true);
        led_green.write(false);

        while(true)
        {
            led_red.write(!led_red.read());
            led_green.write(!led_green.read());

            wait_ms(100);
        }
    }

    manage_perstisted_state();

    s_timer_1.start();

    s_eq_trace.call_every(500, event_proc_trace);
    s_eq_sensors.call_every(100, event_proc_sensors);
    s_eq_ui_in.call_every(50,event_proc_ui_in);
    s_eq_ui_out.call_every(100,event_proc_ui_out);

    s_thread_trace.start(callback(&s_eq_trace, &EventQueue::dispatch_forever));
    s_thread_sensors.start(callback(&s_eq_sensors, &EventQueue::dispatch_forever));
    s_thread_ui_out.start(callback(&s_eq_ui_out, &EventQueue::dispatch_forever));
    s_thread_ui_in.start(callback(&s_eq_ui_in, &EventQueue::dispatch_forever));
}
