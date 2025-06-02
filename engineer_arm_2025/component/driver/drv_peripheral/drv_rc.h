/**
  ******************************************************************************
  * @file           : drv_rc.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#ifndef DRV_RC_H_
#define DRV_RC_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "user_lib.h"
#include "rtos_inc.h"

#define DR16_BUFF_SIZE 18

/*拨杆状态变化事件的相关宏*/
#define RC_SW_L_AREA 0x0F
#define RC_SW_L_UP2MID (1 << 0U)
#define RC_SW_L_MID2UP (1 << 1U)
#define RC_SW_L_MID2DOWN (1 << 2U)
#define RC_SW_L_DOWN2MID (1<<3U)
#define RC_SW_R_AREA 0xF0
#define RC_SW_R_UP2MID (1 << 4U)
#define RC_SW_R_MID2UP (1 << 5U)
#define RC_SW_R_MID2DOWN (1 << 6U)
#define RC_SW_R_DOWN2MID (1<<7U)

/*拨杆状态相关*/
class gimbal_device;

enum RC_SW_STATE {
  RC_SW_LR_NONE = 0,
  RC_SW_L_UP,
  RC_SW_L_DOWN,
  RC_SW_L_MID,

  RC_SW_R_UP,
  RC_SW_R_DOWN,
  RC_SW_R_MID

};

enum RC_WHEEL_STATE {
  RC_WHEEL_NONE = 0,
  RC_WHEEL_UP,
  RC_WHEEL_DOWN
};

enum RC_BUTTON_STATE {
  RC_BUTTON_L_UP = 0,
  RC_BUTTON_L_DOWN,
  RC_BUTTON_R_UP,
  RC_BUTTON_R_DOWN

};

enum keyMap {
  keyW = 0,
  keyS,
  keyA,
  keyD,
  keySHIFT,
  keyCTRL,
  keyQ,
  keyE,
  keyR,
  keyF,
  keyG,
  keyZ,
  keyX,
  keyC,
  keyV,
  keyB,
  keyNum
};

enum KEY_DIR {
  dir_up = 0,
  dir_down

};
//单个拨杆的状态
enum RC_SW {
  RC_SW_NONE = 0,
  RC_SW_UP = 1,
  RC_SW_DOWN = 2,
  RC_SW_MID = 3
};

enum RC_BUTTON {
  RC_BUTTON_UP = 0,
  RC_BUTTON_DOWN = 1
};

#pragma pack(1)

typedef struct {
  float x;
  float y;
} rc_rocker_t;

typedef struct {
  float x;
  float y;
  float z;
  enum RC_BUTTON left_button;
  enum RC_BUTTON right_button;
} rc_mouse_t;

typedef struct {
  uint16_t W: 1;
  uint16_t S: 1;
  uint16_t A: 1;
  uint16_t D: 1;
  uint16_t SHIFT: 1;
  uint16_t CTRL: 1;
  uint16_t Q: 1;
  uint16_t E: 1;
  uint16_t R: 1;
  uint16_t F: 1;
  uint16_t G: 1;
  uint16_t Z: 1;
  uint16_t X: 1;
  uint16_t C: 1;
  uint16_t V: 1;
  uint16_t B: 1;
} rc_key_bit_t;
#pragma pack()

typedef struct {
  rc_rocker_t left_rocker;
  rc_rocker_t right_rocker;
  enum RC_SW left_sw;
  enum RC_SW right_sw;
  rc_mouse_t mouse;
  union {
    uint16_t key_code;
    rc_key_bit_t key_bit_state;//代表键盘按下的状态
  } kb;
  float wheel;
} rc_data_t;
/**
 * 键盘状态的变化量,表示一个动作
 */
typedef struct {
  uint16_t sw_act;//用前后各四位表示两个拨杆的四个动作行为
  uint16_t keys_up_act;
  uint16_t keys_down_act;
  uint8_t right_button_up_act;
  uint8_t right_button_down_act;
  uint8_t left_button_up_act;
  uint8_t left_button_down_act;
} rc_event_t;

#pragma pack(1)
typedef union {
  struct {
    uint16_t ch0: 11;
    uint16_t ch1: 11;
    uint16_t ch2: 11;
    uint16_t ch3: 11;
    uint16_t s1: 2;
    uint16_t s2: 2;
    int16_t mouse_x: 16;
    int16_t mouse_y: 16;
    int16_t mouse_z: 16;
    uint8_t left_click: 8;
    uint8_t right_click: 8;
    uint16_t keys: 16;
    uint16_t wheel: 16;
  };
  uint8_t buff[DR16_BUFF_SIZE];
} rc_raw_data_t;
#pragma pack()

class rc_device {
 private:
  bool lost_flag;
  bool ready_flag;
  //    uint8_t raw_buff[DR16_BUFF_SIZE];

  rc_data_t last_data;
  rc_event_t event;
 public:
  friend class kb_mu_device;

  UART_HandleTypeDef *huart;
  rc_raw_data_t raw_data;

  rc_device(UART_HandleTypeDef *huart);

  void update_data();

  void update_event();

  void update_ready();

  bool check_ready();

  void set_lost();

  void set_connect();

  bool check_lost() const;

  bool check_key_down_state(enum keyMap key);

  bool check_key_up_state(enum keyMap key);

  bool check_key_down_event(enum keyMap key);

  bool check_key_up_event(enum keyMap key);

  bool check_mouse_left_click_down_event();

  bool check_mouse_left_click_up_event();

  bool check_mouse_right_click_down_event();

  bool check_mouse_right_click_up_evnet();

  bool check_mouse_left_click_state() const;

  bool check_mouse_right_click_state() const;

  bool check_sw_state(enum RC_SW_STATE sw_state);

  bool check_sw_event(uint16_t sw_event);

  bool check_wheel_state(enum RC_WHEEL_STATE wheel_state);

  float get_mouse_x() const;

  float get_mouse_y() const;

  float get_mouse_z() const;

  float get_right_rocker_x() const;

  float get_right_rocker_y() const;

  float get_left_rocker_x() const;

  float get_left_rocker_y() const;

  rc_data_t data;
  bool using_kb_flag;
};

#endif //DRV_RC_H_
