//
// Created by 34147 on 2024/2/21.
//

#ifndef ENGINEER_CHASSIS_2024_DRV_UI_H
#define ENGINEER_CHASSIS_2024_DRV_UI_H

#include "user_lib.h"

#include "task_communicate.h"

#ifdef __cplusplus
extern "C" {
#endif
//C






#ifdef __cplusplus
}
#endif
//C++

#include "drv_pc.h"
typedef struct {
  struct{
    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
  }joint_states;

  struct{
    bool is_arm_pump_on;
    bool is_left_pump_on;
    bool is_right_pump_on;
    bool is_arm_pump_holding;
    bool is_left_pump_holding;
    bool is_right_pump_holding;
  }pump_states;

  robot_error_u error;
  robot_auto_situation_e auto_situation ;

} ui_data_t;

class ui_device {
 private:
  judgement_device *judgement;

  const std::string error_types[none + 1] = {
      "remote" ,
      "pc",
      "comm",
      "ctrl",
      "arm1",
      "arm2",
      "arm3",
      "arm4",
      "arm5",
      "arm6",
      "imu",
      "chas1",
      "chas2",
      "chas3",
      "chas4",
      "none",};

  const std::string auto_situations[arm_homing + 1] = {
      "NO",
      "SI",
      "BI",
      "EX",
      "CT",
      "GM",
      "AH"
  };

 public:
/*----------------------------------绘制UI所需要的数据---------------------------*/

  ui_data_t data;

/*------------------------------------绘制UI需要的变量--------------------------*/
  ext_client_custom_graphic_delete_t Delete_Graph;
  ext_client_custom_graphic_single_t Single_Graph;
  ext_client_custom_graphic_double_t Double_Graph;
  ext_client_custom_graphic_five_t Five_Graph;
  ext_client_custom_graphic_seven_t Seven_Graph;
  ext_client_custom_graphic_t Graph_Struct;

  ext_client_custom_character_t Character_Graph;

  bool clear_all_flag;
  ext_student_interactive_header_data_t student_interactive_header;
 public:
  ui_device();

  void update_data_send();

  void update_data(pc_device & pc);

  robot_error_type get_error_type() ;

  void ptr_init(judgement_device *judgement_ptr);

  void draw_arm(ui_operation operation_);

  void draw_assistant(ui_operation operation_);

  void draw_pump(ui_operation operation_);

  void add();

  void update();

  void send();

  void clear_all_UI();

  void character_update(ext_client_custom_character_t *_character);

  void character_init(ext_client_custom_character_t *_character);

  void ClearAll();

  void ClearLayer(uint32_t _layer);

  void show_line(graphic_data_struct_t *_graphic,
                 const char *name,
                 enum ui_operation _operation,
                 enum ui_color _color,
                 uint32_t _layer,
                 uint32_t _width,
                 uint32_t start_x,
                 uint32_t start_y,
                 uint32_t end_x,
                 uint32_t end_y);

  void show_rectangle(graphic_data_struct_t *_graphic,
                      const char *name,
                      enum ui_operation _operation,
                      enum ui_color _color,
                      uint32_t _layer,
                      uint32_t _width,
                      uint32_t start_x,
                      uint32_t start_y,
                      uint32_t end_x,
                      uint32_t end_y);

  void show_circle(graphic_data_struct_t *_graphic,
                   const char *name,
                   enum ui_operation _operation,
                   enum ui_color _color,
                   uint32_t _layer,
                   uint32_t _width,
                   uint32_t center_x,
                   uint32_t center_y,
                   uint32_t _radius);

  void show_oval(graphic_data_struct_t *_graphic,
                 const char *name,
                 enum ui_operation _operation,
                 enum ui_color _color,
                 uint32_t _layer,
                 uint32_t _width,
                 uint32_t center_x,
                 uint32_t center_y,
                 uint32_t x_axis_len,
                 uint32_t y_axis_len);

  void show_arc(graphic_data_struct_t *_graphic,
                const char *name,
                enum ui_operation _operation,
                enum ui_color _color,
                uint32_t _layer,
                uint32_t _width,
                uint32_t center_x,
                uint32_t center_y,
                uint32_t start_ang,
                uint32_t end_ang,
                uint32_t x_axis_len,
                uint32_t y_axis_len);

  void show_floating(graphic_data_struct_t *_graphic,
                     const char *name,
                     enum ui_operation _operation,
                     enum ui_color _color,
                     uint32_t _layer,
                     uint32_t _width,
                     uint32_t start_x,
                     uint32_t start_y,
                     uint32_t font_size,
                     uint32_t signi_digits,
                     float num);

  void show_integer(graphic_data_struct_t *_graphic,
                    const char *name,
                    enum ui_operation _operation,
                    enum ui_color _color,
                    uint32_t _layer,
                    uint32_t _width,
                    uint32_t start_x,
                    uint32_t start_y,
                    uint32_t font_size,
                    int32_t num);

  void character_config(graphic_data_struct_t *_character,
                        const char *name,
                        enum ui_operation _operation,
                        uint32_t _layer,
                        enum ui_color _color,
                        uint32_t font_size,
                        uint32_t _len,
                        uint32_t _width,
                        uint32_t start_x,
                        uint32_t start_y);

};

#endif //ENGINEER_CHASSIS_2024_DRV_UI_H
