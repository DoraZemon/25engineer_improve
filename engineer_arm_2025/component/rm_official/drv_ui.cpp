//
// Created by 34147 on 2024/2/21.
//

#include "drv_ui.h"
#include "cstring"
#include "cstdio"
#include "drv_judgement.h"

ui_device::ui_device() : clear_all_flag(false) {}

void ui_device::update_data_send() {
    if (this->judgement->judge_rece_mesg.game_robot_state_data.robot_id == RED_ENGINEER_ID) {//红6
        this->student_interactive_header.sender_ID = RED_ENGINEER_ID;
        this->student_interactive_header.receiver_ID = RED_ENGINEER_CUSTOM_ID;
        this->send();
    } else if (this->judgement->judge_rece_mesg.game_robot_state_data.robot_id == BLUE_ENGINEER_ID) {//蓝6
        this->student_interactive_header.sender_ID = BLUE_ENGINEER_ID;
        this->student_interactive_header.receiver_ID = BLUE_ENGINEER_CUSTOM_ID;
        this->send();
    }
}

void ui_device::ptr_init(judgement_device *judgement_ptr) {
    this->judgement = judgement_ptr;
}

void ui_device::update_data(pc_device &pc) {
    data.joint_states = {pc.normal_tx_data.joint1,
                         pc.normal_tx_data.joint2,
                         pc.normal_tx_data.joint3,
                         pc.normal_tx_data.joint4,
                         pc.normal_tx_data.joint5,
                         pc.normal_tx_data.joint6};
    data.error = pc.error;
    data.auto_situation = pc.auto_situation;
    data.pump_states.is_arm_pump_on = pc.rx_data.is_arm_pump_on;
    data.pump_states.is_left_pump_on = pc.rx_data.is_left_pump_on;
    data.pump_states.is_right_pump_on = pc.rx_data.is_right_pump_on;
    data.pump_states.is_arm_pump_holding = pc.normal_tx_data.is_arm_pump_holding_on;
    data.pump_states.is_left_pump_holding = pc.normal_tx_data.is_left_pump_holding_on;
    data.pump_states.is_right_pump_holding = pc.normal_tx_data.is_right_pump_holding_on;
}

robot_error_type ui_device::get_error_type() {
    for(int i = 0; i < none; i++) {
        if(data.error.code >> i & 0x01) {
            return (robot_error_type)i;
        }
    }

    return none;
}

void ui_device::add() {
    static uint8_t _ui_add_case = 0;

    _ui_add_case = (_ui_add_case + 1) % 30;
    if (_ui_add_case < 6) {
        draw_pump(ui_add);
    } else if (_ui_add_case < 12) {
        draw_assistant(ui_add);
    } else if (_ui_add_case < 30) {
        this->character_init(&Character_Graph);

    } else {
        _ui_add_case = 0;//没什么用 用作保护
    }
}

void ui_device::update() {
    static uint8_t _ui_update_case = 0;
    _ui_update_case = (_ui_update_case + 1) % 24;
    if (_ui_update_case < 6) {
        draw_pump(ui_modify);
    } else if (_ui_update_case < 18) {
        this->character_update(&Character_Graph);
    } else {
        _ui_update_case = 0;
    }
}

void ui_device::send() {
    static uint8_t _ui_send_case = 200;//先添加好多次,防止没添加上

    if (this->clear_all_flag) {
        this->clear_all_UI();
        return;
    }

    if (_ui_send_case > 50) {
        this->add();
        _ui_send_case--;
        return;
    }

    _ui_send_case = (_ui_send_case + 1) % 50;
    if (_ui_send_case % 10 == 0)  //5:1进行 add
        this->add();
    else
        this->update();
}


/// ----------------------------------------------- 分部发送 ----------------------------------------------- ///
void ui_device::character_init(ext_client_custom_character_t *_character) {
    static uint8_t _character_init_case = 0;
    for (int i = 0; i < 30; i++)
        _character->data[i] = '\0';

    _character_init_case = (_character_init_case + 1) % 6;

    if (_character_init_case < 3) {//30
        character_config(&_character->grapic_data_struct, "C1", ui_add, 2, ui_cyan, 15, 9, 2, 1500, 850);
        strcat(_character->data, "Error\n\n");
        strcat(_character->data, "State\n\n");

    } else if (_character_init_case < 6) {
        character_config(&_character->grapic_data_struct, "C2", ui_add, 2, ui_cyan, 15, 9, 2, 1700, 850);
        strcat(_character->data, "None\n\n");
        strcat(_character->data, "None\n\n");

    }
    student_interactive_header.data_cmd_id = STU_CUSTOM_CHARACTER_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Character_Graph, sizeof(Character_Graph),
                                student_interactive_header);
}

void ui_device::character_update(ext_client_custom_character_t *_character) {
    static uint8_t _character_update_case = 0;
    for (int i = 0; i < 30; i++)
        _character->data[i] = '\0';

    _character_update_case = (_character_update_case + 1) % 20;

//    if (_character_update_case % 2 == 0) {

        if(data.error.code == 0){
            character_config(&_character->grapic_data_struct, "C2", ui_modify, 2, ui_orange, 15, 9, 2, 1700, 850);
        }else {
            character_config(&_character->grapic_data_struct, "C2", ui_modify, 2, ui_yellow, 15, 9, 2, 1700, 850);
        }

        strcat(_character->data, error_types[this->get_error_type()].c_str());
        strcat(_character->data, "\n\n");
        strcat(_character->data, auto_situations[data.auto_situation].c_str());
        strcat(_character->data, "\n\n");

//    } else if (_character_update_case % 2 == 1) {
//
//
//    }
    student_interactive_header.data_cmd_id = STU_CUSTOM_CHARACTER_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Character_Graph, sizeof(Character_Graph),
                                student_interactive_header);
}

void ui_device::draw_arm(ui_operation operation_) {

}

void ui_device::draw_pump(ui_operation operation_) {


    if(data.pump_states.is_arm_pump_on && data.pump_states.is_arm_pump_holding) {
        show_circle(&Five_Graph.grapic_data_struct[0], "P1", operation_, ui_green, 1, 30, 1500, 650, 10);
    }else if(data.pump_states.is_arm_pump_on) {
        show_circle(&Five_Graph.grapic_data_struct[0], "P1", operation_, ui_yellow, 1, 30, 1500, 650, 10);
    }else{
        show_circle(&Five_Graph.grapic_data_struct[0], "P1", operation_, ui_black, 1, 30, 1500, 650, 10);
    }

    if(data.pump_states.is_left_pump_on && data.pump_states.is_left_pump_holding) {
        show_circle(&Five_Graph.grapic_data_struct[1], "P2", operation_, ui_green, 1, 30, 1600, 650, 10);
    }else if(data.pump_states.is_left_pump_on) {
        show_circle(&Five_Graph.grapic_data_struct[1], "P2", operation_, ui_yellow, 1, 30, 1600, 650, 10);
    }else{
        show_circle(&Five_Graph.grapic_data_struct[1], "P2", operation_, ui_black, 1, 30, 1600, 650, 10);
    }

    if(data.pump_states.is_right_pump_on && data.pump_states.is_right_pump_holding) {
        show_circle(&Five_Graph.grapic_data_struct[2], "P3", operation_, ui_green, 1, 30, 1700, 650, 10);
    }else if(data.pump_states.is_right_pump_on) {
        show_circle(&Five_Graph.grapic_data_struct[2], "P3", operation_, ui_yellow, 1, 30, 1700, 650, 10);
    }else{
        show_circle(&Five_Graph.grapic_data_struct[2], "P3", operation_, ui_black, 1, 30, 1700, 650, 10);
    }

    student_interactive_header.data_cmd_id = STU_CUSTOM_FIVE_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Five_Graph, sizeof(Five_Graph),
                                student_interactive_header);
}

/**
 * @brief 绘制取矿的辅助线
 */
void ui_device::draw_assistant(ui_operation operation_) {
    show_line(&Five_Graph.grapic_data_struct[0], "A1", operation_, ui_yellow, 0, 5, 1500, 850, 1500, 1000);
    show_line(&Five_Graph.grapic_data_struct[1], "A2", operation_, ui_yellow, 0, 5, 1500, 850, 1700, 850);
    show_line(&Five_Graph.grapic_data_struct[2], "A3", operation_, ui_yellow, 0, 5, 1500, 850, 1500, 700);
    show_line(&Five_Graph.grapic_data_struct[3], "A4", operation_, ui_yellow, 0, 5, 1500, 850, 1300, 850);
    show_line(&Five_Graph.grapic_data_struct[4], "A5", operation_, ui_yellow, 0, 5, 1500, 850, 1300, 850);

    student_interactive_header.data_cmd_id = STU_CUSTOM_FIVE_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Five_Graph, sizeof(Five_Graph),
                                student_interactive_header);

}


void ui_device::clear_all_UI(void) {
    Delete_Graph.operate_type = ui_clear;
    student_interactive_header.data_cmd_id = STU_CUSTOM_DELETE_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Delete_Graph, sizeof(Delete_Graph),
                                student_interactive_header);
}



/// ----------------------------------------------- 基本操作 ----------------------------------------------- ///
void ui_device::ClearAll(void) {
    Delete_Graph.operate_type = ui_delete_all;
    student_interactive_header.data_cmd_id = STU_CUSTOM_DELETE_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Delete_Graph, sizeof(Delete_Graph),
                                student_interactive_header);
}

void ui_device::ClearLayer(uint32_t _layer) {
    VAL_LIMIT(_layer, 0, 9);
    Delete_Graph.operate_type = ui_delete_layer;
    Delete_Graph.layer = _layer;
    student_interactive_header.data_cmd_id = STU_CUSTOM_DELETE_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Delete_Graph, sizeof(Delete_Graph),
                                student_interactive_header);
}

//现在是单个发送
// width 5
void
ui_device::show_line(graphic_data_struct_t *_graphic,
                     const char *name,
                     enum ui_operation _operation,
                     enum ui_color _color,
                     uint32_t _layer,
                     uint32_t _width,
                     uint32_t start_x,
                     uint32_t start_y,
                     uint32_t end_x,
                     uint32_t end_y) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);
    VAL_LIMIT(end_x, 0, 2048);
    VAL_LIMIT(end_y, 0, 2048);

    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_line;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = 0;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = start_x;
    _graphic->start_y = start_y;
    _graphic->radius = 0;
    _graphic->end_x = end_x;
    _graphic->end_y = end_y;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void ui_device::show_rectangle(graphic_data_struct_t *_graphic, const char *name, enum ui_operation _operation,
                               enum ui_color _color, uint32_t _layer, uint32_t _width,
                               uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);
    VAL_LIMIT(end_x, 0, 2048);
    VAL_LIMIT(end_y, 0, 2048);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_rectangle;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = 0;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = start_x;
    _graphic->start_y = start_y;
    _graphic->radius = 0;
    _graphic->end_x = end_x;
    _graphic->end_y = end_y;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void
ui_device::show_circle(graphic_data_struct_t *_graphic,
                       const char *name,
                       enum ui_operation _operation,
                       enum ui_color _color,
                       uint32_t _layer,
                       uint32_t _width,
                       uint32_t center_x,
                       uint32_t center_y,
                       uint32_t _radius) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(center_x, 0, 2048);
    VAL_LIMIT(center_y, 0, 2048);
    VAL_LIMIT(_radius, 0, 1024);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_circle;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = 0;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = center_x;
    _graphic->start_y = center_y;
    _graphic->radius = _radius;
    _graphic->end_x = 0;
    _graphic->end_y = 0;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void
ui_device::show_oval(graphic_data_struct_t *_graphic,
                     const char *name,
                     enum ui_operation _operation,
                     enum ui_color _color,
                     uint32_t _layer,
                     uint32_t _width,
                     uint32_t center_x,
                     uint32_t center_y,
                     uint32_t x_axis_len,
                     uint32_t y_axis_len) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(center_x, 0, 2048);
    VAL_LIMIT(center_y, 0, 2048);
    VAL_LIMIT(x_axis_len, 0, 2048);
    VAL_LIMIT(y_axis_len, 0, 2048);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_oval;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = 0;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = center_x;
    _graphic->start_y = center_y;
    _graphic->radius = 0;
    _graphic->end_x = x_axis_len;
    _graphic->end_y = y_axis_len;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void
ui_device::show_arc(graphic_data_struct_t *_graphic,
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
                    uint32_t y_axis_len) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(center_x, 0, 2048);
    VAL_LIMIT(center_y, 0, 2048);
    VAL_LIMIT(start_ang, 0, 360);
    VAL_LIMIT(end_ang, 0, 360);
    VAL_LIMIT(x_axis_len, 0, 2048);
    VAL_LIMIT(y_axis_len, 0, 2048);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_arc;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = start_ang;
    _graphic->end_angle = end_ang;
    _graphic->width = _width;
    _graphic->start_x = center_x;
    _graphic->start_y = center_y;
    _graphic->radius = 0;
    _graphic->end_x = x_axis_len;
    _graphic->end_y = y_axis_len;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void ui_device::show_floating(graphic_data_struct_t *_graphic, const char *name, enum ui_operation _operation,
                              enum ui_color _color, uint32_t _layer, uint32_t _width,
                              uint32_t start_x, uint32_t start_y, uint32_t font_size, uint32_t signi_digits,
                              float num) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);
    VAL_LIMIT(font_size, 0, 360);
    VAL_LIMIT(signi_digits, 0, 360);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_floating;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = font_size;
    _graphic->end_angle = signi_digits;
    _graphic->width = _width;
    _graphic->start_x = start_x;
    _graphic->start_y = start_y;
    _graphic->radius = (int32_t) (1000 * num);

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void
ui_device::show_integer(graphic_data_struct_t *_graphic,
                        const char *name,
                        enum ui_operation _operation,
                        enum ui_color _color,
                        uint32_t _layer,
                        uint32_t _width,
                        uint32_t start_x,
                        uint32_t start_y,
                        uint32_t font_size,
                        int32_t num) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);
    VAL_LIMIT(font_size, 0, 360);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_integer;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = font_size;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = start_x;
    _graphic->start_y = start_y;
    _graphic->radius = (int32_t) num;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

//font_size 20 _len 9 _width 2 layer 7
void ui_device::character_config(graphic_data_struct_t *_character, const char *name, enum ui_operation _operation,
                                 uint32_t _layer,
                                 enum ui_color _color, uint32_t font_size, uint32_t _len, uint32_t _width,
                                 uint32_t start_x, uint32_t start_y) {
    VAL_LIMIT(font_size, 0, 360);
    VAL_LIMIT(_len, 0, 360);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);

    student_interactive_header.data_cmd_id = STU_CUSTOM_CHARACTER_ID;//绘制字符

//    strcpy(Character_Graph.grapic_data_struct.graphic_name, "R1");
    memcpy(_character->graphic_name, name, 3);
    _character->graphic_type = ui_character;
    _character->operate_type = _operation;
    _character->layer = _layer;
    _character->color = _color;
    _character->start_angle = font_size;//字体大小
    _character->end_angle = _len;//字符长度
    _character->width = _width;//线条宽度
    _character->start_x = start_x;
    _character->start_y = start_y;
    _character->radius = 0;
    _character->end_x = 0;
    _character->end_y = 0;
}