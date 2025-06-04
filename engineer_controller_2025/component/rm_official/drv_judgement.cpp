//
// Created by 34147 on 2024/2/22.
//
#include <cstring>
#include "drv_judgement.h"
#include "crc.h"
#include "task_communicate.h"

#define USE_JUDEG_UPDATE 0

// ---------------------------------------------- 外设初始化 ---------------------------------------------- //
void judgement_device::init(UART_HandleTypeDef *huart,
                            osEventFlagsId_t event,
                            const osMutexAttr_t *rx_attr,
                            const osMutexAttr_t *tx_attr) {
    /* create the judge_rxdata_mutex mutex  */
    this->judge_rxdata_mutex = osMutexNew(rx_attr);
    /* create the judge_txdata_mutex mutex  */
    this->judge_txdata_mutex = osMutexNew(tx_attr);
    /* judge data fifo init */
    fifo_s_init(&this->judge_rxdata_fifo, this->judge_rxdata_buf, JUDGE_FIFO_BUF_LEN, this->judge_rxdata_mutex);
    fifo_s_init(&this->judge_txdata_fifo, this->judge_txdata_buf, JUDGE_FIFO_BUF_LEN, this->judge_txdata_mutex);

    this->usart.huart = huart;

    this->usart.event = event;
    this->usart.tx_finish_flag = 1;
    /* initial judge data dma receiver object */

    this->usart.uart_dma_rxdata.data_fifo = &this->judge_rxdata_fifo;
    this->usart.uart_dma_rxdata.buff_size = UART_RX_DMA_SIZE;
    this->usart.uart_dma_rxdata.buff = this->judge_dma_rxbuff;

    /* initial judge data unpack object */
    this->judge_unpack_obj.data_fifo = &this->judge_rxdata_fifo;
    this->judge_unpack_obj.p_header = (frame_header_t *) this->judge_unpack_obj.protocol_packet;
    this->judge_unpack_obj.index = 0;
    this->judge_unpack_obj.data_len = 0;
    this->judge_unpack_obj.unpack_step = STEP_HEADER_SOF;

    usart_communicate_config(&this->usart);
}

/** ---------------------------------------------- 裁判系统接收 ---------------------------------------------- */
// ---------------------------------------------- 裁判系统接收数据的服务处理函数 ---------------------------------------------- //
void judgement_device::data_handler(uint8_t *p_frame) {
    auto *p_header = (frame_header_t *) p_frame;
    memcpy(p_header, p_frame, HEADER_LEN);

    uint16_t data_length = p_header->data_length;
    uint16_t cmd_id = *(uint16_t *) (p_frame + HEADER_LEN);
    uint8_t *data_addr = p_frame + HEADER_LEN + CMD_LEN;

    switch (cmd_id) {
        case GAME_STATE_ID:memcpy(&this->judge_rece_mesg.game_state_data, data_addr, data_length);
            break;
        case GAME_RESULT_ID:memcpy(&this->judge_rece_mesg.game_result_data, data_addr, data_length);
            break;
        case GAME_SURVIVORS_ID://机器人血量数据
            memcpy(&this->judge_rece_mesg.ext_game_robot_HP_t, data_addr, data_length);
            break;
        case EVENT_DATA_ID:memcpy(&this->judge_rece_mesg.event_data, data_addr, data_length);
            break;
        case REFEREE_WARNING_ID://裁判系统警告ID
            memcpy(&this->judge_rece_mesg.referee_warning_data, data_addr, data_length);
            break;
        case DART_REMAINING_ID://飞镖发射口计时
            memcpy(&this->judge_rece_mesg.dart_remaining_data, data_addr, data_length);
            break;
        case GAME_ROBOT_STATE_ID://比赛机器人状态，10HZ
            memcpy(&this->judge_rece_mesg.game_robot_state_data, data_addr, data_length);
#if USE_JUDEG_UPDATE
            Do_heat_calibration = 1;
            TargetBigBulletSpeed=this->judge_rece_mesg.game_robot_state_data.shooter_id1_42mm_speed_limit;//输入子弹初速度上限
#endif//第一次接收到机器人信息才会定时进行热量校准
            break;
        case POWER_HEAT_ID://实时功率热量数据，50HZ
            memcpy(&this->judge_rece_mesg.power_heat_data, data_addr, data_length);
            break;
        case ROBOT_POS_ID://机器人位置，10HZ
            memcpy(&this->judge_rece_mesg.game_robot_pos_data, data_addr, data_length);
            break;
        case BUFF_MUSK_ID://机器人增益，状态改变后发送
            memcpy(&this->judge_rece_mesg.buff_musk_data, data_addr, data_length);
            break;
        case ROBOT_HURT_ID://伤害状态，伤害发生后发送
            memcpy(&this->judge_rece_mesg.robot_hurt_data, data_addr, data_length);
            break;
        case SHOOT_DATA_ID://实时射击信息，射击后发送
            memcpy(&this->judge_rece_mesg.shoot_data, data_addr, data_length);
#if USE_JUDEG_UPDATE
            switch(this->judge_rece_mesg.shoot_data.bullet_type)
            {
                case 1:
                    this->SmallBulletShotNum++;
                    this->SmalBulletSped = (int16_t)(this->judge_rece_mesg.shoot_data.bullet_speed * 1000.0f);
                    this->SmallGim_Heat_User += this->judge_rece_mesg.shoot_data.bullet_speed;//小子弹用户热量更新
                    break;
                case 2:
                   this-> BigBulletShotNum++;
                    if(this->BigBulletShotNum   !=this-> BigBulletShotNumLast)
                    {
                       this-> BigBulletShotNumLast = BigBulletShotNum;
                       this-> BigBulletIncreFlag   = 1;//用于反馈发弹
                       this-> BigBulletIncreFlag_2 = 1;//用于摩擦轮速度调整
//							if(!DeathModeFlag)
//							{
//								BigGimHeat_User   += 100;
//							}
                    }
                   this-> BigBulletActualSpeed = this->judge_rece_mesg.shoot_data.bullet_speed*1000;
                   this-> BigBulletSped = (int16_t)(this->judge_rece_mesg.shoot_data.bullet_speed * 1000.0f);
                    break;
                default:
                    break;
            }
#endif
            break;
        case STU_INTERACTIVE_ID://交互数据接收信息
            memcpy(&this->judge_rece_mesg.student_student_data, data_addr, data_length);
            break;
        case BULLET_REMAIN_ID://剩余子弹信息
            memcpy(&this->judge_rece_mesg.bullet_remaining_data, data_addr, data_length);
            break;
        case RFID_STATUS_ID://机器人RFID状态
            memcpy(&this->judge_rece_mesg.rfid_status_data, data_addr, data_length);
            break;
        case DART_CMD_ID://飞镖机器人客户端指令数据
            memcpy(&this->judge_rece_mesg.dart_cmd_data, data_addr, data_length);
            break;
        case CUSTOM_CONTROLLER_ID://自定义控制器交互数据接口
            memcpy(&this->judge_rece_mesg.custom_interactive_data, data_addr, data_length);
            break;
        case MINIMAP_DATA_ID://客户端小地图交互数据
            memcpy(&this->judge_rece_mesg.mini_map_data, data_addr, data_length);
            break;
        case KEYBOARD_DATA_ID://键盘鼠标信息
            memcpy(&this->judge_rece_mesg.interactive_information_data, data_addr, data_length);
            break;
        default:break;
    }
}

// ---------------------------------------------- 打包与解包 ---------------------------------------------- //
/**
  * @brief  解包
  */
void judgement_device::unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof) {
    uint8_t byte = 0;

    while (fifo_used_count(p_obj->data_fifo)) {
        byte = fifo_s_get(p_obj->data_fifo);
        switch (p_obj->unpack_step) {
            case STEP_HEADER_SOF: {
                if (byte == sof) {
                    p_obj->unpack_step = STEP_LENGTH_LOW;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } else {
                    p_obj->index = 0;
                }
            }
                break;

            case STEP_LENGTH_LOW: {
                p_obj->data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_LENGTH_HIGH;
            }
                break;

            case STEP_LENGTH_HIGH: {
                p_obj->data_len |= (byte << 8);
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->data_len < (PROTOCOL_DATA_MAX_SIZE)) {
                    p_obj->unpack_step = STEP_FRAME_SEQ;
                } else {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
                break;

            case STEP_FRAME_SEQ: {
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_HEADER_CRC8;
            }
                break;

            case STEP_HEADER_CRC8: {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->index == HEADER_LEN) {
                    if (verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN)) {
                        p_obj->unpack_step = STEP_DATA_CRC16;
                    } else {
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            }
                break;

            case STEP_DATA_CRC16: {
                if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN)) {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN)) {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;

                    if (verify_crc16_check_sum(p_obj->protocol_packet,
                                               HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN)) {
                        if (sof == DN_REG_ID) {
                            this->data_handler(p_obj->protocol_packet);
                        }
                    }
                }
            }
                break;

            default: {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
                break;
        }
    }
}

void judgement_device::dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type) {
    int16_t tmp_len;
    uint16_t remain_data_counter;
    uint8_t *pdata = dma_obj->buff;

    if (UART_IDLE_IT == it_type) {
        dma_obj->write_index = dma_obj->buff_size;
    } else if (UART_DMA_FULL_IT == it_type) {
        dma_obj->write_index = dma_obj->buff_size;
    }

    if (dma_obj->write_index < dma_obj->read_index) {
        dma_write_len = dma_obj->buff_size * 2 - dma_obj->read_index + dma_obj->write_index;

        tmp_len = dma_obj->buff_size * 2 - dma_obj->read_index;
        if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
            fifo_overflow = 1;
        else
            fifo_overflow = 0;
        dma_obj->read_index = 0;

        tmp_len = dma_obj->write_index;
        if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
            fifo_overflow = 1;
        else
            fifo_overflow = 0;
        dma_obj->read_index = dma_obj->write_index;
    } else {
        dma_write_len = dma_obj->write_index - dma_obj->read_index;

        tmp_len = dma_obj->write_index - dma_obj->read_index;
        if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
            fifo_overflow = 1;
        else
            fifo_overflow = 0;
        dma_obj->read_index = (dma_obj->write_index) % (dma_obj->buff_size * 2);
    }
}

/**
  * @brief  打包,用于发送数据，普通的机器人间数据发送给裁判系统
  */
uint8_t *judgement_device::protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t *_tx_buf,
                                                ext_student_interactive_header_data_t _student_interactive_header) {

    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN + INTERACTIVE_HEADER_LEN;
    frame_header_t *p_header = (frame_header_t *) _tx_buf;

    p_header->sof = DN_REG_ID;
    p_header->data_length = len + 6;
    p_header->seq = 0;

    //与上面等效，据说这种才能发送出去数据，上面直接赋值无法发送数据
    _tx_buf[0] = DN_REG_ID;
    _tx_buf[1] = (uint8_t) ((len + 6) & 0xff);
    _tx_buf[2] = (len + 6) >> 8;

    //p_header->seq          = seq++;
    memcpy(&_tx_buf[HEADER_LEN], (uint8_t *) &cmd_id, CMD_LEN);
    append_crc8_check_sum(_tx_buf, HEADER_LEN);
    memcpy(&_tx_buf[HEADER_LEN + CMD_LEN], (uint8_t *) &_student_interactive_header, INTERACTIVE_HEADER_LEN);
    memcpy(&_tx_buf[HEADER_LEN + CMD_LEN + INTERACTIVE_HEADER_LEN], p_data, len);
    append_crc16_check_sum(_tx_buf, frame_length);

    //实际串口发送出去的数组
    return _tx_buf;
}

//发送数据
uint32_t judgement_device::send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof) {
    uint8_t tx_buf[JUDGE_FIFO_BUF_LEN];
    uint32_t fifo_count = fifo_used_count(pfifo);
    if (fifo_count) {
        fifo_s_gets(pfifo, tx_buf, fifo_count);
        if (sof == DN_REG_ID)
            usart_dma_send(&this->usart, tx_buf, fifo_count);
        else
            return 0;
    }
    return fifo_count;
}

void judgement_device::data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len,
                                        ext_student_interactive_header_data_t _student_interactive_header) {

    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN + INTERACTIVE_HEADER_LEN;   //5+2+4+2

    protocol_packet_pack(cmd_id, p_data, len, tx_buf, _student_interactive_header);

    /* use mutex operation */
    fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);
}

// ----------------------------------------- 雷达数据打包 ----------------------------------------- //
uint8_t *judgement_device::lidar_protocol_packet_pack(uint16_t cmd_id, lidar_send_struct lidar_data, uint8_t *_tx_buf,
                                                      ext_student_interactive_header_data_t _student_interactive_header) {
    //memset(tx_buf, 0, 100);
    //static uint8_t seq;

    uint16_t frame_length = HEADER_LEN + CMD_LEN + 14 + CRC_LEN;
    //uint16_t frame_length = HEADER_LEN + CMD_LEN + 14 + CRC_LEN + INTERACTIVE_HEADER_LEN;
    frame_header_t *p_header = (frame_header_t *) _tx_buf;

    p_header->sof = DN_REG_ID;
    p_header->data_length = 14;
    p_header->seq = 0;
    //p_header->seq          = seq++;
    memcpy(&_tx_buf[HEADER_LEN], (uint8_t *) &cmd_id, CMD_LEN);
    append_crc8_check_sum(_tx_buf, HEADER_LEN);
    memcpy(&_tx_buf[HEADER_LEN + CMD_LEN], (uint8_t *) &_student_interactive_header, INTERACTIVE_HEADER_LEN);
    memcpy(&_tx_buf[HEADER_LEN + CMD_LEN + INTERACTIVE_HEADER_LEN], (uint8_t *) &lidar_data, 14);
    //memcpy(&tx_buf[HEADER_LEN + CMD_LEN], (uint8_t*)&lidar_data,14);
    append_crc16_check_sum(_tx_buf, frame_length);

    return _tx_buf;
}

void judgement_device::lidar_data_pack(uint16_t cmd_id, lidar_send_struct lidar_data,
                                       ext_student_interactive_header_data_t _student_interactive_header) {
    //uint16_t frame_length = HEADER_LEN + CMD_LEN + 14 + CRC_LEN + INTERACTIVE_HEADER_LEN;  //雷达发送数据固定长度为14，标志头长度+命令长度+交互长度
    uint16_t frame_length = HEADER_LEN + CMD_LEN + 14 + CRC_LEN;

    lidar_protocol_packet_pack(cmd_id, lidar_data, tx_buf, _student_interactive_header);

    fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);
}

// ----------------------------------------- 车间通信发送代码 ----------------------------------------- //

void judgement_device::robot_communicate_send_data(uint16_t data_cmd_id,
                                                   uint16_t sender_ID,
                                                   uint16_t receiver_ID,
                                                   uint8_t *_p_data,
                                                   uint16_t _len) {
    ext_student_interactive_header_data_t robot_interactive;
    robot_interactive.data_cmd_id = data_cmd_id;
    robot_interactive.sender_ID = sender_ID;
    robot_interactive.receiver_ID = receiver_ID;
    data_packet_pack(STU_INTERACTIVE_ID, _p_data, _len, robot_interactive);
}

// ----------------------------------------- 车间通信 ----------------------------------------- //


void judgement_device::robot_communicate_init() {
    robot_communication_p_data = robot_communication_data;
    robot_communication_len = 3;
    robot_interactive.data_cmd_id = 0x02F0;
    robot_interactive.sender_ID = BLUE_INFANTRY4_ID;
    robot_interactive.receiver_ID = BLUE_HERO_ID;
}

// ----------------------------------------- 雷达发送数据初始化 ----------------------------------------- //


void judgement_device::lidar_send_init() {
    lidar_robot_interactive_header.data_cmd_id = 0x02F0;
    lidar_robot_interactive_header.receiver_ID = 0x016A;
    lidar_robot_interactive_header.sender_ID = 0x006D;

    lidar_data.target_robot_id = RED_HERO_ID;
    lidar_data.target_position_x = 5.1f;
    lidar_data.target_position_y = 5.1f;
    lidar_data.target_state_point = 0;
}

void judgement_device::usart_rx_processed(uint16_t Size) {
    auto _param = &this->usart;
    //    Rx_length = Size - Rx_buf_pos;
//    Rx_buf_pos += Rx_length;
//    if (Rx_buf_pos >= RX_BUF_SIZE) Rx_buf_pos = 0;	//缓冲区用完后，返回 0 处重新开始

    /* Check if number of received data in recpetion buffer has changed */
    if (Size != this->Rx_buf_pos) {
        /* Check if position of index in reception buffer has simply be increased
           of if end of buffer has been reached */
        if (Size > this->Rx_buf_pos) {
            /* Current position is higher than previous one */
            Rx_length = Size - this->Rx_buf_pos;
            /* Copy received data in "User" buffer for evacuation */
            fifo_s_puts(_param->uart_dma_rxdata.data_fifo, &_param->uart_dma_rxdata.buff[Rx_buf_pos], this->Rx_length);
        } else {
            /* Current position is lower than previous one : end of buffer has been reached */
            /* First copy data from current position till end of buffer */
            this->Rx_length = _param->uart_dma_rxdata.buff_size - this->Rx_buf_pos;
            /* Copy received data in "User" buffer for evacuation */
            fifo_s_puts(_param->uart_dma_rxdata.data_fifo, &_param->uart_dma_rxdata.buff[Rx_buf_pos], this->Rx_length);
            /* Check and continue with beginning of buffer */
            if (Size > 0) {
                fifo_s_puts(_param->uart_dma_rxdata.data_fifo, &_param->uart_dma_rxdata.buff[0], Size);
                this->Rx_length += Size;
            }
        }
    }
    /* Update old_pos as new reference of position in User Rx buffer that
       indicates position to which data have been processed */
    this->Rx_buf_pos = Size;
    osEventFlagsSet(_param->event, UART_IDLE_SIGNAL);
    /*
    if(LL_USART_IsActiveFlag_IDLE(_param->USARTx)){
        LL_USART_ClearFlag_IDLE(_param->USARTx);
        osEventFlagsSet(_param->event,UART_IDLE_SIGNAL);
    }*/
}


uint8_t *judgement_device::protocol_packet_pack_image_tran_chain(uint16_t cmd_id, uint8_t *p_data, uint16_t len,
                                                                 uint8_t *_tx_buf) {
    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;///NOTICE!
    frame_header_t *p_header = (frame_header_t *) _tx_buf;

    p_header->sof = DN_REG_ID;
    p_header->data_length = len;
    p_header->seq = 0;

    //p_header->seq          = seq++;
    memcpy(&_tx_buf[HEADER_LEN], (uint8_t *) &cmd_id, CMD_LEN);
    append_crc8_check_sum(_tx_buf, HEADER_LEN);
    memcpy(&_tx_buf[HEADER_LEN + CMD_LEN], p_data, len);            ///NOTICE!!
    append_crc16_check_sum(_tx_buf, frame_length);

    return _tx_buf;//实际串口发送出去的数组
}

void judgement_device::data_packet_pack_image_tran_chain(uint16_t cmd_id, uint8_t *p_data, uint16_t len) {
    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;   /// NOTICE!!!
    protocol_packet_pack_image_tran_chain(cmd_id, p_data, len, tx_buf);//把p_data里的数据打包给tx_buf
    /* use mutex operation */
    fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);//把打包数据放在fifo里
}

uint8_t *judgement_device::protocol_packet_pack_conventional_chain(uint16_t cmd_id, uint8_t *p_data, uint16_t len,
                                                                   uint8_t *_tx_buf,
                                                                   ext_student_interactive_header_data_t _student_interactive_header) {
    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN + INTERACTIVE_HEADER_LEN;
    frame_header_t *p_header = (frame_header_t *) _tx_buf;

    p_header->sof = DN_REG_ID;
    p_header->data_length = len + 6;
    p_header->seq = 0;

    //与上面等效，据说这种才能发送出去数据，上面直接赋值无法发送数据
    _tx_buf[0] = DN_REG_ID;
    _tx_buf[1] = (uint8_t) ((len + 6) & 0xff);
    _tx_buf[2] = (len + 6) >> 8;

    //p_header->seq          = seq++;
    memcpy(&_tx_buf[HEADER_LEN], (uint8_t *) &cmd_id, CMD_LEN);
    append_crc8_check_sum(_tx_buf, HEADER_LEN);
    memcpy(&_tx_buf[HEADER_LEN + CMD_LEN], (uint8_t *) &_student_interactive_header, INTERACTIVE_HEADER_LEN);
    memcpy(&_tx_buf[HEADER_LEN + CMD_LEN + INTERACTIVE_HEADER_LEN], p_data, len);
    append_crc16_check_sum(_tx_buf, frame_length);

    return _tx_buf;//实际串口发送出去的数组
}

void judgement_device::data_packet_pack_conventional_chain(uint16_t cmd_id, uint8_t *p_data, uint16_t len,
                                                           ext_student_interactive_header_data_t _student_interactive_header) {
    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN + INTERACTIVE_HEADER_LEN;   //5+2+4+2
    protocol_packet_pack_conventional_chain(cmd_id, p_data, len, tx_buf, _student_interactive_header);
    /* use mutex operation */
    fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);
}