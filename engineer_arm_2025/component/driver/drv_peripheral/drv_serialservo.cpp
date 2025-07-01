//
// Created by 34147 on 2024/2/11.
//

#include "drv_serialservo.h"

servo_t servo_device::servos = {};

servo_device::servo_device(UART_HandleTypeDef *huart, uint32_t id)
    : id(id), servo_flag(true), load_flag(true), engine_flag(false), lost_flag(false), init_flag(true) {
    servos.huart = huart;
    servos.init_flag = true;
    servos.enable_flag = true;
    this->ctrl_data.id = id;

    if (id <= LOBOT_SERVO_NUM) {
        servos.id2dev[this->id] = this;
    }
}

void servo_device::set_pos(uint16_t set) {
    taskENTER_CRITICAL();
    this->ctrl_data.set_1000 = set;
    osMessageQueuePut(ServoCtrlQueueHandle, &this->ctrl_data, 0, 0);
    taskEXIT_CRITICAL();

}

//要解决收发不能同时READ中可能要改
//用不了结构体,因为长度都不一定,后面如果希望使用，可以使用一个长度较长的数组进行接收
//已控制的是id1和id2,所有已知的也都是id
#define LobotSerialWrite  usart_send_buf_dma   //若使用dma需要使能dma,尽量用dma不用等待,速度大概是2-3倍,差距很大
#define LobotSerialRead  usart_idle_receive_buf

void open_hook(servo_t *_servo) {//其实5已经很快,舵机追赶不上这个速度,所以和0没有区别
    if (_servo->enable_flag) {
        LobotSerialServoMoveSet(_servo, 1, 900, 5);//开救援400 //关救援900
        LobotSerialServoMoveSet(_servo, 2, 50, 5);//开救援600 //关救援50
    }
}

void close_hook(servo_t *_servo) {
    if (_servo->enable_flag) {
        LobotSerialServoMoveSet(_servo, 2, 600, 5);//开救援600 //关救援50
    }
}

//可以写一个在转之前设置角度的，确实就是可以转到那个位置，但是后面可以用两个函数来组合做，也很好理解

//_position 0~1
void LobotSerialServoSetPosf(servo_t *_servo, uint8_t _id, float _position_f, uint16_t _time_ms) {
    int16_t pos = (int16_t) (_position_f * LOBOT_SERVO_POS_MAX);
    LobotSerialServoMoveSet(_servo, _id, pos, _time_ms);
}

//_ang 0~240
void LobotSerialServoSetAng(servo_t *_servo, uint8_t _id, float _ang_f, uint16_t _time_ms) {
    int16_t pos = (int16_t) (_ang_f / LOBOT_SERVO_ANG_MAX * LOBOT_SERVO_POS_MAX);
    LobotSerialServoMoveSet(_servo, _id, pos, _time_ms);
}

//speed -1~1 不支持掉电保存
void LobotSerialServoSetEngineSpeedf(servo_t *_servo, uint8_t _id, float _speed_f) {
    int16_t _speed = (int16_t) (_speed_f * LOBOT_ENGINE_SPEED_MAX);
    LobotSerialServoSetEngineMode(_servo, _id, _speed);
}

//_ang 0~240 支持掉电保存
void LobotSerialServoAngLimit(servo_t *_servo, uint8_t _id, float _ang_min_f, float _ang_max_f) {
    uint16_t _pos_min, _pos_max;
    if (_ang_min_f <= _ang_max_f) {
        if (_ang_max_f > 240.0f) _ang_max_f = 240.0f;
        if (_ang_min_f < 0.0f) _ang_min_f = 0.0f;
        _pos_min = (uint16_t) (_ang_min_f / LOBOT_SERVO_ANG_MAX * LOBOT_SERVO_POS_MAX);
        _pos_max = (uint16_t) (_ang_max_f / LOBOT_SERVO_ANG_MAX * LOBOT_SERVO_POS_MAX);
        LobotSerialServoPosLimit(_servo, _id, _pos_min, _pos_max);
    }
}

//_ang -30~30 不支持掉电保存
void LobotSerialServoChangeAngOffset(servo_t *_servo, uint8_t _id, float _offset_ang_f) {
    int8_t _offset_pos = 0;
    if (_offset_ang_f > 30.0f) _offset_ang_f = 30.0f;
    if (_offset_ang_f < -30.0f) _offset_ang_f = -30.0f;
    _offset_pos = (int8_t) (_offset_ang_f / LOBOT_SERVO_ANG_MAX * LOBOT_SERVO_POS_MAX);
    LobotSerialServoChangePosOffset(_servo, _id, _offset_pos);
}

//_ang -30~30 掉电保存
void LobotSerialServoSetSaveAngOffset(servo_t *_servo, uint8_t _id, float _offset_ang_f) {
    LobotSerialServoChangeAngOffset(_servo, _id, _offset_ang_f);
    LobotSerialServoSaveOffset(_servo, _id);
}

//_pos -125~125 掉电保存
void LobotSerialServoSetSavePosOffset(servo_t *_servo, uint8_t _id, int8_t _offset_pos) {
    LobotSerialServoChangePosOffset(_servo, _id, _offset_pos);
    LobotSerialServoSaveOffset(_servo, _id);
}

//总线上只有一个舵机时使用其读ID
uint8_t LobotSerialReadID(servo_t *_servo) {
    return LobotSerialServoReadID(_servo, LOBOT_BROADCAST_ID);
}

/** ------------------------------------- BASIC FUNCTIONS ---------------------------------------- **/

uint8_t LobotCheckSum(uint8_t buf[]) {
    uint8_t i;
    uint16_t temp = 0;
    for (i = 2; i < buf[3] + 2; i++) {
        temp += buf[i];
    }
    temp = ~temp;
    i = (uint8_t) temp;
    return i;
}

bool LobotReadCheck(uint8_t rx_buf[], uint8_t _id, uint8_t _data_len, uint8_t _instruction) {

    if (unsigned_16(rx_buf) == 0x5555 &&
        rx_buf[2] == _id && rx_buf[3] == _data_len &&
        rx_buf[4] == _instruction &&
        LobotCheckSum(rx_buf) != rx_buf[rx_buf[3] + 2])
        return true;
    else
        return false;
}


/** ------------------------------------- WRITE ---------------------------------------- **/
//1 0-1000 0-30000
void LobotSerialServoMoveSet(servo_t *_servo, uint8_t _id, int16_t _position, uint16_t _time_ms) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[10];//static,这个用了static后如果不用osdelay就会只动一个爪子
    if (_position < 0) _position = 0;
    if (_position > 1000) _position = 1000;
    _time_ms = _time_ms > 30000 ? 30000 : _time_ms;
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 7;
    _buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
    _buf[5] = LOW_BYTE((uint16_t) _position);
    _buf[6] = HIGH_BYTE((uint16_t) _position);
    _buf[7] = LOW_BYTE(_time_ms);
    _buf[8] = HIGH_BYTE(_time_ms);
    _buf[9] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 10);
    _servo_dev->servo_mode.current_pos = _position;
    _servo_dev->servo_mode.current_ang = (float) _servo_dev->servo_mode.current_pos * LOBOT_SERVO_POS2ANG;
}

//7 0-1000  0-30000ms
void LobotSerialServoPreMoveSet(servo_t *_servo, uint8_t _id, int16_t _pre_pos, uint16_t _pre_time_ms) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[10];
    if (_pre_pos < 0) _pre_pos = 0;
    if (_pre_pos > 1000) _pre_pos = 1000;
    _pre_time_ms = _pre_time_ms > 30000 ? 30000 : _pre_time_ms;
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 7;
    _buf[4] = LOBOT_SERVO_MOVE_TIME_WAIT_WRITE;
    _buf[5] = LOW_BYTE((uint16_t) _pre_pos);
    _buf[6] = HIGH_BYTE((uint16_t) _pre_pos);
    _buf[7] = LOW_BYTE(_pre_time_ms);
    _buf[8] = HIGH_BYTE(_pre_time_ms);
    _buf[9] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 10);
    _servo_dev->servo_mode.pre_pos = (uint16_t) _pre_pos;
    _servo_dev->servo_mode.pre_ang = (float) _servo_dev->servo_mode.pre_pos * LOBOT_SERVO_POS2ANG;
    _servo_dev->servo_mode.need_start_flag = true;
}

//11
void LobotSerialServoStart(servo_t *_servo, uint8_t _id) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_MOVE_START;
    _buf[5] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 6);
    _servo_dev->servo_mode.current_pos = (int16_t) _servo_dev->servo_mode.pre_pos;
    _servo_dev->servo_mode.current_ang = _servo_dev->servo_mode.pre_ang;
    _servo_dev->servo_mode.need_start_flag = false;
}

//12
void LobotSerialServoStop(servo_t *_servo, uint8_t _id) {
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_MOVE_STOP;
    _buf[5] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 6);
}

//13 0~253 默认为1 掉电保存
void LobotSerialServoSetID(servo_t *_servo, uint8_t oldID, uint8_t newID) {
    servo_device_t *_servo_dev = _servo->id2dev[oldID];
    _servo->id2dev[oldID] = NULL;//原来的设为空
    uint8_t _buf[7];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = oldID;
    _buf[3] = 4;
    _buf[4] = LOBOT_SERVO_ID_WRITE;
    _buf[5] = newID;
    _buf[6] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 7);
    _servo_dev->id = newID;
    _servo->id2dev[_servo_dev->id] = _servo_dev;
}

//17 -125~125 不支持掉电保存
void LobotSerialServoChangePosOffset(servo_t *_servo, uint8_t _id, int8_t _offset_pos) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[7];
    if (_offset_pos > 125) _offset_pos = 125;
    if (_offset_pos < -125) _offset_pos = -125;
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 4;
    _buf[4] = LOBOT_SERVO_ANGLE_OFFSET_ADJUST;
    _buf[5] = (uint8_t) _offset_pos;
    _buf[6] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 7);
    _servo_dev->servo_mode.pos_offset = _offset_pos;
    _servo_dev->servo_mode.ang_offset = (float) _servo_dev->servo_mode.pos_offset * LOBOT_SERVO_POS2ANG;
}

//18
void LobotSerialServoSaveOffset(servo_t *_servo, uint8_t _id) {
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_ANGLE_OFFSET_WRITE;
    _buf[5] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 6);
}

//20支持掉电保存  0-1000
void LobotSerialServoPosLimit(servo_t *_servo, uint8_t _id, uint16_t _pos_min, uint16_t _pos_max) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[10];
    _pos_max = _pos_max > 1000 ? 1000 : _pos_max;
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 7;
    _buf[4] = LOBOT_SERVO_ANGLE_LIMIT_WRITE;
    _buf[5] = LOW_BYTE(_pos_min);
    _buf[6] = HIGH_BYTE(_pos_min);
    _buf[7] = LOW_BYTE(_pos_max);
    _buf[8] = HIGH_BYTE(_pos_max);
    _buf[9] = LobotCheckSum(_buf);
    if (_pos_min < _pos_max) {
        LobotSerialWrite(_servo->huart, _buf, 10);
        _servo_dev->servo_mode.pos_max = _pos_max;
        _servo_dev->servo_mode.pos_min = _pos_min;
    }

}

//22支持掉电保存  4500-12000mV
void LobotSerialServoVoltageLimit(servo_t *_servo, uint8_t _id, uint16_t _vol_min, uint16_t _vol_max) {
    uint8_t _buf[10];
    _vol_min = _vol_min < 4500 ? 4500 : _vol_min;
    _vol_max = _vol_max > 12000 ? 12000 : _vol_max;
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 7;
    _buf[4] = LOBOT_SERVO_VIN_LIMIT_WRITE;
    _buf[5] = LOW_BYTE(_vol_min);
    _buf[6] = HIGH_BYTE(_vol_min);
    _buf[7] = LOW_BYTE(_vol_max);
    _buf[8] = HIGH_BYTE(_vol_max);
    _buf[9] = LobotCheckSum(_buf);
    if (_vol_min < _vol_max) {
        LobotSerialWrite(_servo->huart, _buf, 10);
    }

}

//24支持掉电保存 50~100  默认85℃
void LobotSerialServoTempLimit(servo_t *_servo, uint8_t _id, uint8_t _temp_max) {
    uint8_t _buf[7];
    _temp_max = _temp_max < 50 ? 50 : (_temp_max > 100 ? 100 : _temp_max);
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 4;
    _buf[4] = LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE;
    _buf[5] = _temp_max;
    _buf[6] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 7);
}

//29 -1000-1000 不支持掉电保存
void LobotSerialServoSetEngineMode(servo_t *_servo, uint8_t _id, int16_t _speed) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[10];
    if (_speed > 1000) _speed = 1000;
    if (_speed < -1000) _speed = -1000;
    if (_servo_dev->engine_mode.reverse_flag) _speed = -_speed;
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 7;
    _buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
    _buf[5] = 1;
    _buf[6] = 0;
    _buf[7] = LOW_BYTE((uint16_t) _speed);
    _buf[8] = HIGH_BYTE((uint16_t) _speed);
    _buf[9] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 10);
    _servo_dev->engine_flag = true;
    _servo_dev->servo_flag = false;
    _servo_dev->engine_mode.speedf = (float) _speed / 1000.0f;
}

//29 不支持掉电保存
void LobotSerialServoSetServoMode(servo_t *_servo, uint8_t _id) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[10];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 7;
    _buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
    _buf[5] = 0;
    _buf[6] = _buf[7] = _buf[8] = 0;
    _buf[9] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 10);
    _servo_dev->servo_flag = true;
    _servo_dev->engine_flag = false;
}

//31
void LobotSerialServoSetUnload(servo_t *_servo, uint8_t _id) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[7];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 4;
    _buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
    _buf[5] = 0;
    _buf[6] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 7);
    _servo_dev->load_flag = false;
}

//31
void LobotSerialServoSetLoad(servo_t *_servo, uint8_t _id) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[7];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 4;
    _buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
    _buf[5] = 1;
    _buf[6] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 7);
    _servo_dev->load_flag = true;
}

//35
void LobotSerialServoErrorCfg(servo_t *_servo, uint8_t _id, uint8_t _error_type) {
    uint8_t _buf[7];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 4;
    _buf[4] = LOBOT_SERVO_LED_ERROR_WRITE;
    _buf[5] = _error_type;
    _buf[6] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 7);
}



/** ----------------------------------- READ ---------------------------------------- **/
//这里也许要用到判断
//if (DR16_BUFF_LEN - __HAL_DMA_GET_COUNTER(_rc_dev->_huart->hdmarx) == 18)

//14 _id应为254 只能有一个舵机
uint8_t LobotSerialServoReadID(servo_t *_servo, uint8_t _id) {
//    static osStatus_t stat = osError;
    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_ID_READ;
    _buf[5] = LobotCheckSum(_buf);
//    while(stat!=osOK && error_cnt < 8){
//        LobotSerialWrite(&SERVO_UART, _buf, 6);
//        LobotSerialRead(&SERVO_UART,rec_buf,rec_data_len+3);
//        stat = osSemaphoreAcquire(serialServoRecBinarySemHandle, 30);
//        if(stat==osOK && !LobotReadCheck(rec_buf,id,rec_data_len,LOBOT_SERVO_ID_READ)){
//            stat = osError;
//        }
//        error_cnt++;
//    }
    while (!LobotReadCheck(rec_buf, _id, rec_data_len, LOBOT_SERVO_ID_READ) && error_cnt < 30) {
        LobotSerialWrite(_servo->huart, _buf, 6);
        LobotSerialRead(_servo->huart, rec_buf, SERVO_BUFF_LEN);//rec_data_len+3
        //这个地方可以大一点,或者是取最大即可,给rec_buf可以直接给最大,不用特别标准
        error_cnt++;
    }
    if (error_cnt < 30)
        return rec_buf[5];//ID
    else
        return UINT8_ERROR_RETURN;
}

//19 -125~125
int8_t LobotSerialServoReadPosOffset(servo_t *_servo, uint8_t _id) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_ANGLE_OFFSET_READ;
    _buf[5] = LobotCheckSum(_buf);

    while (!LobotReadCheck(rec_buf, _id, rec_data_len, LOBOT_SERVO_ANGLE_OFFSET_READ) && error_cnt < 30) {
        LobotSerialWrite(_servo->huart, _buf, 6);
        LobotSerialRead(_servo->huart, rec_buf, SERVO_BUFF_LEN);//rec_data_len+3
        error_cnt++;
    }
    if (error_cnt < 30) {
        _servo_dev->servo_mode.pos_offset = (int8_t) rec_buf[5];//Offset
        return (int8_t) rec_buf[5];//Offset
    } else
        return INT8_ERROR_RETURN;
}

//26
uint8_t LobotSerialServoReadTemp(servo_t *_servo, uint8_t _id) {
    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_TEMP_READ;
    _buf[5] = LobotCheckSum(_buf);

    while (!LobotReadCheck(rec_buf, _id, rec_data_len, LOBOT_SERVO_TEMP_READ) && error_cnt < 30) {
        LobotSerialWrite(_servo->huart, _buf, 6);
        LobotSerialRead(_servo->huart, rec_buf, SERVO_BUFF_LEN);//rec_data_len+3
        error_cnt++;
    }
    if (error_cnt < 30)
        return rec_buf[5];//Temp
    else
        return UINT8_ERROR_RETURN;
}

//27
int16_t LobotSerialServoReadVoltage(servo_t *_servo, uint8_t _id) {
    uint8_t rec_data_len = 5, rec_buf[7] = {0}, error_cnt = 0;
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_VIN_READ;
    _buf[5] = LobotCheckSum(_buf);

    while (!LobotReadCheck(rec_buf, _id, rec_data_len, LOBOT_SERVO_VIN_READ) && error_cnt < 30) {
        LobotSerialWrite(_servo->huart, _buf, 6);
        LobotSerialRead(_servo->huart, rec_buf, SERVO_BUFF_LEN);//rec_data_len+3
        error_cnt++;
    }
    if (error_cnt < 30)
        return (int16_t) MERGE_BYTES(rec_buf[6], rec_buf[5]);
    else
        return INT16_ERROR_RETURN;
}

//28读出角度可能为负值
int16_t LobotSerialServoReadPosition(servo_t *_servo, uint8_t _id) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t rec_data_len = 5, rec_buf[7] = {0}, error_cnt = 0;
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_POS_READ;
    _buf[5] = LobotCheckSum(_buf);

    while (!LobotReadCheck(rec_buf, _id, rec_data_len, LOBOT_SERVO_POS_READ) && error_cnt < 30) {
        LobotSerialWrite(_servo->huart, _buf, 6);
        LobotSerialRead(_servo->huart, rec_buf, SERVO_BUFF_LEN);//rec_data_len+3
        error_cnt++;
    }
    if (error_cnt < 30) {
        _servo_dev->servo_mode.current_pos = (int16_t) MERGE_BYTES(rec_buf[6], rec_buf[5]);
        return _servo_dev->servo_mode.current_pos;
    } else
        return INT16_ERROR_RETURN;
}

//30 speed可写为NULL
uint8_t LobotSerialServoReadMode(servo_t *_servo, uint8_t _id, int16_t *_speed) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t rec_data_len = 7, rec_buf[7] = {0}, error_cnt = 0;
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_READ;
    _buf[5] = LobotCheckSum(_buf);

    while (!LobotReadCheck(rec_buf, _id, rec_data_len, LOBOT_SERVO_OR_MOTOR_MODE_READ) && error_cnt < 30) {
        LobotSerialWrite(_servo->huart, _buf, 6);
        LobotSerialRead(_servo->huart, rec_buf, SERVO_BUFF_LEN);//rec_data_len+3
        error_cnt++;
    }

    if (rec_buf[5] == LOBOT_SERVO_MODE_ENGINE && error_cnt < 30) {
        *_speed = (int16_t) MERGE_BYTES(rec_buf[8], rec_buf[7]);
        _servo_dev->servo_flag = false;
        _servo_dev->engine_flag = true;
        return 1;
    } else if (rec_buf[5] == LOBOT_SERVO_MODE_SERVO && error_cnt < 30) {
        _servo_dev->servo_flag = true;
        _servo_dev->engine_flag = false;
        return 0;
    }
    return UINT8_ERROR_RETURN;
}

//32 输出可能是错误
bool IsLobotSerialServoLoading(servo_t *_servo, uint8_t _id) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_READ;
    _buf[5] = LobotCheckSum(_buf);

    while (!LobotReadCheck(rec_buf, _id, rec_data_len, LOBOT_SERVO_LOAD_OR_UNLOAD_READ) && error_cnt < 30) {
        LobotSerialWrite(_servo->huart, _buf, 6);
        LobotSerialRead(_servo->huart, rec_buf, SERVO_BUFF_LEN);//rec_data_len+3
        error_cnt++;
    }

    if (rec_buf[5] == 1 && error_cnt < 30) {
        _servo_dev->load_flag = true;
        return true;
    } else
        return false;
}

//36
uint8_t LobotSerialServoReadError(servo_t *_servo, uint8_t _id) {
    servo_device_t *_servo_dev = _servo->id2dev[_id];
    uint8_t rec_data_len = 4, rec_buf[7] = {0}, error_cnt = 0;
    uint8_t _buf[6];
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 3;
    _buf[4] = LOBOT_SERVO_LED_ERROR_READ;
    _buf[5] = LobotCheckSum(_buf);

    while (!LobotReadCheck(rec_buf, _id, rec_data_len, LOBOT_SERVO_LED_ERROR_READ) && error_cnt < 30) {
        LobotSerialWrite(_servo->huart, _buf, 6);
        LobotSerialRead(_servo->huart, rec_buf, SERVO_BUFF_LEN);//rec_data_len+3
        error_cnt++;
    }
    if (error_cnt < 30) {
        _servo_dev->error_code = rec_buf[5];
        return rec_buf[5];
    } else
        return UINT8_ERROR_RETURN;
}
