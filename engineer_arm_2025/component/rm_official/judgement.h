//
// Created by 34147 on 2024/2/22.
//

#ifndef ENGINEER_CHASSIS_2024_JUDGEMENT_H
#define ENGINEER_CHASSIS_2024_JUDGEMENT_H
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
// ----------------------------------- 雷达裁判系统测试部分 ----------------------------------- //
typedef struct {
  uint16_t target_robot_id;
  float target_position_x;
  float target_position_y;
  float target_state_point;
} lidar_send_struct;

/*********************************************************************************
 * 							     协议部分 —— 公用协议
 *********************************************************************************/

#define DN_REG_ID    0xA5  //裁判系统通信
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //命令帧
#define CRC_LEN      2    //CRC16校验
#define INTERACTIVE_HEADER_LEN 6 //交互帧头

#define PROTOCOL_FRAME_MAX_SIZE  300 //交互数据最大长度

#define PROTOCOL_DATA_MAX_SIZE 128  //协议中数据最大长度

/**
  * @brief  帧头定义
  */
typedef struct __packed {
  uint8_t sof;                 //DN_REG_ID 固定值0xA5
  uint16_t data_length;         //数据段长度
  uint8_t seq;                 //包序号
  uint8_t crc8;                //帧头校验
} frame_header_t;

/**
  * @brief  解包步骤定义
  */
typedef enum {
  STEP_HEADER_SOF = 0,
  STEP_LENGTH_LOW = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct {
  fifo_s_t *data_fifo;
  frame_header_t *p_header;
  uint16_t data_len;
  uint8_t protocol_packet[PROTOCOL_DATA_MAX_SIZE];
  unpack_step_e unpack_step;
  uint16_t index;
} unpack_data_t;


/*********************************************************************************
 * 			                 协议部分   ——  与裁判系统通信
 *********************************************************************************/

/**
  * @brief  裁判系统命令码
  */
typedef enum {
//从裁判系统接收
  GAME_STATE_ID = 0x0001,        //比赛状态数据，1HZ
  GAME_RESULT_ID = 0x0002,        //比赛结果数据，比赛结束后发送
  GAME_SURVIVORS_ID = 0x0003,        //机器人血量数据，1HZ
  EVENT_DATA_ID = 0x0101,        //场地事件数据，事件改变后发送
  REFEREE_WARNING_ID = 0X0104,        //裁判警告信息,警告发生后发送
  DART_REMAINING_ID = 0X0105,        //飞镖发射口倒计时,1Hz
  GAME_ROBOT_STATE_ID = 0x0201,        //比赛机器人状态，10HZ
  POWER_HEAT_ID = 0x0202,        //实时功率热量数据，50HZ
  ROBOT_POS_ID = 0x0203,        //机器人位置，10HZ
  BUFF_MUSK_ID = 0x0204,        //机器人增益，状态改变后发送
  ROBOT_HURT_ID = 0x0206,        //伤害状态，伤害发生后发送
  SHOOT_DATA_ID = 0x0207,        //实时射击信息，射击后发送
  BULLET_REMAIN_ID = 0X0208,        //子弹余量，1Hz 周期发送，空中机器人以及哨兵机器人主控发送
  RFID_STATUS_ID = 0X0209,        //机器人 RFID 状态1Hz，发送范围：单一机器人。
  DART_CMD_ID = 0X020A,        //飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
  GROUND_ROBOT_CMD_ID = 0x20B, //地面机器人位置数据，固定以1hz频率发送，服务器发送至哨兵
  RADAR_MARK_DATA_ID                = 0X020C,        //雷达标记进度数据，1Hz
  SENTRY_DECISION_DATA_ID = 0x020D , //哨兵自主决策信息同步，固定以1hz频率发送，服务器发送至己方哨兵机器人
  RADAR_DECISION_DATA_ID = 0x020E,//雷达自主决策信息同步，固定以1hz频率发送，服务器发送至己方雷达机器人

  CUSTOM_CONTROLLER_ID = 0X0302,        //自定义控制器交互数据接口，客户端触发发送
  MINIMAP_DATA_ID = 0X0303,        //客户端小地图交互数据，触发发送
  KEYBOARD_DATA_ID = 0X0304,        //键盘鼠标信息，图传串口发送



//通信ID确认
  RED_HERO_ID = 0x0001,      //红方机器人ID
  RED_ENGINEER_ID = 0x0002,
  RED_INFANTRY3_ID = 0x0003,
  RED_INFANTRY4_ID = 0x0004,
  RED_INFANTRY5_ID = 0x0005,
  RED_AERIAL_ID = 0x0006,
  RED_SENTRY_ID = 0x0007,      //红方哨兵
  RED_RADAR_ID = 0x0009,

  BLUE_HERO_ID = 0x0065,      //蓝方机器人ID  101
  BLUE_ENGINEER_ID = 0x0066,      //102
  BLUE_INFANTRY3_ID = 0x0067,      //103
  BLUE_INFANTRY4_ID = 0x0068,      //104
  BLUE_INFANTRY5_ID = 0x0069,      //105
  BLUE_AERIAL_ID = 0x006A,      //106
  BLUE_SENTRY_ID = 0x006B,      //107 蓝方哨兵
  BLUE_RADAR_ID = 0x006D,      //109

  RED_HERO_CUSTOM_ID = 0x0101,      //红方机器人客户端ID
  RED_ENGINEER_CUSTOM_ID = 0x0102,
  RED_INFANTRY3_CUSTOM_ID = 0x0103,
  RED_INFANTRY4_CUSTOM_ID = 0x0104,
  RED_INFANTRY5_CUSTOM_ID = 0x0105,
  RED_AERIAL_CUSTOM_ID = 0x0106,      //红方空中操作手客户端

  BLUE_HERO_CUSTOM_ID = 0x0165,      //蓝方机器人客户端ID
  BLUE_ENGINEER_CUSTOM_ID = 0x0166,
  BLUE_INFANTRY3_CUSTOM_ID = 0x0167,
  BLUE_INFANTRY4_CUSTOM_ID = 0x0168,
  BLUE_INFANTRY5_CUSTOM_ID = 0x0169,
  BLUE_AERIAL_CUSTOM_ID = 0x016A,      //蓝方空中操作手客户端

//向裁判系统发送 机器人间通信 0x0301
  STU_INTERACTIVE_ID = 0x0301,      //交互数据接收信息 学生机器人间通信的cmd_id 0x0301
  STU_STU_DATA_ID = 0x0211,      //机器人间通信数据内容的ID（0x0200~0x02FF)
  STU_CUSTOM_DELETE_PICTURE_ID = 0x0100,      //使客户端删除图形内容的ID
  STU_CUSTOM_ONE_PICTURE_ID = 0x0101,      //使客户端绘制一个图形的ID
  STU_CUSTOM_TWO_PICTURE_ID = 0x0102,      //使客户端绘制两个图形的ID
  STU_CUSTOM_FIVE_PICTURE_ID = 0x0103,      //使客户端绘制五个图形的ID
  STU_CUSTOM_SEVEN_PICTURE_ID = 0x0104,      //使客户端绘制七个图形的ID
  STU_CUSTOM_CHARACTER_ID = 0x0110,      //使客户端绘制字符内容的ID

} judge_data_id_e;

/**
  * @brief  game information structures definition(0x0001)
  * @note 比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人
  */
typedef struct __packed {
  /*game_type
  1 ：RoboMaster 机甲大师赛
  2 ：RoboMaster 机甲大师单项赛
  3 ：ICRA RoboMaster 人工智能挑战赛
  4 ：RoboMaster 联盟赛 3V3
  5 ：RoboMaster 联盟赛 1V1*/
  uint8_t game_type: 4;
  /*game_progress
  0 ：未开始比赛
  1 ：准备阶段
  2 ：自检阶段
  3 ： 5s 倒计时
  4 ：对战中
  5 ：比赛结算中*/
  uint8_t game_progress: 4;
  uint16_t stage_remain_time;     //当前阶段剩余时间，单位s
  uint64_t SyncTimeStamp;         //机器人接收到该指令的精确 时后生效Unix时间，当机载端收到有效的NTP服务器授
} ext_game_state_t;

/**
  * @brief  game result(0x0002)
  */
typedef struct __packed {
  uint8_t winner;     //0平局  1红方胜利   2蓝方胜利
} ext_game_result_t;

/**
  * @brief  (0x0003)
  * @note 机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人
  */
typedef struct __packed {
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t reserved_1;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t reserved_2;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/**
  * @brief  (0x0101)
  */
typedef struct __packed {
  /*event_type:
  bit 0-2：
      bit 0：己方补给站 1 号补血点占领状态 1 为已占领；
      bit 1：己方补给站 2 号补血点占领状态 1 为已占领；
      bit 2：己方补给站 3 号补血点占领状态 1 为已占领；
  bit 3-5：己方能量机关状态：
      bit 3 为打击点占领状态，1 为占领；
      bit 4 为小能量机关激活状态，1 为已激活；
      bit 5 为大能量机关激活状态，1 为已激活；
  bit 6：己方侧 R2/B2 环形高地占领状态 1 为已占领；
  bit 7：己方侧 R3/B3 梯形高地占领状态 1 为已占领；
  bit 8：己方侧 R4/B4 梯形高地占领状态 1 为已占领；
  bit 9：己方基地护盾状态：
      1 为基地有虚拟护盾血量；
      0 为基地无虚拟护盾血量；
  bit 10：己方前哨战状态：
      1 为前哨战存活；
      0 为前哨战被击毁；
  bit 10 -31: 保留*/
  uint32_t event_type;
} ext_event_data_t;


/**
  * @brief  (0x0104)
  * @note 裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。
  */
typedef struct __packed {
  uint8_t level;//判罚等级：1.黄牌 2.红牌 3.判负
  uint8_t offending_robot_id; //违规机器人ID
  uint8_t count;
} ext_referee_warning_t;

/**
  * @brief  (0x0105)
  * @note   飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。
  */
typedef struct __packed {
  uint8_t dart_remaining_time;        //15s倒计时
  uint16_t dart_info;
} ext_dart_remaining_time_t;

/**
  * @brief  (0x0201)
  * @note 比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人。
  */
typedef struct __packed {
  /*
  *本机器人 ID：
  *1：红方英雄机器人
  *2：红方工程机器人
  *3/4/5：红方步兵机器人
  *6：红方空中机器人
  *7：红方哨兵机器人
  *8：红方飞镖机器人
  *9：红方雷达站
  *101：蓝方英雄机器人
  *102：蓝方工程机器人
  *103/104/105：蓝方步兵机器人
  *106：蓝方空中机器人
  *107：蓝方哨兵机器人
  *108：蓝方飞镖机器人
  *109：蓝方雷达站。
  */
  uint8_t robot_id;
  uint8_t robot_level;                //机器人当前等级 1 2 3三个
  uint16_t current_HP;                //当前血量
  uint16_t maximum_HP;                //血量上限
  uint16_t shooter_barrel_cooling_value;    //机器人1号枪口每秒冷却值
  uint16_t shooter_barrel_heat_limit;    //机器人1号枪口热量上限
  uint16_t chassis_power_limit;               //机器人底盘功率上限
  //电源管理模块输出情况
  uint8_t mains_power_gimbal_output: 1;      //1 为有 24V 输出， 0 为无 24v 输出
  uint8_t mains_power_chassis_output: 1;     //1 为有 24V 输出， 0 为无 24v 输出
  uint8_t mains_power_shooter_output: 1;     //1 为有 24V 输出， 0 为无 24v 输出
} ext_game_robot_status_t;

/**
  * @brief  (0x0202)
  * @note 实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。
  */
typedef struct __packed {
  uint16_t reserved_1; //电管chassis口输出电压(mv)
  uint16_t reserved_2; //电管chassis口输出电流(mA)
  float reserved_3;      //底盘功率
  uint16_t buffer_energy;    //缓冲能量(J)
  uint16_t shooter_17mm_1_barrel_heat; //第1个17mm发射机构的枪口热量
  uint16_t shooter_17mm_2_barrel_heat;
  uint16_t shooter_42mm_2_barrel_heat;
} ext_power_heat_data_t;

/**
  * @brief  (0x0203)
  * @note   机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人。
  */
typedef struct __packed {
  float x;
  float y;
  float angle;
} ext_game_robot_pos_t;

/**
  * @brief  (0x0204)
  * @note   机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。
  */
typedef struct __packed {
  uint8_t recovery_buff; //机器人回血增益(百分比，10为每秒回复10%最大血量)
  uint8_t cooling_buff;  //机器人枪口冷却倍率
  uint8_t defence_buff;  //机器人防御增益(百分比)
  uint8_t vulnerability_buff; //机器人负增益(百分比)
  uint16_t attack_buff;  //机器人攻击增益(百分比)
  uint8_t remained_energy; //剩余能量
} ext_buff_musk_t;


/**
  * @brief  (0x0206)
  * @note 伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人。
  */
typedef struct __packed {
  /* bit 0-3 ：
    当血量变化类型为装甲伤害，代表装甲 ID ，
    其中数值为 0-4 号代表机器人的五个装甲片，
    其他血量变化类型，该变量数值为 0 */
  uint8_t armor_id: 4;
  /*hurt_type
   *     bit 4-7 ：血量变化类型
   *     0x0 装甲伤害扣血
   *     0x1 模块掉线扣血
   *     0x2 超射速扣血
   *     0x3 超枪口热量扣血
   *     0x4 超底盘功率扣血
   *     0x5 装甲撞击扣血
   */
  uint8_t HP_deduction_reason: 4;
} ext_robot_hurt_t;

/**
  * @brief  (0x0207)
  * @note 实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。
  */
typedef struct __packed {
  uint8_t bullet_type; //弹丸类型17/42mm
  uint8_t shooter_id;  //发射机构ID
  uint8_t bullet_freq; //弹丸射速
  float bullet_speed;  //弹丸初速度
} ext_shoot_data_t;

/**
  * @brief  (0x0208)
  * @note 子弹剩余发射数：0x0208。
  * 发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，
  * 发送范围：单一机器人。
  */
typedef struct __packed {
  uint16_t projectile_allowance_17mm; //17mm弹丸允许发弹量
  uint16_t projectile_allowance_42mm; //42mm弹丸允许发弹量
  uint16_t remaining_gold_coin;
} ext_bullet_remaining_t;

/**
  * @brief  (0x0209)
  * @note 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。
  */
typedef struct __packed {
  /*
   * bit位值位1/0:是否已检测到该增益点RFID
   * bit 0 ：己方基地增益点
   * bit 1 ：己方环形高地增益点
   * bit 2 : 对方环形高低增益点
   * bit 3 : 己方R3/B3梯形高低增益点
   * bit 4 : 对方R3/B3梯形高低增益点
   * bit 5 : 己方R4/B4梯形高低增益点
   * bit 6 : 对方R4/B4梯形高低增益点
   * bit 7 ：己方能量机关激活点
   * bit 8 : 己方飞坡增益点（飞坡前）
   * bit 9 : 己方飞坡增益点（飞坡后）
   * bit 10 : 对方飞坡增益点（飞坡前）
   * bit 11 : 对方飞坡增益点（飞坡后）
   * bit 12 : 己方前哨战增益点
   * bit 13 : 己方补血点
   * bit 14 : 己方哨兵巡逻区
   * bit 15 : 对方哨兵巡逻区
   * bit 16 : 己方大资源岛增益点
   * bit 17 : 对方大资源岛增益点
   * bit 18 : 己方控制区
   * bit 19 : 对方控制区
   * bit 20-31 ：保留
   * RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不 能获取对应的增益效果。
   */
  uint32_t rfid_status;
} ext_rfid_status_t;

/**
  * @brief  (0x020A)
  * @note   飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
  */
typedef struct __packed {
  uint8_t dart_launch_opening_status; //飞镖发射站状态 1.关闭 2.开启中或关闭中 0.开启
  uint8_t reserved;  //打击目标 0.前哨战 1.基地
  uint16_t target_change_time; //切换打击目标时的比赛剩余时间(s)
  uint16_t latest_launch_cmd_time; //最后一次操作手确定发射指令时的比赛剩余时间(s)
} ext_dart_client_cmd_t;

/**
  * @brief  student custom data(0x0301)
  */
typedef struct __packed {
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} ext_student_interactive_header_data_t;//相互交流的头,都需要这个头


/**
  * @brief  student student data(0x0301)---(0x0201~0x02FF)
  * 学生机器人间通信
  */
typedef struct __packed {
  char *data;//字节大小小于113
} robot_interactive_data_t;

/**
  * @brief  student student data(0x0301)---(0x0201~0x02FF)
  */
typedef struct __packed {
  ext_student_interactive_header_data_t student_interactive_header_data;
  robot_interactive_data_t robot_interactive_data;
} ext_student_student_data_t;

/**
  * @brief  student custom delete picture(0x0301)---(0x0100)
  */
typedef struct __packed {
  uint8_t operate_type;//0 空操作 1 删除图层 2 删除所有
  uint8_t layer;//图层数0-9
} ext_client_custom_graphic_delete_t;

/**
  * @brief  student custom graph data 绘制的基本单元 具有15个字节
  * 字节偏移量已经规定好了,所以不能乱更改,其中空代表该字段的数据对该图形无影响，推荐字体大小与线宽比例为 10:1
  */
typedef struct __packed {
  uint8_t graphic_name[3];//图形名
  //图形配置1
  uint32_t operate_type: 3;//图形操作 0空操作 1增加2修改3删除
  uint32_t graphic_type: 3;//图形类型 0直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数 7字符
  uint32_t layer: 4;//图层数可以选择 0-9
  uint32_t color: 4;//颜色 0红蓝主色 1黄色 2绿色 3橙色 4紫红色 5粉色 6青色 7黑色 8白色
  uint32_t start_angle: 9;//起始角度 [0°,360°], 字符的字体大小
  uint32_t end_angle: 9;//终止角度 [0°,360°] ， 字符的长度
  //图形配置2
  uint32_t width: 10;//线宽
  uint32_t start_x: 11;//起点x坐标
  uint32_t start_y: 11;//起点y坐标
  //图形配置3
  uint32_t radius: 10;//字体大小或半径
  uint32_t end_x: 11;//终点x坐标
  uint32_t end_y: 11;//终点y坐标
} graphic_data_struct_t;//※一个图形，一个图形。见通信协议25页

enum ui_delete {
  ui_delete_none = 0,
  ui_delete_layer,
  ui_delete_all,
};

enum ui_color {
  ui_red_blue = 0,
  ui_yellow,
  ui_green,
  ui_orange,
  ui_purple,
  ui_pink,
  ui_cyan,
  ui_black,
  ui_white,
};

enum ui_operation {
  ui_none_operation = 0,
  ui_add,
  ui_modify,
  ui_clear,
};

enum ui_graphic_type {
  ui_line = 0,
  ui_rectangle,
  ui_circle,
  ui_oval,
  ui_arc,
  ui_floating,
  ui_integer,
  ui_character,
};

/**
  * @brief  student custom draw one picture(0x0301)---(0x0101)
  * 发送者的ID为机器人ID,接收者的ID为对应发送机器人的客户端ID
  */
typedef struct __packed {
  graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

/**
  * @brief  student custom draw two picture(0x0301)---(0x0102)
  * 发送者的ID为机器人ID,接收者的ID为对应发送机器人的客户端ID
  */
typedef struct __packed {
  graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

/**
  * @brief  student custom draw five picture(0x0301)---(0x0103)
  * 发送者的ID为机器人ID,接收者的ID为对应发送机器人的客户端ID
  */
typedef struct __packed {
  graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

/**
  * @brief  student custom draw seven picture(0x0301)---(0x0104)
  * 发送者的ID为机器人ID,接收者的ID为对应发送机器人的客户端ID
  */
typedef struct __packed {
  graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

/**
  * @brief  student custom draw character 字符
  * 发送者的ID为机器人ID,接收者的ID为对应发送机器人的客户端ID
  */
typedef struct __packed {
  graphic_data_struct_t grapic_data_struct;
  char data[30];
} ext_client_custom_character_t;

/**
  * @brief costum_interactive_data_t(0x0302) 交互数据接收信息：0x0302。发送频率：上限 30Hz
  */
typedef struct __packed {
  uint8_t data[30];
} ext_custom_interactive_data_t;

/**
  * 客户端下发信息
  * @brief Interactive information of small map(0x0303) 小地图下发信息标识：0x0303。发送频率：触发时发送
  */
typedef struct __packed {
  float target_position_x;
  float target_position_y;
  float target_position_z;
  uint8_t cmd_keyboard;
  uint8_t target_robot_id;
  uint16_t target_robot_ID;
} ext_minimap_t;

/** 客户端接收信息
  * @brief 小地图接收信息标识：0x0305。最大接收频率：10Hz
  * 雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到
  */
typedef struct __packed {
    float hero_position_x;
    float hero_position_y;
    float engineer_position_x;
    float engineer_position_y;
    float infantry3_position_x;
    float infantry3_position_y;
    float infantry4_position_x;
    float infantry4_position_y;
    float infantry5_position_x;
    float infantry5_position_y;
    float sentry_position_x;
    float sentry_position_y;
} ext_client_map_command_t;

/** 图传
  * @brief Picture transmission remote control information(0x0304) 发送频率：30Hz。
  */
typedef struct __packed {
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int8_t left_button_down;
  int8_t right_button_down;
  uint16_t keyboard_value;
  uint16_t reserved;
} ext_interactive_information_t;

/**
  * @brief  the data structure receive from judgement
  */
typedef struct {
  ext_game_state_t game_state_data;               //0x0001
  ext_game_result_t game_result_data;              //0x0002
  ext_game_robot_HP_t ext_game_robot_HP_t;           //0x0003
  ext_event_data_t event_data;                    //0x0101
  ext_referee_warning_t referee_warning_data;          //0x0104
  ext_dart_remaining_time_t dart_remaining_data;           //0x0105
  ext_game_robot_status_t game_robot_state_data;         //0x0201
  ext_power_heat_data_t power_heat_data;               //0x0202
  ext_game_robot_pos_t game_robot_pos_data;           //0x0203
  ext_buff_musk_t buff_musk_data;                //0x0204
  ext_robot_hurt_t robot_hurt_data;               //0x0206
  ext_shoot_data_t shoot_data;                    //0x0207
  ext_student_student_data_t student_student_data;
  ext_bullet_remaining_t bullet_remaining_data;         //0x0208
  ext_rfid_status_t rfid_status_data;              //0x0209
  ext_dart_client_cmd_t dart_cmd_data;                 //0x020A
  ext_custom_interactive_data_t custom_interactive_data;       //0x0302 自定义控制器
  ext_minimap_t mini_map_data;                 //0x0303
  ext_interactive_information_t interactive_information_data;  //0x0304
} receive_judge_t;

typedef struct {
  ext_client_custom_graphic_delete_t delete_graph;
  ext_client_custom_graphic_single_t single_graph;
  ext_client_custom_graphic_double_t double_graph;
  ext_client_custom_graphic_five_t five_graph;
  ext_client_custom_graphic_seven_t seven_graph;
} ext_client_custom_graphic_t;
#endif //ENGINEER_CHASSIS_2024_JUDGEMENT_H
