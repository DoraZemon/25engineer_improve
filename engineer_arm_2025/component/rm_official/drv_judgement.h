//
// Created by 34147 on 2024/2/22.
//

#ifndef ENGINEER_CHASSIS_2024_DRV_JUDGEMENT_H
#define ENGINEER_CHASSIS_2024_DRV_JUDGEMENT_H
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "bsp_rm_usart.h"
#include "drv_data_fifo.h"
#include "judgement.h"
#ifdef __cplusplus
}
#endif
//C++

#define UART_RX_DMA_SIZE    (1024)
#define JUDGE_FIFO_BUF_LEN  (1024)

class judgement_device {
 public:
  usart_param_t usart;
  ///RX FIFO
/* judge system receive data buffer */
/* judge system dma receive data object */
  uint8_t judge_dma_rxbuff[UART_RX_DMA_SIZE];
  uint8_t judge_rxdata_buf[JUDGE_FIFO_BUF_LEN];
/* judge system receive data fifo */
  fifo_s_t judge_rxdata_fifo;//fifo创建中要用到互斥量
/* judge system receive data mutex */
  osMutexId judge_rxdata_mutex;


///TX FIFO
/* judge system send data buffer */
  uint8_t judge_txdata_buf[JUDGE_FIFO_BUF_LEN];
/* judge system send data fifo */
  fifo_s_t judge_txdata_fifo;//fifo创建中要用到互斥量
/* judge system send data mutex */
  osMutexId judge_txdata_mutex;


  ///UNPACK
/* unpack object */
  unpack_data_t judge_unpack_obj;

  receive_judge_t judge_rece_mesg;//接受的所有数据的一个结构体

  /**
* @brief    get judgement system message
*/
  uint8_t Do_heat_calibration;
  uint8_t SupplyingFlag;
  uint8_t DeathModeFlag;
  float SmallGim_Heat_User;
  float BigGimHeat_User;
  int16_t SmalBulletSped;
  int16_t BigBulletSped;
  int16_t SmallBulletShotNum;
  int16_t BigBulletShotNum;//----------用来计算大子弹消耗数量
  int16_t BigBulletShotNumLast;
  float BigBulletActualSpeed;
  uint8_t BigBulletIncreFlag;
  uint8_t BigBulletIncreFlag_2;
  uint8_t NoBigBulletFlag;
  uint8_t AimingSequence[8];//瞄准顺序

  //for debug
  int dma_write_len = 0;
  int fifo_overflow = 0;

  uint8_t tx_buf[PROTOCOL_FRAME_MAX_SIZE];//最终发送出去的数组

  uint8_t robot_communication_data[3] = {1, 2, 3};
  uint8_t *robot_communication_p_data = robot_communication_data;
  uint16_t robot_communication_len = 3;
  ext_student_interactive_header_data_t robot_interactive;

  lidar_send_struct lidar_data;
  ext_student_interactive_header_data_t lidar_robot_interactive_header;

  uint8_t Rx_buf_pos = 0;    //本次回调接收的数据在缓冲区的起点
  uint32_t Rx_length;    //本次回调接收数据的长度
 public:
  void init(UART_HandleTypeDef *huart,
            osEventFlagsId_t event,
            const osMutexAttr_t *rx_attr,
            const osMutexAttr_t *tx_attr);
  void data_handler(uint8_t *p_frame);
  void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof);
  void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type);
  uint8_t *protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t *_tx_buf,
                                ext_student_interactive_header_data_t _student_interactive_header);
  uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof);
  void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len,
                        ext_student_interactive_header_data_t _student_interactive_header);

  // ----------------------------------------- 雷达数据打包 ----------------------------------------- //
  uint8_t *lidar_protocol_packet_pack(uint16_t cmd_id, lidar_send_struct lidar_data, uint8_t *_tx_buf,
                                      ext_student_interactive_header_data_t _student_interactive_header);
  void lidar_data_pack(uint16_t cmd_id, lidar_send_struct lidar_data,
                       ext_student_interactive_header_data_t _student_interactive_header);
  void robot_communicate_send_data(uint16_t data_cmd_id, uint16_t sender_ID, uint16_t receiver_ID, uint8_t *_p_data,
                                   uint16_t _len);
  void robot_communicate_init();
  void lidar_send_init();

  void usart_rx_processed(uint16_t Size);//接受串口原始数据
};

#endif //ENGINEER_CHASSIS_2024_DRV_JUDGEMENT_H
