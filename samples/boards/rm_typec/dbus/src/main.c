#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <string.h>

LOG_MODULE_REGISTER(dbus_dma, LOG_LEVEL_INF);

#define DBUS_UART_NODE DT_ALIAS(dbus)
#define DBUS_PACKET_SIZE 18
#define DBUS_TIMEOUT_MS 10  // 触发空闲中断 UART_RX_RDY

/* DBUS协议数据结构 */
typedef struct {
    uint8_t value;
} switch_t;

typedef struct {
    uint8_t value;
} key_t;

typedef struct {
    int16_t value;
} thumbwheel_t;

typedef struct {
    int16_t ch0, ch1, ch2, ch3;
    switch_t s1, s2;
    int16_t mouse_vx, mouse_vy, mouse_vz;
    key_t mouse_btn_l, mouse_btn_r;
    key_t W, S, A, D, Shift, Ctrl, Q, E, R, F, G, Z, X, C, V, B;
    thumbwheel_t thumbwheel;
} rc_base_info_t;

static const struct device *dbus_uart = DEVICE_DT_GET(DBUS_UART_NODE);
static uint8_t rx_buffer[DBUS_PACKET_SIZE];
static rc_base_info_t rc_info = {0};
static bool rx_enabled = false;

/* 调试变量 */
static uint8_t test_s1, test_s2;

void rc_base_info_update(rc_base_info_t *info, uint8_t *rxBuf)
{
  info->ch0 = (rxBuf[0]      | rxBuf[1] << 8                 ) & 0x07FF;
  info->ch0 -= 1024;
  info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5                 ) & 0x07FF;
  info->ch1 -= 1024;
  info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
  info->ch2 -= 1024;
  info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7                 ) & 0x07FF;
  info->ch3 -= 1024;
  
  // 开关滤波参数
  #define SWITCH_FILTER_THRESHOLD 5  // 需要连续5次改变才更新
  static uint8_t s1_change_count = 0;
  static uint8_t s2_change_count = 0;
  
  uint8_t temp = ((rxBuf[5] >> 4) & 0x000C) >> 2;
  test_s1 = temp;
  if (temp != info->s1.value)
  {
    s1_change_count++;
    if (s1_change_count >= SWITCH_FILTER_THRESHOLD)
    {
      info->s1.value = temp;
      s1_change_count = 0;
    }
  }
  else
  {
    s1_change_count = 0;
  }
  
  temp = (rxBuf[5] >> 4) & 0x0003;
  test_s2 = temp;
  if (temp != info->s2.value)
  {
    s2_change_count++;
    if (s2_change_count >= SWITCH_FILTER_THRESHOLD)
    {
      info->s2.value = temp;
      s2_change_count = 0;
    }
  }
  else
  {
    s2_change_count = 0;
  }

  info->mouse_vx = rxBuf[6]  | (rxBuf[7 ] << 8);
  info->mouse_vy = rxBuf[8]  | (rxBuf[9 ] << 8);
  info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8);
  info->mouse_btn_l.value = rxBuf[12] & 0x01;
  info->mouse_btn_r.value = rxBuf[13] & 0x01;
  info->W.value =   rxBuf[14]        & 0x01;
  info->S.value = ( rxBuf[14] >> 1 ) & 0x01;
  info->A.value = ( rxBuf[14] >> 2 ) & 0x01;
  info->D.value = ( rxBuf[14] >> 3 ) & 0x01;
  info->Shift.value = ( rxBuf[14] >> 4 ) & 0x01;
  info->Ctrl.value = ( rxBuf[14] >> 5 ) & 0x01;
  info->Q.value = ( rxBuf[14] >> 6 ) & 0x01 ;
  info->E.value = ( rxBuf[14] >> 7 ) & 0x01 ;
  info->R.value = ( rxBuf[15] >> 0 ) & 0x01 ;
  info->F.value = ( rxBuf[15] >> 1 ) & 0x01 ;
  info->G.value = ( rxBuf[15] >> 2 ) & 0x01 ;
  info->Z.value = ( rxBuf[15] >> 3 ) & 0x01 ;
  info->X.value = ( rxBuf[15] >> 4 ) & 0x01 ;
  info->C.value = ( rxBuf[15] >> 5 ) & 0x01 ;
  info->V.value = ( rxBuf[15] >> 6 ) & 0x01 ;
  info->B.value = ( rxBuf[15] >> 7 ) & 0x01 ;

  info->thumbwheel.value = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
  info->thumbwheel.value -= 1024;
}

/**
 * @brief DBUS完整数据包接收回调函数
 * 
 * @param buffer 完整的18字节数据包
 * @param length 数据长度(总是18)
 */
void dbus_packet_received_callback(uint8_t *buffer, size_t length)
{
    /* 解析DBUS数据包 */
    rc_base_info_update(&rc_info, buffer);
    LOG_DBG("Parse DBUS Data");
}

/**
 * @brief UART异步接收回调函数
 */
static void uart_rx_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        /* 检查是否收到完整包 */
        LOG_DBG("UART RX OFFSET: %d, LENGTH: %d", evt->data.rx.offset, evt->data.rx.len);
        if (evt->data.rx.len == DBUS_PACKET_SIZE) {
            dbus_packet_received_callback(evt->data.rx.buf, DBUS_PACKET_SIZE);
        } else {
            LOG_WRN("Incomplete packet: %d bytes, restarting reception", evt->data.rx.len);
        }
        uart_rx_disable(dev);
        
        break;

    case UART_RX_BUF_REQUEST:
        LOG_INF("UART_RX_BUF_REQUEST received");
        break;
    
    case UART_RX_BUF_RELEASED:
        LOG_INF("UART_RX_BUF_RELEASED received");
        break;
    
    case UART_RX_DISABLED:
        LOG_INF("UART_RX_DISABLED received");
        uart_rx_enable(dbus_uart, rx_buffer, DBUS_PACKET_SIZE, DBUS_TIMEOUT_MS);
        break;
    
    case UART_RX_STOPPED:
        LOG_ERR("UART_RX_STOPPED received");
        break;
        
    default:
        LOG_WRN("Unexpected UART event: %d", evt->type);
        break;
    }
}

/**
 * @brief 初始化DBUS UART接收
 */
static int dbus_init(void)
{
    int ret;
    
    if (!device_is_ready(dbus_uart)) {
        LOG_ERR("DBUS UART device not ready");
        return -ENODEV;
    }
    
    LOG_INF("DBUS UART device is ready");
    
    /* 注册回调函数 */
    ret = uart_callback_set(dbus_uart, uart_rx_callback, NULL);
    if (ret != 0) {
        LOG_ERR("Failed to set UART callback: %d", ret);
        return ret;
    }
    
    /* 启动异步接收 */
    ret = uart_rx_enable(dbus_uart, rx_buffer, DBUS_PACKET_SIZE, DBUS_TIMEOUT_MS);
    if (ret != 0) {
        LOG_ERR("Failed to enable UART RX: %d", ret);
        return ret;
    }
    rx_enabled = true;

    LOG_INF("DBUS packet reception initialized successfully");
    return 0;
}

int main(void)
{
    int ret;

    LOG_INF("DBUS Packet Reception Example Starting...");
    
    /* 初始化DBUS */
    ret = dbus_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize DBUS: %d", ret);
        return ret;
    }
    
    LOG_INF("DBUS ready, waiting for 18-byte packets...");
    
    /* 主循环 */
    while (1) {
        k_sleep(K_MSEC(1000));
        /* 可以在这里添加其他任务 */
    }
    
    return 0;
}

