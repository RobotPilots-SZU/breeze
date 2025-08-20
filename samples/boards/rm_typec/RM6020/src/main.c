#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rm6020_motor_control, LOG_LEVEL_INF);

/* 电机控制相关定义 */
#define MOTOR_CONTROL_ID 0x1FF          // 电机控制标识符

/* 电机反馈相关定义 */
#define MOTOR_FEEDBACK_BASE_ID 0x204    // 电机反馈基础ID (0x204 + 驱动器ID)

/* 电机反馈数据结构体 */
struct motor_feedback {
    uint16_t mechanical_angle;     // 机械角度 (0-8191)
    int16_t speed_rpm;            // 转速 (rpm)
    int16_t actual_current;       // 实际转矩电流 (mA)
    uint8_t temperature;          // 电机温度 (°C)
    uint8_t motor_id;             // 电机ID (从CAN ID计算得出)
};

/* 接收统计结构体 */
struct can_rx_stats {
    uint32_t packet_count;         // 接收包计数
    struct motor_feedback latest_feedback;  // 最新的反馈数据
    int64_t last_output_time;     // 上次输出时间
    int64_t first_packet_time;    // 第一个包的时间
    bool has_valid_data;          // 是否有有效数据
};

/* CAN设备获取 */
const struct device *const can1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
const struct device *const can2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));

/* 接收统计变量 */
static struct can_rx_stats can1_stats = {0};
static struct can_rx_stats can2_stats = {0};

/* 发送回调函数 */
void tx_irq_callback(const struct device *dev, int error, void *arg)
{
    if (error != 0) {
        LOG_ERR("Motor control TX error: %d", error);
    }
}

/**
 * @brief 解包电机反馈数据
 * @param frame CAN帧指针
 * @param feedback 解包后的反馈数据结构体指针
 * @return 0 成功, -1 失败
 */
int unpack_motor_feedback(const struct can_frame *frame, struct motor_feedback *feedback)
{
    if (frame->dlc != 8) {
        LOG_ERR("Invalid motor feedback frame DLC: %d", frame->dlc);
        return -1;
    }
    
    /* 检查是否为电机反馈消息 */
    if ((frame->id & 0xFF0) != (MOTOR_FEEDBACK_BASE_ID & 0xFF0)) {
        return -1;  // 不是电机反馈消息
    }
    
    /* 计算电机ID */
    feedback->motor_id = frame->id - MOTOR_FEEDBACK_BASE_ID;
    
    /* 解包数据 - 按照协议格式 */
    feedback->mechanical_angle = (frame->data[0] << 8) | frame->data[1];    // 机械角度高8位 + 低8位
    feedback->speed_rpm = (int16_t)((frame->data[2] << 8) | frame->data[3]); // 转速高8位 + 低8位
    feedback->actual_current = (int16_t)((frame->data[4] << 8) | frame->data[5]); // 实际转矩电流高8位 + 低8位
    feedback->temperature = frame->data[6];  // 电机温度
    // frame->data[7] 为 Null，忽略
    
    return 0;
}

/**
 * @brief CAN1接收回调函数
 * @param dev CAN设备指针
 * @param frame 接收到的CAN帧
 * @param user_data 用户数据
 */
void can1_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct motor_feedback feedback;
    
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    
    /* 尝试解包电机反馈数据 */
    if (unpack_motor_feedback(frame, &feedback) == 0) {
        can1_stats.packet_count++;
        can1_stats.latest_feedback = feedback;
        can1_stats.has_valid_data = true;
        
        /* 记录第一个包的时间 */
        if (can1_stats.packet_count == 1) {
            can1_stats.first_packet_time = k_uptime_get();
        }
        
        if (can1_stats.packet_count % 30 == 0) {
            int64_t current_time = k_uptime_get();
            int64_t time_diff = current_time - can1_stats.first_packet_time;
            uint32_t frequency = 0;
            
            if (time_diff > 0) {
                frequency = (uint32_t)(can1_stats.packet_count * 1000 / time_diff);
            }
            
            LOG_INF("CAN1 [%d packets] Motor[%d] - Angle:%d, Speed:%d rpm, Current:%d mA, Temp:%d°C, Freq:%d Hz",
                    can1_stats.packet_count,
                    can1_stats.latest_feedback.motor_id,
                    can1_stats.latest_feedback.mechanical_angle,
                    can1_stats.latest_feedback.speed_rpm,
                    can1_stats.latest_feedback.actual_current,
                    can1_stats.latest_feedback.temperature,
                    frequency);
        }
    } else {
        LOG_DBG("CAN1 received non-motor message: ID=0x%03X", frame->id);
    }
}

/**
 * @brief CAN2接收回调函数
 * @param dev CAN设备指针
 * @param frame 接收到的CAN帧
 * @param user_data 用户数据
 */
void can2_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct motor_feedback feedback;
    
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    
    /* 尝试解包电机反馈数据 */
    if (unpack_motor_feedback(frame, &feedback) == 0) {
        can2_stats.packet_count++;
        can2_stats.latest_feedback = feedback;
        can2_stats.has_valid_data = true;
        
        /* 记录第一个包的时间 */
        if (can2_stats.packet_count == 1) {
            can2_stats.first_packet_time = k_uptime_get();
        }
        
        if (can2_stats.packet_count % 30 == 0) {
            int64_t current_time = k_uptime_get();
            int64_t time_diff = current_time - can2_stats.first_packet_time;
            uint32_t frequency = 0;
            
            if (time_diff > 0) {
                frequency = (uint32_t)(can2_stats.packet_count * 1000 / time_diff);
            }
            
            LOG_INF("CAN2 [%d packets] Motor[%d] - Angle:%d, Speed:%d rpm, Current:%d mA, Temp:%d°C, Freq:%d Hz",
                    can2_stats.packet_count,
                    can2_stats.latest_feedback.motor_id,
                    can2_stats.latest_feedback.mechanical_angle,
                    can2_stats.latest_feedback.speed_rpm,
                    can2_stats.latest_feedback.actual_current,
                    can2_stats.latest_feedback.temperature,
                    frequency);
        }
    } else {
        LOG_DBG("CAN2 received non-motor message: ID=0x%03X", frame->id);
    }
}

/**
 * @brief 发送电机控制指令
 * @param dev CAN设备
 * @param motor1_voltage 电机1电压给定值 (高8位)
 * @param motor2_voltage 电机2电压给定值 (高8位) 
 * @param motor3_voltage 电机3电压给定值 (高8位)
 * @param motor4_voltage 电机4电压给定值 (高8位)
 */
int send_motor_control(const struct device *dev, 
                      int16_t motor1_voltage, int16_t motor2_voltage,
                      int16_t motor3_voltage, int16_t motor4_voltage)
{
    struct can_frame frame = {
        .flags = 0,
        .id = MOTOR_CONTROL_ID,
        .dlc = 8
    };
    
    /* 按照协议格式填充数据 */
    frame.data[0] = (motor1_voltage >> 8) & 0xFF;  // 电机1电压给定值高8位
    frame.data[1] = motor1_voltage & 0xFF;         // 电机1电压给定值低8位
    frame.data[2] = (motor2_voltage >> 8) & 0xFF;  // 电机2电压给定值高8位
    frame.data[3] = motor2_voltage & 0xFF;         // 电机2电压给定值低8位
    frame.data[4] = (motor3_voltage >> 8) & 0xFF;  // 电机3电压给定值高8位
    frame.data[5] = motor3_voltage & 0xFF;         // 电机3电压给定值低8位
    frame.data[6] = (motor4_voltage >> 8) & 0xFF;  // 电机4电压给定值高8位
    frame.data[7] = motor4_voltage & 0xFF;         // 电机4电压给定值低8位
    
    int ret = can_send(dev, &frame, K_MSEC(100), tx_irq_callback, NULL);
    
    return ret;
}

int main(void)
{
    int ret;
    struct can_filter filter;             // CAN过滤器
    
    /* 检查CAN设备就绪状态 */
    if (!device_is_ready(can1_dev)) {
        LOG_ERR("CAN1 device not ready");
        return -1;
    }
    
    if (!device_is_ready(can2_dev)) {
        LOG_ERR("CAN2 device not ready");
        return -1;
    }
    
    /* 启动CAN控制器 */
    ret = can_start(can1_dev);
    if (ret != 0) {
        LOG_ERR("Failed to start CAN1: %d", ret);
        return -1;
    }
    
    ret = can_start(can2_dev);
    if (ret != 0) {
        LOG_ERR("Failed to start CAN2: %d", ret);
        return -1;
    }

        /* 配置CAN接收过滤器和回调函数 */
    LOG_INF("Configuring CAN RX filters and callbacks");

    /* 设置CAN1接收过滤器 - 接收所有消息 */
    filter.id = 0;
    filter.mask = 0;
    filter.flags = 0;

    ret = can_add_rx_filter(can1_dev, can1_rx_callback, NULL, &filter);
    if (ret < 0) {
        LOG_ERR("Failed to add CAN1 RX filter: %d", ret);
    } else {
        LOG_INF("CAN1 RX filter added successfully");
    }

    /* 设置CAN2接收过滤器 - 接收所有消息 */
    ret = can_add_rx_filter(can2_dev, can2_rx_callback, NULL, &filter);
    if (ret < 0) {
        LOG_ERR("Failed to add CAN2 RX filter: %d", ret);
    } else {
        LOG_INF("CAN2 RX filter added successfully");
    }
    
    LOG_INF("RM6020 Motor Control System Started");
    
    int16_t target_vol = 16000;
    
    while (1) {
        
        /* CAN1发送电机控制指令 - 控制电机1和2 */
        send_motor_control(can1_dev, target_vol, target_vol, target_vol, target_vol);
        /* CAN2发送电机控制指令 - 控制电机3和4 */
        send_motor_control(can2_dev, target_vol, target_vol, target_vol, target_vol);
        k_sleep(K_MSEC(1));
    }
    
    return 0;
}
