#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <drivers/can_rx_manager.h>

LOG_MODULE_REGISTER(can_rx_manage_sample, LOG_LEVEL_INF);

/* CAN 控制器设备（基于 board DTS 中的 node labels） */
const struct device *const fdcan1 = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
const struct device *const fdcan2 = DEVICE_DT_GET(DT_NODELABEL(fdcan2));

/* 管理器在 overlay 中设置了 label = "can_rx_mgr1" */
#define CAN_MGR_LABEL "can_rx_mgr1"

/* 我们的“设备”结构，用于保存最近接收到的帧 */
struct my_device {
	uint32_t last_id;
	uint8_t last_data[CAN_MAX_DLEN];
	uint8_t last_dlc;
};

static struct my_device my_dev = {0};

/* 管理器回调：把 frame 的内容拷贝到我们的设备并打印 */
static void my_dev_handler(const struct can_frame *frame, void *user_data)
{
	struct my_device *dev = (struct my_device *)user_data;
	if ((frame == NULL) || (dev == NULL)) {
		return;
	}

	dev->last_id = frame->id;
	dev->last_dlc = frame->dlc;
	(void)memcpy(dev->last_data, frame->data, (size_t)frame->dlc);

	/* 打印接收到的数据 */
	char buf[64];
	size_t off = 0;
	off += snprintk(buf + off, sizeof(buf) - off, "my_dev recv id=0x%03x dlc=%u data=", frame->id, frame->dlc);
	for (int i = 0; i < frame->dlc && off + 4 < sizeof(buf); i++) {
		off += snprintk(buf + off, sizeof(buf) - off, "%02X ", frame->data[i]);
	}
	LOG_INF("%s", buf);
}

int main(void)
{
	const struct device *mgr;
	int ret;

	LOG_INF("CAN RX Manager example start");

	/* 获取管理器设备 */
	mgr = device_get_binding(CAN_MGR_LABEL);
	if (mgr == NULL) {
		LOG_ERR("can_rx_manager device '%s' not found", CAN_MGR_LABEL);
		return -ENODEV;
	}

	/* 检查 CAN 控制器就绪 */
	if (!device_is_ready(fdcan1)) {
		LOG_ERR("fdcan1 not ready");
		return -ENODEV;
	}
	if (!device_is_ready(fdcan2)) {
		LOG_ERR("fdcan2 not ready");
		return -ENODEV;
	}

	/* 启动控制器（管理器的 init 已在驱动中为 fdcan1 添加了 HW 过滤器） */
	ret = can_start(fdcan1);
	if ((ret < 0) && (ret != -EALREADY)) {
		LOG_ERR("failed to start fdcan1: %d", ret);
		return ret;
	}
	ret = can_start(fdcan2);
	if ((ret < 0) && (ret != -EALREADY)) {
		LOG_ERR("failed to start fdcan2: %d", ret);
		return ret;
	}

	/* 在管理器上注册监听器（接收所有 ID） */
	struct can_filter filter = { .id = 0, .mask = 0, .flags = 0 };
	ret = can_rx_manager_register(mgr, &filter, my_dev_handler, &my_dev);
	if (ret < 0) {
		LOG_ERR("failed to register listener on manager: %d", ret);
		return ret;
	}
	LOG_INF("registered listener id=%d on %s", ret, CAN_MGR_LABEL);

	/* 发送线程循环：由 fdcan2 发帧到总线，fdcan1（由管理器监听）会收到并在 my_dev_handler 打印 */
	struct can_frame txf = {0};
	txf.flags = 0; /* 标准帧 */
	txf.id = 0x123;
	txf.dlc = 4;

	uint8_t counter = 0;
	while (1) {
		/* 填充数据 */
		for (int i = 0; i < txf.dlc; i++) {
			txf.data[i] = counter + i;
		}

		ret = can_send(fdcan2, &txf, K_MSEC(100), NULL, NULL);
		if (ret != 0) {
			LOG_WRN("can_send failed: %d", ret);
		} else {
			LOG_INF("fdcan2 sent id=0x%03x dlc=%u", txf.id, txf.dlc);
		}

		counter++;
		k_sleep(K_MSEC(500));
	}

	return 0;
}


