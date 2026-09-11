#ifndef ROBOT_INTERFACE_ROBOT_DATA_H_
#define ROBOT_INTERFACE_ROBOT_DATA_H_

#include <cstdint>
#include <cstddef>

/**
 * @brief IgH 周期线程与算法之间的原始实时数据契约。
 *
 * 本头文件只包含每周期实际交换的数据，不包含单位转换、编码器参数、
 * 零点、标定、FoE 内容、IgH 句柄或具体驱动实现。运动数据保持 PDO
 * 原始表示，字段顺序和位宽是算法与 IgH 共同遵守的内存契约。
 *
 * 周期中的数据所有权：
 * - IgH 从设备 TxPDO 更新 servos[].tx、endio.tx 和通信状态；
 * - 算法读取 TxPDO 数据，直接写入 servos[].rx 和 endio.rx；
 * - 回调返回后，IgH 将算法填写的 RxPDO 数据原样传递给设备。
 *
 * 公共接口不包含 CiA402 业务请求或状态机；算法直接生成全部 RxPDO。
 */
namespace robot_interface
{

    inline constexpr uint8_t kMaxSiasunServoCount = 30; // 本体siasun最大轴数。

    struct SiasunServoRxPdo
    {
        // Master -> Servo，算法写入。

        int32_t target_position = 0;      // 0x607A:00
        uint32_t digital_outputs = 0;     // 0x60FE:00
        int32_t target_velocity = 0;      // 0x60FF:00
        uint16_t controlword = 0;         // 0x6040:00
        int16_t target_torque = 0;        // 0x6071:00
        int8_t operation_mode = 0;        // 0x6060:00
        uint8_t safe_control = 0;         // 0x7006:00
        int32_t target_safe_position = 0; // 0x7007:00
        uint32_t user_output = 0;         // 0x7008:00
    };

    struct SiasunServoTxPdo
    {
        // Servo -> Master，算法只读。

        int32_t actual_position = 0;       // 0x6064:00
        uint32_t digital_inputs = 0;       // 0x60FD:00
        int32_t object_6063_00 = 0;        // 0x6063:00
        int32_t object_6069_00 = 0;        // 0x6069:00
        uint32_t error_code = 0;           // 0x603F:00，SIASUN 为 32 bit
        int32_t actual_velocity = 0;       // 0x606C:00
        uint16_t statusword = 0;           // 0x6041:00
        int16_t actual_torque = 0;         // 0x6077:00
        int16_t actual_current = 0;        // 0x6078:00
        int8_t operation_mode_display = 0; // 0x6061:00

        uint16_t object_600b_00 = 0;
        uint32_t object_600c_00 = 0;
        uint32_t object_600d_01 = 0;
        uint16_t object_600d_02 = 0;
        uint32_t object_600d_03 = 0;
        uint32_t object_600d_04 = 0;
        uint16_t object_600d_05 = 0;
        uint16_t object_600d_06 = 0;
        uint32_t object_600d_07 = 0;
        uint32_t object_600d_08 = 0;
        uint16_t object_600d_09 = 0;
        uint16_t object_600d_0a = 0;
        uint32_t object_600d_0b = 0;
        uint32_t object_600d_0c = 0;
        uint32_t object_600d_0d = 0;
    };

    struct SiasunServoCycleData
    {
        SiasunServoRxPdo rx{};
        SiasunServoTxPdo tx{};

        // 不是 PDO，由主站填写。
        uint8_t communication_valid = 0;
    };

    struct SiasunEndIoRxPdo
    {
        // Master -> EndIO，算法写入。

        uint8_t led_work_control = 0;
        uint8_t digital_outputs_control = 0;
        uint16_t rs485_outputs_count = 0;
        uint16_t rs485_outputs_length = 0;
        uint8_t rs485_outputs_data[32]{};
    };

    struct SiasunEndIoTxPdo
    {
        // EndIO -> Master，算法只读。

        uint8_t error_code = 0;
        uint8_t digital_inputs = 0;
        uint16_t analog_voltage_1 = 0;
        uint16_t analog_voltage_2 = 0;
        int16_t temperature = 0;
        int16_t acceleration_x = 0;
        int16_t acceleration_y = 0;
        int16_t acceleration_z = 0;
        uint16_t rs485_inputs_count = 0;
        uint16_t rs485_inputs_length = 0;
        uint8_t rs485_inputs_data[32]{};
    };

    struct SiasunEndIoCycleData
    {
        SiasunEndIoRxPdo rx{};
        SiasunEndIoTxPdo tx{};

        // 不是 PDO，由主站填写。
        uint8_t communication_valid = 0;
    };

    /**
     * @brief 机器人单个实时周期的公共交换数据。
     *
     * IgH 在调用算法周期回调前更新全部 TxPDO 和通信状态；算法在回调内
     * 读取 TxPDO 并填写全部 RxPDO。回调返回后，IgH 不解释字段业务含义，
     * 只负责将 RxPDO 原样写入对应设备的过程数据。
     */
    struct RobotCycleData
    {
        SiasunServoCycleData servos[kMaxSiasunServoCount]{};
        SiasunEndIoCycleData endio{};

        // 初始化成功后由底层填写实际数量。
        uint8_t servo_count = 0;

        // 整个 Domain 本周期是否完整。
        uint8_t domain_data_valid = 0;

        // 调用 init 前由算法设置；0 表示默认周期。
        uint32_t cycle_time_ns = 0;

        // 底层每周期递增。
        uint64_t cycle_count = 0;
    };
} // namespace robot_interface

#endif
