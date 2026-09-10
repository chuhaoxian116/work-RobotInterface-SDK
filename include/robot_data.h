#ifndef ROBOT_INTERFACE_ROBOT_DATA_H_
#define ROBOT_INTERFACE_ROBOT_DATA_H_

#include <cstdint>

/**
 * @brief IgH 周期线程与算法之间的原始实时数据契约。
 *
 * 本头文件只包含每周期实际交换的数据，不包含单位转换、编码器参数、
 * 零点、标定、FoE 内容、IgH 句柄或具体驱动实现。运动数据保持 PDO
 * 原始表示，字段顺序和位宽是算法与 IgH 共同遵守的内存契约。
 *
 * 周期中的数据所有权：
 * - IgH 写入 AxisFeedback；
 * - 算法读取 AxisFeedback，写入 AxisSetpoint 和 RobotServiceRequest；
 * - IgH 内部将服务请求转换为驱动命令后写回 PDO。
 */
namespace robot_interface
{

    inline constexpr uint8_t kMaxRobotAxisCount = 30;  // 机器人本体支持的最大轴数。
    inline constexpr uint8_t kMaxExternalAxisCount = 6;  // 外部扩展轴支持的最大轴数。

    /**
     * @brief 算法可请求或反馈的机器人运行模式。
     *
     * 枚举值与常用 CiA402 模式值保持一致，调用方只能使用本业务枚举，
     * 不需要接触任何 CiA402 类型。
     */
    enum class RobotMode : int8_t
    {
        kHoming = 6, // Homing mode，回零模式。
        kCsp = 8,    // CSP，周期同步位置模式。
        kCsv = 9,    // CSV，周期同步速度模式。
        kCst = 10,   // CST，周期同步转矩模式。
    };

    /**
     * @brief 单轴在当前周期的原始实时反馈。
     *
     * 由 IgH 从 TxPDO 按原值写入，不执行单位、方向或零点处理。
     */
    struct AxisFeedback
    {
        int32_t actual_position = 0;  // 0x6064 原始实际位置。
        int32_t actual_velocity = 0;  // 0x606C 原始实际速度。
        int16_t actual_torque = 0;  // 0x6077 原始实际转矩。
        uint32_t error_code = 0;  // 原始错误码，兼容 16/32 bit PDO。
        uint16_t statusword = 0;  // 0x6041 原始状态字。
        int8_t mode_display = 0;  // 0x6061 原始模式反馈。

        uint8_t communication_valid = 0;  // 本周期该轴的通信数据是否有效。
    };

    /**
     * @brief 算法在当前周期给出的单轴原始运动目标。
     *
     * IgH 按原值写入 RxPDO，不执行单位、方向或零点处理。控制字和驱动
     * 运行模式仍由 IgH 内部的 CiA402 命令调度生成。
     */
    struct AxisSetpoint
    {
        int32_t target_position = 0;  // 0x607A 原始目标位置。
        int32_t target_velocity = 0;  // 0x60FF 原始目标速度。
        int16_t target_torque = 0;  // 0x6071 原始目标转矩。
    };

    /**
     * @brief 算法提交给 IgH 的高层服务请求。
     *
     * 多个字段允许在同一周期同时置位。是否调用、调用顺序及跨周期
     * 状态管理由 IgH 上层逻辑决定；本公共接口不规定业务调度策略。
     */
    struct RobotServiceRequest
    {
        uint8_t clear_error = 0;  // 非 0 时请求清除机器人本体轴故障。
        uint8_t power_request_valid = 0;  // 非 0 时本周期 power_enable 字段有效。
        uint8_t power_enable = 0;  // 有效的 Power 请求中，非 0 为使能，0 为断使能。

        uint8_t switch_mode = 0;  // 非 0 时请求切换到 target_mode。
        RobotMode target_mode = RobotMode::kCsp;  // 模式切换请求的目标模式。

        uint8_t home = 0;  // 非 0 时请求启动或维持回零业务。
    };

    /**
     * @brief 机器人单个实时周期的公共交换数据。
     *
     * IgH 在调用算法周期回调前更新 robot_feedback；算法在回调内更新
     * robot_setpoints 和 service。回调返回后，IgH 使用这些结果驱动其
     * 私有的 CiA402 命令调度与 PDO 写入流程。
     */
    struct RobotCycleData
    {
        AxisFeedback robot_feedback[kMaxRobotAxisCount]{};  // 本周期反馈，由 IgH 写入。
        AxisSetpoint robot_setpoints[kMaxRobotAxisCount]{};  // 运动目标，由算法写入。
        RobotServiceRequest service{};  // 本周期高层服务请求，由算法写入。

        uint8_t robot_axis_count = 0;  // robot_feedback 和 robot_setpoints 中实际参与的轴数。
        uint32_t cycle_time_ns = 0;    // 输入为 0 时使用库内默认 1 ms，非 0 时指定周期；库写回最终采用值。
        uint64_t cycle_count = 0;  // 从 IgH 通信循环开始累计的周期号。
    };

} // namespace robot_interface

#endif
