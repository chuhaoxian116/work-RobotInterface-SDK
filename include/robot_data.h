#ifndef ROBOT_INTERFACE_ROBOT_DATA_H_
#define ROBOT_INTERFACE_ROBOT_DATA_H_

#include <cstdint>

/**
 * @brief IgH、RobotRuntime 和算法之间的公共机器人数据契约。
 *
 * 本头文件只描述机器人业务语义和实时周期交换的数据，不包含 EtherCAT
 * PDO、IgH 句柄、CiA402 statusword/controlword 或任何具体驱动实现。
 *
 * 周期中的数据所有权：
 * - IgH 写入 AxisFeedback；
 * - 算法读取 AxisFeedback，写入 AxisSetpoint 和 RobotServiceRequest；
 * - IgH 内部调用 RobotRuntime，将服务请求转换为驱动命令后写回 PDO。
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
     * @brief 单轴在当前周期的标准反馈。
     *
     * 由 IgH 从 PDO 映射后写入。位置、速度和转矩的单位必须由机器人
     * 配置统一约定；算法不应把这些字段解释为某个驱动器的原始 PDO 布局。
     */
    struct AxisFeedback
    {
        int32_t actual_position = 0;  // 当前实际位置。
        int32_t actual_velocity = 0;  // 当前实际速度。
        int16_t actual_torque = 0;  // 当前实际转矩。
        uint16_t error_code = 0;  // 当前驱动器或轴的错误码；0 通常表示没有错误。

        uint8_t communication_valid = 0;  // 本周期该轴的通信数据是否有效。
        uint8_t enabled = 0;  // 该轴是否已进入可执行运动命令的使能状态。
        RobotMode active_mode = RobotMode::kCsp;  // 当前反馈的运行模式。
    };

    /**
     * @brief 算法在当前周期给出的单轴运动目标。
     *
     * 算法只写入这些运动目标。控制字和驱动运行模式由 IgH 内部调用
     * RobotRuntime 后生成，不通过本结构暴露给算法。
     */
    struct AxisSetpoint
    {
        int32_t target_position = 0;  // 目标位置。
        int32_t target_velocity = 0;  // 目标速度。
        int16_t target_torque = 0;  // 目标转矩。
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
     * 私有的 RobotRuntime 与 PDO 写入流程。
     */
    struct RobotCycleData
    {
        AxisFeedback robot_feedback[kMaxRobotAxisCount]{};  // 本周期反馈，由 IgH 写入。
        AxisSetpoint robot_setpoints[kMaxRobotAxisCount]{};  // 运动目标，由算法写入。
        RobotServiceRequest service{};  // 本周期高层服务请求，由算法写入。

        uint8_t robot_axis_count = 0;  // robot_feedback 和 robot_setpoints 中实际参与的轴数。
        uint32_t cycle_time_ns = 0;  // 标称实时周期，单位为纳秒。
        uint64_t cycle_count = 0;  // 从 IgH 通信循环开始累计的周期号。
    };

} // namespace robot_interface

#endif
