#include "system_task_handle.h"
#include "Robot_Arm.h"
#include "Arduino.h"
#include <string.h>

void pump_at32_feedback(void);
bool sync_arm_feedback(uint32_t timeout_ms);

namespace {

CommProtocol_t at32_protocol;

uint32_t last_oled_refresh_ms = 0;
uint32_t last_bat_refresh_ms = 0;
uint32_t last_status_sync_ms = 0;
uint32_t button_enable_ms = 0;
uint32_t last_servo_feedback_ms = 0;

int16_t last_servo_positions[6] = {2048, 2048, 2048, 2048, 2048, 2048};

constexpr uint32_t OLED_REFRESH_INTERVAL_MS = 80;
constexpr uint32_t BAT_REFRESH_INTERVAL_MS = 200;
constexpr uint32_t STATUS_SYNC_INTERVAL_MS = 500;
constexpr uint32_t BUTTON_STARTUP_GUARD_MS = 1500;
constexpr uint8_t PICK_PLACE_MOVE_ACC = 18;

struct PoseTarget {
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float claw;
    uint16_t duration_ms;
};

const PoseTarget kPickPlaceSequence[] = {
    {250.0f,   0.0f,  20.0f, -90.0f, 0.0f, -50.0f, 1500},
    {250.0f,   0.0f,  20.0f, -90.0f, 0.0f,   0.0f, 400},
    {250.0f,   0.0f,  20.0f, -90.0f, 0.0f,   0.0f, 400},
    {250.0f,   0.0f,  100.0f, -90.0f, 0.0f,   0.0f, 1500},
    {200.0f,   0.0f, 200.0f,   0.0f, 0.0f,   0.0f, 1500},
    {100.0f, 180.0f, 200.0f,   0.0f, 0.0f,   0.0f, 1500},
    {100.0f, 180.0f,  40.0f, -90.0f, 0.0f,   0.0f, 1500},
    {100.0f, 180.0f,  40.0f, -90.0f, 0.0f, -50.0f, 400},
    {100.0f, 180.0f,  40.0f, -90.0f, 0.0f, -50.0f, 400},
    {100.0f, 180.0f, 200.0f,   0.0f, 0.0f, -50.0f, 1500},
    {200.0f,   0.0f, 200.0f,   0.0f, 0.0f,   0.0f, 1500},
};

// Move to one programmed waypoint(运动到一个编程定点位姿)
void run_pose_step(const PoseTarget& pose, uint8_t step_index)
{
    arm.set_torque(true);
    arm.set_move_acc(PICK_PLACE_MOVE_ACC);

    Serial.printf(
        "[PickPlace] step=%u x=%.1f y=%.1f z=%.1f pitch=%.1f roll=%.1f claw=%.1f time=%u\n",
        (unsigned int)step_index,
        pose.x,
        pose.y,
        pose.z,
        pose.pitch,
        pose.roll,
        pose.claw,
        pose.duration_ms
    );

    arm.move(pose.x, pose.y, pose.z, pose.pitch, pose.roll, pose.claw, pose.duration_ms);
    delay(pose.duration_ms + 150);
}

// Run the full programmed pick-and-place routine(运行整套编程定点夹取搬运流程)
void run_pick_place_sequence()
{
    Serial.println("[PickPlace] sequence start");

    for (uint8_t i = 0; i < (sizeof(kPickPlaceSequence) / sizeof(kPickPlaceSequence[0])); ++i) {
        run_pose_step(kPickPlaceSequence[i], (uint8_t)(i + 1));
        // delay(400);
    }

    Serial.println("[PickPlace] sequence finished");
    arm.board.buzzer.set(60, 60, 2, 2200);
}

// Decode tcp pose fields from one AT32 packet(从一帧AT32数据中解析末端位姿字段)
void parse_pose_from_packet(PacketTypeDef* rx_packet, ArmPose_t& pose)
{
    if (rx_packet->elements.length < 14) {
        return;
    }

    int16_t raw_x = (int16_t)((rx_packet->elements.args[1] << 8) | rx_packet->elements.args[0]);
    int16_t raw_y = (int16_t)((rx_packet->elements.args[3] << 8) | rx_packet->elements.args[2]);
    int16_t raw_z = (int16_t)((rx_packet->elements.args[5] << 8) | rx_packet->elements.args[4]);
    int16_t raw_pitch = (int16_t)((rx_packet->elements.args[7] << 8) | rx_packet->elements.args[6]);
    int16_t raw_roll = (int16_t)((rx_packet->elements.args[9] << 8) | rx_packet->elements.args[8]);
    int16_t raw_claw = (int16_t)((rx_packet->elements.args[11] << 8) | rx_packet->elements.args[10]);

    pose.x = (float)raw_x;
    pose.y = (float)raw_y;
    pose.z = (float)raw_z;
    pose.pitch = (float)raw_pitch / 10.0f;
    pose.roll = (float)raw_roll;
    pose.claw = (float)raw_claw;
}

// Cache latest pose and servo feedback locally(在本地缓存最新位姿和舵机反馈)
void update_pose_from_packet(PacketTypeDef* rx_packet)
{
    parse_pose_from_packet(rx_packet, arm.current_pose);

    if (rx_packet->elements.cmd == CMD_GET_CUR_COORDS && rx_packet->elements.length >= 26) {
        for (int i = 0; i < 6; ++i) {
            int idx = 12 + i * 2;
            last_servo_positions[i] = (int16_t)((rx_packet->elements.args[idx + 1] << 8) | rx_packet->elements.args[idx]);
        }
        last_servo_feedback_ms = millis();
    }
}

} // namespace

// Handle parsed packets returned from AT32(处理AT32返回并已解析完成的数据包)
void at32_packet_callback(PacketTypeDef* rx_packet)
{
    if (rx_packet->elements.id != 0x5A) {
        return;
    }

    if (rx_packet->elements.cmd == CMD_GET_CUR_COORDS) {
        update_pose_from_packet(rx_packet);
    }
}

// Feed uart bytes into the AT32 protocol parser(把串口字节送入AT32协议解析器)
void pump_at32_feedback(void)
{
    while (servo.uart->available()) {
        uint8_t c = servo.uart->read();
        at32_protocol.parsing(&c, 1);
    }
}

// Wait until one fresh status frame is received(等待收到一帧新的状态反馈)
bool sync_arm_feedback(uint32_t timeout_ms)
{
    arm.update_status();
    uint32_t start = millis();
    uint32_t prev_feedback_ms = last_servo_feedback_ms;

    while (millis() - start < timeout_ms) {
        pump_at32_feedback();
        if (last_servo_feedback_ms != 0 && last_servo_feedback_ms != prev_feedback_ms) {
            return true;
        }
        delay(2);
    }

    return last_servo_feedback_ms != 0;
}

// Export latest cached servo positions for board helpers(给板级辅助逻辑导出最新舵机位置)
bool get_last_servo_positions(int16_t out_pos[6])
{
    if (last_servo_feedback_ms == 0) {
        return false;
    }

    memcpy(out_pos, last_servo_positions, sizeof(last_servo_positions));
    return true;
}

// Run one-time startup initialization only once(仅执行一次的启动初始化)
void register_system_task(esp_event_loop_handle_t *event_loop)
{
    (void)event_loop;

    setCpuFrequencyMhz(240);
    Serial.begin(115200);

    servo.begin(Serial1, 1000000, 16, 17);
    arm.begin();
    arm.set_torque(true);
    delay(1000);
    arm.reset_all(1500);

    at32_protocol.begin();
    at32_protocol.register_success_callback(at32_packet_callback);

    sync_arm_feedback(300);
    arm.board.button.update();
    arm.board.button.is_clicked(0);
    arm.board.button.is_clicked(1);
    button_enable_ms = millis() + BUTTON_STARTUP_GUARD_MS;

    Serial.println("[Demo] initialization finished");
    Serial.println("[Demo] press BOOT to run programmed pick-place sequence");
}

// Keep the programmed pick-place demo alive in the main loop(在主循环中维持编程定点夹取搬运演示运行)
void system_loop_handler(void)
{
    // Continuously receive feedback from AT32(持续接收AT32反馈)
    pump_at32_feedback();
    arm.board.button.update();
    arm.board.buzzer.update();

    uint32_t now = millis();

    if (now - last_bat_refresh_ms >= BAT_REFRESH_INTERVAL_MS) {
        last_bat_refresh_ms = now;
        arm.board.bat.update();
    }

    if (now - last_status_sync_ms >= STATUS_SYNC_INTERVAL_MS) {
        last_status_sync_ms = now;
        arm.update_status();
    }

    if (now - last_oled_refresh_ms >= OLED_REFRESH_INTERVAL_MS) {
        last_oled_refresh_ms = now;
        arm.board.oled.show_status(
            arm.current_pose.x,
            arm.current_pose.y,
            arm.current_pose.z,
            arm.current_pose.pitch,
            arm.current_pose.roll,
            arm.current_pose.claw
        );
    }

    // BOOT button runs the full programmed routine(BOOT按键运行整套编程流程)
    if (now >= button_enable_ms && arm.board.button.is_clicked(0)) {
        run_pick_place_sequence();
    }
}
