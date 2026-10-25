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

int16_t last_servo_positions[6] = {2048, 2048, 2048, 2048, 2048, 2048};
uint32_t last_servo_feedback_ms = 0;
uint8_t current_pose_index = 0;
uint8_t current_fk_index = 0;
bool fk_move_pending = false;

constexpr uint32_t OLED_REFRESH_INTERVAL_MS = 80;
constexpr uint32_t BAT_REFRESH_INTERVAL_MS = 200;
constexpr uint32_t STATUS_SYNC_INTERVAL_MS = 500;
constexpr uint32_t BUTTON_STARTUP_GUARD_MS = 1500;

struct IkPose {
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float claw;
    uint16_t duration_ms;
};

struct FkJoints {
    float j1;
    float j2;
    float j3;
    float j4;
    float roll;
    float claw;
};

FkJoints last_requested_fk = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

const IkPose kDemoPoses[] = {
    {200.0f, 0.0f, 200.0f, 0.0f, 0.0f, 0.0f, 1200},
    {220.0f, 80.0f, 180.0f, -10.0f, 0.0f, 15.0f, 1200},
    {180.0f, -80.0f, 240.0f, 8.0f, 15.0f, 0.0f, 1200},
};

const FkJoints kDemoJoints[] = {
    {0.0f, 90.0f, -150.0f, 45.0f, 0.0f, 0.0f},
    {10.0f, 90.0f, -150.0f, 14.0f, 0.0f, 10.0f},
    {-10.0f, 90.0f, -150.0f, 12.0f, 8.0f, 0.0f},
};

// Run one IK preset pose(执行一组逆运动学预设位姿)
void apply_demo_pose(uint8_t index)
{
    const IkPose& pose = kDemoPoses[index % (sizeof(kDemoPoses) / sizeof(kDemoPoses[0]))];
    Serial.printf(
        "[IK] target pose index %u -> x=%.1f y=%.1f z=%.1f pitch=%.1f roll=%.1f claw=%.1f time=%u\n",
        index % (sizeof(kDemoPoses) / sizeof(kDemoPoses[0])),
        pose.x,
        pose.y,
        pose.z,
        pose.pitch,
        pose.roll,
        pose.claw,
        pose.duration_ms
    );
    arm.move(pose.x, pose.y, pose.z, pose.pitch, pose.roll, pose.claw, pose.duration_ms);
    arm.board.buzzer.set(80, 60, 1, 2000);
}

// Request one FK sample from AT32(向AT32请求一组正运动学计算)
void request_fk_demo(uint8_t index)
{
    const FkJoints& joints = kDemoJoints[index % (sizeof(kDemoJoints) / sizeof(kDemoJoints[0]))];
    last_requested_fk = joints;
    fk_move_pending = true;
    arm.request_fk_calc(
        joints.j1,
        joints.j2,
        joints.j3,
        joints.j4,
        joints.roll,
        joints.claw
    );
    Serial.printf(
        "[FK] request joints=(%.1f, %.1f, %.1f, %.1f) roll=%.1f claw=%.1f\n",
        joints.j1,
        joints.j2,
        joints.j3,
        joints.j4,
        joints.roll,
        joints.claw
    );
    arm.board.buzzer.set(60, 60, 1, 2200);
}

// Decode tcp pose from packet(从回包中解析末端位姿)
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

// Refresh cached realtime pose(更新缓存的实时位姿)
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

// Handle parsed AT32 response packets(处理AT32解析完成的响应包)
void at32_packet_callback(PacketTypeDef* rx_packet)
{
    if (rx_packet->elements.id != 0x5A) {
        return;
    }

    switch (rx_packet->elements.cmd) {
        case CMD_GET_CUR_COORDS:
        case CMD_IKINE_RESULT_GET:
            update_pose_from_packet(rx_packet);
            break;

        case CMD_FKINE_RESULT_GET:
        {
            ArmPose_t fk_pose = {};
            parse_pose_from_packet(rx_packet, fk_pose);

            Serial.printf(
                "[FK] joints=(%.1f, %.1f, %.1f, %.1f) roll=%.1f claw=%.1f -> "
                "pose=(x=%.1f, y=%.1f, z=%.1f, pitch=%.1f, roll=%.1f, claw=%.1f)\n",
                last_requested_fk.j1,
                last_requested_fk.j2,
                last_requested_fk.j3,
                last_requested_fk.j4,
                last_requested_fk.roll,
                last_requested_fk.claw,
                fk_pose.x,
                fk_pose.y,
                fk_pose.z,
                fk_pose.pitch,
                fk_pose.roll,
                fk_pose.claw
            );

            if (fk_move_pending) {
                fk_move_pending = false;
    
                arm.move(fk_pose.x, fk_pose.y, fk_pose.z, fk_pose.pitch, fk_pose.roll, fk_pose.claw, 1200);
                Serial.println("[FK] moving arm to computed pose");
            }
            break;
        }

        default:
            break;
    }
}

// Feed bytes into AT32 protocol parser(把串口字节送入AT32协议解析器)
void pump_at32_feedback(void)
{
    while (servo.uart->available()) {
        uint8_t c = servo.uart->read();
        at32_protocol.parsing(&c, 1);
    }
}

// Wait for one fresh feedback frame(等待一帧新的反馈数据)
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

// Export latest servo positions for helper logic(为辅助逻辑提供最新舵机位置)
bool get_last_servo_positions(int16_t out_pos[6])
{
    if (last_servo_feedback_ms == 0) {
        return false;
    }

    memcpy(out_pos, last_servo_positions, sizeof(last_servo_positions));
    return true;
}

// Do one-time startup initialization(执行一次性的启动初始化)
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
    
    delay(2500);
    sync_arm_feedback(300);
    current_pose_index = 0;
    current_fk_index = 0;
    arm.board.button.update();
    arm.board.button.is_clicked(0);
    arm.board.button.is_clicked(1);
    button_enable_ms = millis() + BUTTON_STARTUP_GUARD_MS;
    Serial.println("[Demo] initialization finished");
    Serial.println("[Demo] press BOOT for IK pose demo");
    Serial.println("[Demo] press USER for FK calculate + move demo");
}

// Run the main demo loop(运行演示主循环)
void system_loop_handler(void)
{
    // Parse AT32 feedback continuously(持续解析AT32反馈)
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

    if (now >= button_enable_ms && arm.board.button.is_clicked(0)) {
        current_pose_index = (current_pose_index + 1) % (sizeof(kDemoPoses) / sizeof(kDemoPoses[0]));
        Serial.printf("[IK] move to demo pose index %u\n", current_pose_index);
        apply_demo_pose(current_pose_index);
    }

    if (now >= button_enable_ms && arm.board.button.is_clicked(1)) {
        // Trigger FK demo only when USER is clicked(仅在USER按下时触发正运动学演示)
        Serial.printf("[FK] trigger demo index %u\n", current_fk_index);
        request_fk_demo(current_fk_index);
        current_fk_index = (current_fk_index + 1) % (sizeof(kDemoJoints) / sizeof(kDemoJoints[0]));
    }
}
