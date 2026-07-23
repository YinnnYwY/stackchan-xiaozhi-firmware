#include "wifi_board.h"
#include "cores3_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "config.h"
#include "power_save_timer.h"
#include "i2c_device.h"
#include "axp2101.h"
#include "mcp_server.h"
#include "assets/lang_config.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ili9341.h>
#include <esp_timer.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctime>
#include <vector>
#include <functional>
#include <cJSON.h>
#include "esp_video.h"
#include "lvgl.h"
#include "SCSCL.h"
#include "i2c_bus.h"
#include "bmi270_api.h"
#include "bmi2.h"

// BMI270 SDK 在 .a 里有这些 public 符号但头文件未暴露——自己声明用来绕过 bmi270_sensor_create 硬编码 0x68 的限制
extern "C" {
    int8_t bmi270_init(struct bmi2_dev *dev);
    extern const uint8_t bmi270_config_file[];
}

// BMI270 自定义 I2C read/write（地址 0x69）
static int8_t Bmi270I2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    auto dev = (i2c_master_dev_handle_t)intf_ptr;
    return i2c_master_transmit_receive(dev, &reg_addr, 1, data, len, 200) == ESP_OK ? 0 : -1;
}

static int8_t Bmi270I2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    static uint8_t big_buf[8300];  // BMI270 config blob ~8KB
    if (len + 1 > sizeof(big_buf)) return -1;
    auto dev = (i2c_master_dev_handle_t)intf_ptr;
    big_buf[0] = reg_addr;
    memcpy(big_buf + 1, data, len);
    return i2c_master_transmit(dev, big_buf, len + 1, 500) == ESP_OK ? 0 : -1;
}

static void Bmi270DelayUs(uint32_t period_us, void *intf_ptr) {
    if (period_us < 1000) {
        esp_rom_delay_us(period_us);
    } else {
        vTaskDelay(pdMS_TO_TICKS((period_us + 999) / 1000));
    }
}

#define TAG "M5StackCoreS3Board"

class FaceTracker;

class StackChanServo {
public:
    bool Begin() {
        if (!bus_.begin(UART_NUM_1, 1000000, 6, 7)) {
            ESP_LOGE("Servo", "SCS bus begin failed");
            return false;
        }
        ESP_LOGI("Servo", "SCS bus init OK on UART1 GPIO6/7 @1Mbps");
        MoveTo(0, 30, 1500);

        esp_timer_create_args_t args = {};
        args.callback = &StackChanServo::IdleScanCb;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "servo_idle";
        args.skip_unhandled_events = true;
        esp_timer_create(&args, &idle_timer_);
        esp_timer_start_periodic(idle_timer_, 4000000);
        scan_running_ = true;
        return true;
    }

    void MoveTo(int yaw_deg, int pitch_deg, int time_ms) {
        if (yaw_deg < -45) yaw_deg = -45;
        if (yaw_deg > 45) yaw_deg = 45;
        if (pitch_deg < 5) pitch_deg = 5;
        if (pitch_deg > 60) pitch_deg = 60;
        int yaw_pos = 460 + yaw_deg * 16 / 5;
        int pitch_pos = 620 + pitch_deg * 16 / 5;
        ESP_LOGI("Servo", "MoveTo yaw=%d pitch=%d t=%d (caller=%p)",
                 yaw_deg, pitch_deg, time_ms, __builtin_return_address(0));
        bus_.WritePos(1, yaw_pos, time_ms, 0);
        bus_.WritePos(2, pitch_pos, time_ms, 0);
        // 记当前命令位置——Nod/Shake 动画用这个作 base，而不是 tracker_->GetYaw()
        // 后者只反映人脸追踪目标，不反映 MCP head.move 设的硬位置。
        last_yaw_deg_ = yaw_deg;
        last_pitch_deg_ = pitch_deg;
    }

    void PauseScan() {
        if (scan_running_ && idle_timer_) {
            esp_timer_stop(idle_timer_);
            scan_running_ = false;
        }
    }

    void ResumeScan() {
        if (!scan_running_ && idle_timer_) {
            esp_timer_start_periodic(idle_timer_, 4000000);
            scan_running_ = true;
        }
    }

    void Center() { MoveTo(0, 30, 600); }

    void SetFaceTracker(FaceTracker* ft) { tracker_ = ft; }

    // 当前命令位置（最后一次 MoveTo 设的）— Nod/Shake 用这个作 base
    int GetCurrentYaw() const { return last_yaw_deg_; }
    int GetCurrentPitch() const { return last_pitch_deg_; }

    void Nod();
    void Shake();
    void Tilt();

    bool IsAnimating() const { return anim_running_; }

private:
    static void IdleScanCb(void* arg) {
        // 待机不再随机甩头(用户要"别老动、对准我"):保持当前朝向不动。
        // 头部只在对话时的人脸/动作追踪,或 LLM head.move 指令下才移动。
        (void)arg;
    }

    SCSCL bus_;
    esp_timer_handle_t idle_timer_ = nullptr;
    FaceTracker* tracker_ = nullptr;
    bool scan_running_ = false;
    volatile bool anim_running_ = false;
    int last_yaw_deg_ = 0;
    int last_pitch_deg_ = 30;
};

class FaceTracker {
    static constexpr int DS_W = 40;
    static constexpr int DS_H = 30;
public:
    void Start(EspVideo* camera, StackChanServo* servo) {
        camera_ = camera;
        servo_ = servo;
        if (!camera_ || !servo_) return;

        memset(prev_frame_, 0, sizeof(prev_frame_));
        paused_ = true;
        xTaskCreatePinnedToCore(TaskFunc, "face_track", 4096, this, 1, &task_, 1);
        ESP_LOGI("FaceTrack", "Started (paused until conversation)");
    }

    void Pause(bool resume_scan = true) {
        if (!paused_) {
            paused_ = true;
            // 睡眠/暂停时,别让"着急找人"或"注意到你"的表情卡住。
            if (tracking_) { tracking_ = false; if (tracking_cb_) tracking_cb_(false); }
            if (searching_) { searching_ = false; if (search_cb_) search_cb_(false); }
            if (resume_scan) servo_->ResumeScan();
            ESP_LOGI("FaceTrack", "Paused (scan=%d)", resume_scan);
        }
    }

    void Resume() {
        // 当 manual_lock_ 被 LLM 工具锁住时，忽略状态机周期 Resume，
        // 否则 Application::Listening/Speaking 每次刷新都会自动 Resume，
        // 让用户"派蒙转 30 度"立刻被人脸追踪复位。
        if (manual_lock_) return;
        if (paused_) {
            paused_ = false;
            has_prev_ = false;
            servo_->PauseScan();
            ESP_LOGI("FaceTrack", "Resumed");
        }
    }

    // 手动锁：lock=true 后，外部 Resume() 无效（除非 lock=false 解锁再 Resume）。
    // LLM head.move/center 用这个保证位置不被自动 tracker 抢回去。
    void SetManualLock(bool locked) {
        manual_lock_ = locked;
        ESP_LOGI("FaceTrack", "Manual lock %s", locked ? "ENABLED" : "DISABLED");
    }

    bool IsManualLocked() const { return manual_lock_; }

    // 摄像头追踪到的人脸位置回调给头像眼睛(即使没有舵机/脖子也能"看向你")
    void SetGazeCallback(std::function<void(float, float)> cb) { gaze_cb_ = std::move(cb); }

    // 找不到人、持续一段时间后开始"着急找"(true)/重新找到或暂停时结束(false)
    void SetSearchingCallback(std::function<void(bool)> cb) { search_cb_ = std::move(cb); }

    // 真的在跟踪明确动静时(true)/停止跟踪时(false)——让"她在动"始终配一个表情理由
    void SetTrackingCallback(std::function<void(bool)> cb) { tracking_cb_ = std::move(cb); }

    bool IsPaused() const { return paused_; }
    float GetYaw() const { return yaw_; }
    float GetPitch() const { return pitch_; }

private:
    static void TaskFunc(void* arg) {
        auto* self = static_cast<FaceTracker*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (self->paused_ || !self->camera_->IsOk()) continue;
            self->Track();
        }
    }

    void Track() {
        uint8_t cur_frame[DS_W * DS_H];
        int sum_x = 0, sum_y = 0, count = 0;

        bool ok = camera_->PeekFrame([&](const uint8_t* data, size_t len, uint16_t w, uint16_t h) {
            if (w == 0 || h == 0) return;
            int sx = w / DS_W;
            int sy = h / DS_H;
            for (int dy = 0; dy < DS_H; dy++) {
                for (int dx = 0; dx < DS_W; dx++) {
                    int src_offset = (dy * sy * w + dx * sx) * 2;
                    if ((size_t)src_offset >= len) { cur_frame[dy * DS_W + dx] = 0; continue; }
                    cur_frame[dy * DS_W + dx] = data[src_offset];
                }
            }

            if (!has_prev_) return;

            for (int dy = 3; dy < DS_H - 1; dy++) {
                for (int dx = 1; dx < DS_W - 1; dx++) {
                    int idx = dy * DS_W + dx;
                    uint8_t brightness = cur_frame[idx];
                    if (brightness > 200) continue;
                    int diff = abs((int)cur_frame[idx] - (int)prev_frame_[idx]);
                    if (diff > 28) {                        // 提高单像素判定门槛,滤掉传感器噪点
                        sum_x += dx;
                        sum_y += dy;
                        count++;
                    }
                }
            }
        });

        memcpy(prev_frame_, cur_frame, sizeof(prev_frame_));
        if (!has_prev_) { has_prev_ = true; return; }
        if (!ok) return;

        int total_pixels = (DS_W - 2) * (DS_H - 4);
        // 之前 count<3 太敏感——一两个噪点像素就判定"有动静",导致主人没动
        // 她却在动。提高到 count<16(约 1.6% 画面),要求更明确的动作才跟踪。
        if (count < 16 || count > total_pixels / 3) {
            no_move_count_++;
            if (no_move_count_ > 6 && tracking_) {
                tracking_ = false;
                if (tracking_cb_) tracking_cb_(false);   // 停止跟踪:收回"注意到你"的表情
            }
            // 找不到人:先等一会儿再"着急找"——常驻追踪下拉长到约 10s,
            // 避免主人只是安静坐一会儿就被当成"不见了"一直着急扫视。
            const int kSearchStartCycles = 100;  // TaskFunc 每 100ms 跑一次 → ~10s
            if (no_move_count_ == kSearchStartCycles) {
                searching_ = true;
                search_phase_ = 0;
                if (search_cb_) search_cb_(true);
                ESP_LOGI("FaceTrack", "Lost person, start searching");
            }
            if (searching_) {
                search_phase_++;
                float sweep = sinf(search_phase_ * 0.05f);         // -1..1
                if (gaze_cb_) gaze_cb_(sweep, 0.15f * sinf(search_phase_ * 0.03f));
                if (search_phase_ % 8 == 0) {                       // 每 ~0.8s 挪一次舵机,别太frantic
                    servo_->MoveTo((int)(sweep * 30.0f), 30, 500);
                }
            }
            return;
        }

        if (searching_) {
            searching_ = false;
            if (search_cb_) search_cb_(false);
            ESP_LOGI("FaceTrack", "Found person, stop searching");
        }
        no_move_count_ = 0;
        if (!tracking_) {
            servo_->PauseScan();
            tracking_ = true;
            if (tracking_cb_) tracking_cb_(true);   // 开始跟踪真实动静:亮出"注意到你"的表情
        }

        float cx = (float)sum_x / count;
        float cy = (float)sum_y / count;
        float target_x = (cx - DS_W / 2.0f) / (DS_W / 2.0f);
        float target_y = (cy - DS_H / 2.0f) / (DS_H / 2.0f);

        smooth_x_ = smooth_x_ * 0.3f + target_x * 0.7f;
        smooth_y_ = smooth_y_ * 0.3f + target_y * 0.7f;

        if (fabsf(smooth_x_) < 0.03f) smooth_x_ = 0;
        if (fabsf(smooth_y_) < 0.03f) smooth_y_ = 0;

        yaw_ -= smooth_x_ * 6.0f;
        pitch_ -= smooth_y_ * 4.0f;
        if (yaw_ < -45) yaw_ = -45;
        if (yaw_ > 45) yaw_ = 45;
        if (pitch_ < 5) pitch_ = 5;
        if (pitch_ > 60) pitch_ = 60;

        servo_->MoveTo((int)yaw_, (int)pitch_, 150);
        if (gaze_cb_) gaze_cb_(-target_x, target_y);   // 眼睛跟着追踪目标动(镜像水平)
    }

    EspVideo* camera_ = nullptr;
    StackChanServo* servo_ = nullptr;
    TaskHandle_t task_ = nullptr;
    volatile bool paused_ = false;
    volatile bool manual_lock_ = false;  // LLM head 工具锁住时为 true
    bool tracking_ = false;
    bool has_prev_ = false;
    int no_move_count_ = 0;
    float yaw_ = 0.0f;
    float pitch_ = 30.0f;
    float smooth_x_ = 0.0f;
    float smooth_y_ = 0.0f;
    uint8_t prev_frame_[DS_W * DS_H];
    std::function<void(float, float)> gaze_cb_;
    bool searching_ = false;
    int search_phase_ = 0;
    std::function<void(bool)> search_cb_;
    std::function<void(bool)> tracking_cb_;
};

// ── 闹钟系统:本地存储(NVS)+ 本地计时(不依赖云端推送)──────────────
// 云端到设备是"拉取式"(reverse-MCP),没法在闹钟时间到时主动戳设备,
// 所以触发靠固件自己每 ~20s 拿本机时间(开机时小智 OTA 服务器已经把
// 系统时间设好,见 ota.cc settimeofday)比对;命中后走 Alert()(本地
// 状态/表情/提示音,离线也响)+ SendUserText()(复用摸头/触摸那条
// canned-message 通道,让云端用她的语气自然"叫"一声,最佳努力)。
struct AlarmEntry {
    int id = 0;
    int hour = 0;
    int minute = 0;
    uint8_t weekday_mask = 0;    // bit0=周日..bit6=周六;0 = 一次性
    bool enabled = true;
    std::string label;           // 语音里提到的简短理由,比如"起床"
    int32_t last_fired_ymd = 0;  // 20260722 这种整数,防止同一天内重复触发
};

class AlarmManager {
public:
    void Load() {
        Settings settings("alarms", false);
        std::string json = settings.GetString("list", "[]");
        alarms_.clear();
        cJSON* arr = cJSON_Parse(json.c_str());
        if (arr && cJSON_IsArray(arr)) {
            cJSON* item;
            cJSON_ArrayForEach(item, arr) {
                AlarmEntry a;
                cJSON* v;
                v = cJSON_GetObjectItem(item, "id");     a.id = v ? v->valueint : 0;
                v = cJSON_GetObjectItem(item, "hour");   a.hour = v ? v->valueint : 0;
                v = cJSON_GetObjectItem(item, "minute"); a.minute = v ? v->valueint : 0;
                v = cJSON_GetObjectItem(item, "wd");     a.weekday_mask = v ? (uint8_t)v->valueint : 0;
                v = cJSON_GetObjectItem(item, "en");     a.enabled = v ? cJSON_IsTrue(v) : true;
                v = cJSON_GetObjectItem(item, "label");  a.label = (v && cJSON_IsString(v)) ? v->valuestring : "";
                v = cJSON_GetObjectItem(item, "lf");     a.last_fired_ymd = v ? v->valueint : 0;
                if (a.id > 0) alarms_.push_back(a);
            }
        }
        if (arr) cJSON_Delete(arr);
        next_id_ = 1;
        for (auto& a : alarms_) if (a.id >= next_id_) next_id_ = a.id + 1;
        ESP_LOGI("Alarm", "Loaded %d alarm(s)", (int)alarms_.size());
        for (auto& a : alarms_) {
            ESP_LOGI("Alarm", "  #%d %02d:%02d wd=0x%02x en=%d label=%s",
                     a.id, a.hour, a.minute, a.weekday_mask, a.enabled, a.label.c_str());
        }
    }

    void Save() {
        cJSON* arr = cJSON_CreateArray();
        for (auto& a : alarms_) {
            cJSON* item = cJSON_CreateObject();
            cJSON_AddNumberToObject(item, "id", a.id);
            cJSON_AddNumberToObject(item, "hour", a.hour);
            cJSON_AddNumberToObject(item, "minute", a.minute);
            cJSON_AddNumberToObject(item, "wd", a.weekday_mask);
            cJSON_AddBoolToObject(item, "en", a.enabled);
            cJSON_AddStringToObject(item, "label", a.label.c_str());
            cJSON_AddNumberToObject(item, "lf", a.last_fired_ymd);
            cJSON_AddItemToArray(arr, item);
        }
        char* s = cJSON_PrintUnformatted(arr);
        Settings settings("alarms", true);
        settings.SetString("list", s ? s : "[]");
        if (s) cJSON_free(s);
        cJSON_Delete(arr);
    }

    int Add(int hour, int minute, uint8_t weekday_mask, const std::string& label) {
        AlarmEntry a;
        a.id = next_id_++;
        a.hour = hour; a.minute = minute; a.weekday_mask = weekday_mask;
        a.enabled = true; a.label = label; a.last_fired_ymd = 0;
        alarms_.push_back(a);
        Save();
        return a.id;
    }

    bool Cancel(int id) {
        for (auto it = alarms_.begin(); it != alarms_.end(); ++it) {
            if (it->id == id) { alarms_.erase(it); Save(); return true; }
        }
        return false;
    }

    // 带上"现在几点"——之前主人问"为什么没提醒"时,LLM 没有任何办法核实当前时间,
    // 只能瞎猜(比如说"还没到4:30"却可能压根不对)。现在她查闹钟列表时能顺带看到
    // 设备自己认为的当前时间,回答才有依据。
    std::string ListText() const {
        char now_buf[64];
        time_t now = time(nullptr);
        struct tm tmv;
        localtime_r(&now, &tmv);
        if (tmv.tm_year < 2025 - 1900) {
            snprintf(now_buf, sizeof(now_buf), "(设备当前时间还没同步,可能不准)");
        } else {
            static const char* wd[7] = {"日", "一", "二", "三", "四", "五", "六"};
            snprintf(now_buf, sizeof(now_buf), "现在是 %02d:%02d(周%s)。",
                     tmv.tm_hour, tmv.tm_min, wd[tmv.tm_wday]);
        }
        std::string s = now_buf;
        if (alarms_.empty()) { s += "当前没有设置任何闹钟"; return s; }
        for (auto& a : alarms_) {
            char buf[96];
            snprintf(buf, sizeof(buf), " #%d %02d:%02d %s%s%s%s；",
                     a.id, a.hour, a.minute, WeekdayMaskText(a.weekday_mask).c_str(),
                     a.label.empty() ? "" : " ", a.label.c_str(),
                     a.enabled ? "" : "(已关闭)");
            s += buf;
        }
        return s;
    }

    // 每 ~20s 调用一次;命中的闹钟通过回调交出去"响铃"(只传值,不传引用/指针,
    // 避免回调延后执行时悬空)。一次性闹钟响过即自动关闭。
    void CheckAndFire(const std::function<void(int id, int hour, int minute, const std::string& label)>& fire_cb) {
        time_t now = time(nullptr);
        struct tm tmv;
        localtime_r(&now, &tmv);
        if (tmv.tm_year < 2025 - 1900) {
            ESP_LOGW("Alarm", "System time not synced yet (tm_year=%d), skip check", tmv.tm_year + 1900);
            return;  // 系统时间还没同步(未做过 OTA 检查),别误触发
        }
        // 每次检查都打一条当前时间日志(低频、20s 一次,方便日后排查"到点没响"问题)
        ESP_LOGI("Alarm", "Check tick: now=%02d:%02d wd=%d", tmv.tm_hour, tmv.tm_min, tmv.tm_wday);
        int32_t ymd = (tmv.tm_year + 1900) * 10000 + (tmv.tm_mon + 1) * 100 + tmv.tm_mday;
        bool changed = false;
        for (auto& a : alarms_) {
            if (!a.enabled) continue;
            if (a.hour != tmv.tm_hour || a.minute != tmv.tm_min) continue;
            if (a.last_fired_ymd == ymd) continue;  // 这一天已经响过,避免 20s 轮询内重复触发
            if (a.weekday_mask != 0 && !(a.weekday_mask & (1 << tmv.tm_wday))) continue;

            a.last_fired_ymd = ymd;
            if (a.weekday_mask == 0) a.enabled = false;  // 一次性闹钟响过就关闭
            changed = true;
            if (fire_cb) fire_cb(a.id, a.hour, a.minute, a.label);
        }
        if (changed) Save();
    }

    static uint8_t ParseRepeat(const std::string& r) {
        if (r == "daily")    return 0b1111111;
        if (r == "weekdays") return 0b0111110;  // 周一~周五
        if (r == "weekends") return 0b1000001;  // 周日+周六
        return 0;  // "once" 及其它未识别值,一律按一次性处理
    }

    static std::string WeekdayMaskText(uint8_t mask) {
        if (mask == 0) return "(一次性)";
        if (mask == 0b1111111) return "(每天)";
        if (mask == 0b0111110) return "(工作日)";
        if (mask == 0b1000001) return "(周末)";
        static const char* names[7] = {"日", "一", "二", "三", "四", "五", "六"};
        std::string s = "(每周";
        for (int i = 0; i < 7; i++) if (mask & (1 << i)) s += names[i];
        s += ")";
        return s;
    }

    // UTF-8 安全截断(只在字符边界切,避免切出半个汉字乱码)
    static std::string Utf8Truncate(const std::string& s, size_t max_bytes) {
        if (s.size() <= max_bytes) return s;
        size_t cut = max_bytes;
        while (cut > 0 && (static_cast<unsigned char>(s[cut]) & 0xC0) == 0x80) cut--;
        return s.substr(0, cut);
    }

private:
    std::vector<AlarmEntry> alarms_;
    int next_id_ = 1;
};

struct ServoAnimCtx {
    StackChanServo* servo;
    int base_yaw;
    int base_pitch;
};

void StackChanServo::Nod() {
    if (anim_running_) return;
    anim_running_ = true;
    // 用 servo 当前命令位置作 base —— 不用 tracker_->GetYaw()，因为后者只反映
    // 人脸追踪的目标值，不反映 LLM head.move 工具设的硬位置。这样 Nod 在
    // 当前位置上下点头，不会把头甩回 (0, 30)。
    auto* ctx = new ServoAnimCtx{this, GetCurrentYaw(), GetCurrentPitch()};
    if (tracker_) tracker_->Pause(false);
    xTaskCreatePinnedToCore([](void* arg) {
        auto* c = static_cast<ServoAnimCtx*>(arg);
        auto* s = c->servo;
        int y = c->base_yaw, p = c->base_pitch;
        s->MoveTo(y, p - 10, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        s->MoveTo(y, p + 5, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        s->MoveTo(y, p - 8, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        s->MoveTo(y, p, 300);
        vTaskDelay(pdMS_TO_TICKS(300));
        s->anim_running_ = false;
        if (s->tracker_) s->tracker_->Resume();
        delete c;
        vTaskDelete(nullptr);
    }, "nod", 2048, ctx, 2, nullptr, 1);
}

void StackChanServo::Shake() {
    if (anim_running_) return;
    anim_running_ = true;
    auto* ctx = new ServoAnimCtx{this, GetCurrentYaw(), GetCurrentPitch()};
    if (tracker_) tracker_->Pause(false);
    xTaskCreatePinnedToCore([](void* arg) {
        auto* c = static_cast<ServoAnimCtx*>(arg);
        auto* s = c->servo;
        int y = c->base_yaw, p = c->base_pitch;
        s->MoveTo(y - 15, p, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        s->MoveTo(y + 15, p, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        s->MoveTo(y - 10, p, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        s->MoveTo(y, p, 300);
        vTaskDelay(pdMS_TO_TICKS(300));
        s->anim_running_ = false;
        if (s->tracker_) s->tracker_->Resume();
        delete c;
        vTaskDelete(nullptr);
    }, "shake", 2048, ctx, 2, nullptr, 1);
}

void StackChanServo::Tilt() {
    if (anim_running_) return;
    anim_running_ = true;
    auto* ctx = new ServoAnimCtx{this, GetCurrentYaw(), GetCurrentPitch()};
    if (tracker_) tracker_->Pause(false);
    xTaskCreatePinnedToCore([](void* arg) {
        auto* c = static_cast<ServoAnimCtx*>(arg);
        auto* s = c->servo;
        int y = c->base_yaw, p = c->base_pitch;
        s->MoveTo(y + 10, p - 10, 400);
        vTaskDelay(pdMS_TO_TICKS(1500));
        s->MoveTo(y, p, 500);
        vTaskDelay(pdMS_TO_TICKS(500));
        s->anim_running_ = false;
        if (s->tracker_) s->tracker_->Resume();
        delete c;
        vTaskDelete(nullptr);
    }, "tilt", 2048, ctx, 2, nullptr, 1);
}

static bool EnableServoPowerViaPy32(i2c_master_bus_handle_t i2c_bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x6F,
        .scl_speed_hz = 100000,
        .scl_wait_us = 0,
        .flags = { .disable_ack_check = 0 },
    };
    i2c_master_dev_handle_t dev = nullptr;
    esp_err_t err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PY32: failed to add device: %s", esp_err_to_name(err));
        return false;
    }

    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        uint8_t reg = 0x02;
        uint8_t ver = 0;
        err = i2c_master_transmit_receive(dev, &reg, 1, &ver, 1, 200);
        if (err == ESP_OK && ver != 0 && ver != 0xFF) {
            ESP_LOGI(TAG, "PY32 found! version=%d, enabling VM_EN", ver);
            uint8_t buf[2];
            reg = 0x03; i2c_master_transmit_receive(dev, &reg, 1, &buf[1], 1, 200);
            buf[0] = 0x03; buf[1] |= 0x01; i2c_master_transmit(dev, buf, 2, 200);
            reg = 0x09; i2c_master_transmit_receive(dev, &reg, 1, &buf[1], 1, 200);
            buf[0] = 0x09; buf[1] |= 0x01; i2c_master_transmit(dev, buf, 2, 200);
            reg = 0x05; i2c_master_transmit_receive(dev, &reg, 1, &buf[1], 1, 200);
            buf[0] = 0x05; buf[1] |= 0x01; i2c_master_transmit(dev, buf, 2, 200);
            ESP_LOGI(TAG, "Servo power enabled (VM_EN)");
            return true;
        }
        ESP_LOGD(TAG, "PY32 attempt %d: err=%s ver=0x%02X", i, esp_err_to_name(err), ver);
    }

    ESP_LOGW(TAG, "PY32 not found after 10 attempts");
    i2c_master_bus_rm_device(dev);
    return false;
}

namespace shizhou_avatar {

enum class Expression {
    Neutral, Happy, Angry, Sad, Sleepy,
    Loving, Crying,
    Kissy, Cool, Confident,
    Shocked, Thinking, Surprised, Confused,
    Embarrassed, Silly, Winking, Laughing, Funny, Relaxed, Delicious
};

struct Overlay {
    bool tear = false;
    bool heart_eyes = false;
    bool kiss_heart = false;
    bool cheek_blush = false;
    bool cool_glasses = false;
    bool excl_mark = false;
    bool think_bubble = false;
    bool star_burst = false;
    bool wave_squiggle = false;
    bool drool = false;
    bool laugh_lines = false;
    bool question_mark = false;
    bool zzz = false;
};

class LvglAvatar {
public:
    LvglAvatar() = default;
    ~LvglAvatar() { Destroy(); }

    bool Init(lv_obj_t* parent, int w, int h) {
        if (canvas_) return true;
        w_ = w; h_ = h;
        size_t bytes = (size_t)w * h * 2;
        buf_ = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
        if (!buf_) return false;
        canvas_ = lv_canvas_create(parent);
        lv_canvas_set_buffer(canvas_, buf_, w, h, LV_COLOR_FORMAT_RGB565);
        lv_obj_align(canvas_, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_clear_flag(canvas_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_move_background(canvas_);
        timer_ = lv_timer_create(&LvglAvatar::TimerCb, 50, this);
        next_blink_ms_ = 3000;
        last_saccade_ms_ = 0;
        Draw();
        return true;
    }

    void Destroy() {
        if (timer_) { lv_timer_delete(timer_); timer_ = nullptr; }
        if (canvas_) { lv_obj_delete(canvas_); canvas_ = nullptr; }
        if (buf_)   { heap_caps_free(buf_); buf_ = nullptr; }
    }

    bool IsReady() const { return canvas_ != nullptr; }

    void SetExpression(Expression e) { expression_ = e; }
    void SetOverlay(const Overlay& o) { overlay_ = o; }
    void StartSpeaking(uint32_t duration_ms) {
        speaking_until_ms_ = lv_tick_get() + duration_ms;
    }
    void StopSpeaking() { speaking_until_ms_ = 0; }

    // 摄像头追踪结果直接喂给眼睛视线(不依赖舵机/脖子也能"看向你")
    void SetGazeTarget(float gh, float gv) {
        if (gh < -1) gh = -1; else if (gh > 1) gh = 1;
        if (gv < -1) gv = -1; else if (gv > 1) gv = 1;
        gaze_h_ = gh; gaze_v_ = gv;
        gaze_ext_ms_ = lv_tick_get();
    }

    // 摄像头找不到人时:临时显示"着急找人"的表情(眼睛已经在扫,见 SetGazeTarget 调用方)。
    // 只借用 Draw() 那一帧的表情显示,不改 expression_ 本身——LLM 的正常表情不会被打乱。
    void SetSearching(bool s) { searching_ = s; }

    // 明确进入/退出"睡眠"(self.sleep.* 工具触发,不再随 Idle 自动睡)。
    // activity: 0=睡觉(闭眼+zzz) 1=看书 2=写代码 3=看论文——只借用显示,
    // 不动 expression_/overlay_ 本身。
    void SetSleeping(bool s, int activity = 0) { sleeping_ = s; sleep_activity_ = activity; }

    // 正在跟踪真实动静时(true)/停止时(false):睁大眼睛表示"注意到你了",
    // 让"她在动"这件事总有个看得出来的理由,不是无缘无故动。
    void SetAttentive(bool a) { attentive_ = a; }

private:
    static void TimerCb(lv_timer_t* t) {
        static_cast<LvglAvatar*>(lv_timer_get_user_data(t))->OnTick();
    }

    void UpdateBreathParams() {
        breath_amp_ = 3.0f;
        breath_period_steps_ = 100;
        breath_paused_ = false;
        switch (expression_) {
            case Expression::Relaxed:
                breath_amp_ = 7.0f;
                breath_period_steps_ = 160;
                break;
            case Expression::Shocked:
                breath_paused_ = true;
                break;
            default: break;
        }
    }

    bool BlinkAllowed() const {
        switch (expression_) {
            case Expression::Cool:
            case Expression::Confident:
            case Expression::Shocked:
            case Expression::Winking:
            case Expression::Kissy:
                return false;
            default:
                return true;
        }
    }

    bool SlowBlink() const {
        return expression_ == Expression::Thinking || expression_ == Expression::Relaxed;
    }

    bool SaccadeEnabled() const {
        switch (expression_) {
            case Expression::Cool:
            case Expression::Confident:
            case Expression::Shocked:
            case Expression::Thinking:
            case Expression::Embarrassed:
            case Expression::Winking:
                return false;
            default:
                return true;
        }
    }

    void GetGazeOverride(float* gh, float* gv) const {
        switch (expression_) {
            case Expression::Thinking:
                *gh = 0; *gv = -1.0f; break;
            case Expression::Embarrassed:
                *gh = 0; *gv = 0.7f; break;
            default:
                *gh = gaze_h_; *gv = gaze_v_;
        }
    }

    void OnTick() {
        tick_count_++;
        uint32_t now = lv_tick_get();

        UpdateBreathParams();
        if (breath_paused_) {
            breath_ = 0;
        } else {
            breath_ = sinf((tick_count_ % breath_period_steps_) * 2.0f * 3.14159265f / breath_period_steps_);
        }

        if (BlinkAllowed()) {
            if (now >= next_blink_ms_) {
                uint32_t mult = SlowBlink() ? 2 : 1;
                if (eye_closed_) {
                    eye_open_ratio_ = 1.0f;
                    next_blink_ms_ = now + mult * (2500 + (rand() % 2000));
                    eye_closed_ = false;
                } else {
                    eye_open_ratio_ = 0.0f;
                    next_blink_ms_ = now + 150 + (rand() % 200);
                    eye_closed_ = true;
                }
            }
        } else {
            eye_open_ratio_ = 1.0f;
            eye_closed_ = false;
        }

        if (SaccadeEnabled() && (now - gaze_ext_ms_ > 1200) && now - last_saccade_ms_ > 1500) {
            gaze_h_ = (rand() % 21 - 10) / 10.0f;
            gaze_v_ = (rand() % 21 - 10) / 10.0f;
            last_saccade_ms_ = now;
        }

        bool speaking = (speaking_until_ms_ != 0 && now < speaking_until_ms_);
        if (!speaking && speaking_until_ms_ != 0) speaking_until_ms_ = 0;
        if (speaking) {
            mouth_open_ = 0.2f + (rand() % 80) / 100.0f;
        } else {
            mouth_open_ = 0.0f;
        }

        Draw();
    }

    void Draw() {
        if (!canvas_) return;
        const lv_color_t bg = lv_color_make(0x00, 0x00, 0x00);

        lv_canvas_fill_bg(canvas_, bg, LV_OPA_COVER);
        lv_layer_t layer;
        lv_canvas_init_layer(canvas_, &layer);

        int bob = (int)(breath_ * 3.0f);                            // 呼吸:轻微上下浮
        if (mouth_open_ > 0.05f) bob -= (int)(mouth_open_ * 3.0f);   // 说话时轻轻上跳(无嘴,靠身体动)

        // 睡眠/找不到人/正在跟踪时,这一帧临时借用对应表情,只改这一帧的显示,
        // 不动 expression_/overlay_/gaze_ 本身——状态结束就自动恢复 LLM 设的正常
        // 表情。优先级:睡眠 > 着急找人 > 注意到你(三者互斥,追踪在睡眠/搜索时
        // 不会同时发生)。睡眠里 activity!=0("忙自己的")不闭眼,睁眼往下看+图标。
        Expression saved_expr = expression_;
        Overlay saved_overlay = overlay_;
        float saved_gaze_h = gaze_h_, saved_gaze_v = gaze_v_;
        bool draw_book = false, draw_code = false, draw_papers = false;
        if (sleeping_) {
            if (sleep_activity_ == 0) {
                expression_ = Expression::Sleepy;
                overlay_.zzz = true;
            } else {
                expression_ = Expression::Neutral;   // 睁眼,不用默认呆呆直视
                gaze_h_ = 0.0f; gaze_v_ = 0.8f;       // 往下看,像在看手里的东西
                draw_book = (sleep_activity_ == 1);
                draw_code = (sleep_activity_ == 2);
                draw_papers = (sleep_activity_ == 3);
            }
        } else if (searching_) {
            expression_ = Expression::Confused;
            overlay_.question_mark = true;
        } else if (attentive_) {
            expression_ = Expression::Surprised;   // 睁大眼睛="注意到你在动了"
        }

        DrawBody(&layer, bob);
        DrawFace(&layer, bob);
        DrawExtras(&layer, bob);
        if (draw_book) DrawBookIcon(&layer, 238, 60 + bob);
        if (draw_code) DrawCodeIcon(&layer, 236, 58 + bob);
        if (draw_papers) DrawPapersIcon(&layer, 238, 58 + bob);

        expression_ = saved_expr;
        overlay_ = saved_overlay;
        gaze_h_ = saved_gaze_h; gaze_v_ = saved_gaze_v;

        lv_canvas_finish_layer(canvas_, &layer);
    }

    void FillRect(lv_layer_t* layer, int x, int y, int w, int h, lv_color_t c) {
        if (w <= 0 || h <= 0) return;
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.bg_color = c;
        d.bg_opa = LV_OPA_COVER;
        d.radius = 0;
        d.border_width = 0;
        lv_area_t a = {x, y, x + w - 1, y + h - 1};
        lv_draw_rect(layer, &d, &a);
    }

    void FillCircle(lv_layer_t* layer, int cx, int cy, int r, lv_color_t c) {
        if (r <= 0) return;
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.bg_color = c;
        d.bg_opa = LV_OPA_COVER;
        d.radius = LV_RADIUS_CIRCLE;
        d.border_width = 0;
        lv_area_t a = {cx - r, cy - r, cx + r - 1, cy + r - 1};
        lv_draw_rect(layer, &d, &a);
    }

    void FillTriangle(lv_layer_t* layer, int x0, int y0, int x1, int y1, int x2, int y2, lv_color_t c) {
        lv_draw_triangle_dsc_t d;
        lv_draw_triangle_dsc_init(&d);
        d.p[0].x = (float)x0; d.p[0].y = (float)y0;
        d.p[1].x = (float)x1; d.p[1].y = (float)y1;
        d.p[2].x = (float)x2; d.p[2].y = (float)y2;
        d.color = c;
        d.opa = LV_OPA_COVER;
        lv_draw_triangle(layer, &d);
    }

    void FillRoundRect(lv_layer_t* layer, int x, int y, int w, int h, int radius, lv_color_t c) {
        if (w <= 0 || h <= 0) return;
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.bg_color = c;
        d.bg_opa = LV_OPA_COVER;
        d.radius = radius;
        d.border_width = 0;
        lv_area_t a = {x, y, x + w - 1, y + h - 1};
        lv_draw_rect(layer, &d, &a);
    }

    void DrawArc(lv_layer_t* layer, int cx, int cy, int r, int start_deg, int end_deg, int width, lv_color_t c, bool rounded = false) {
        lv_draw_arc_dsc_t d;
        lv_draw_arc_dsc_init(&d);
        d.color = c;
        d.opa = LV_OPA_COVER;
        d.width = width;
        d.center.x = cx;
        d.center.y = cy;
        d.radius = r;
        d.start_angle = start_deg;
        d.end_angle = end_deg;
        d.rounded = rounded ? 1 : 0;
        lv_draw_arc(layer, &d);
    }

    void DrawLine(lv_layer_t* layer, int x1, int y1, int x2, int y2, int width, bool round, lv_color_t c) {
        lv_draw_line_dsc_t d;
        lv_draw_line_dsc_init(&d);
        d.color = c;
        d.opa = LV_OPA_COVER;
        d.width = width;
        d.round_start = round ? 1 : 0;
        d.round_end = round ? 1 : 0;
        d.p1.x = (float)x1; d.p1.y = (float)y1;
        d.p2.x = (float)x2; d.p2.y = (float)y2;
        lv_draw_line(layer, &d);
    }

    // ── Clawd(Claude Code 吉祥物)身体:珊瑚方块躯干 + 小手臂 + 三条腿 ──
    void DrawBody(lv_layer_t* layer, int bob) {
        // 珊瑚色偏深、加饱和,补偿机器人 LCD 的"泛白"(电脑上偏艳,实机才正)
        const lv_color_t BODY = lv_color_make(0xC8, 0x64, 0x3C);
        int o = bob;
        FillRect(layer, 96, 66 + o, 128, 78, BODY);   // 主体
        FillRect(layer, 80, 98 + o, 16, 18, BODY);    // 左臂
        FillRect(layer, 224, 98 + o, 16, 18, BODY);   // 右臂
        FillRect(layer, 103, 144 + o, 18, 22, BODY);  // 腿 ×4
        FillRect(layer, 135, 144 + o, 18, 22, BODY);
        FillRect(layer, 167, 144 + o, 18, 22, BODY);
        FillRect(layer, 199, 144 + o, 18, 22, BODY);
    }

    // 眼型 helper(全部矩形=像素风)
    void EyeSquare(lv_layer_t* layer, int cx, int ey, int sz, lv_color_t c) {
        FillRect(layer, cx - sz / 2, ey - sz / 2, sz, sz, c);
    }
    void EyeBar(lv_layer_t* layer, int cx, int ey, lv_color_t c) {   // 闭眼/睡:一条
        FillRect(layer, cx - 7, ey - 2, 14, 4, c);
    }
    void HappyEye(lv_layer_t* layer, int cx, int ey, lv_color_t c) { // ^
        FillRect(layer, cx - 8, ey + 1, 5, 4, c);
        FillRect(layer, cx - 3, ey - 2, 6, 4, c);
        FillRect(layer, cx + 3, ey + 1, 5, 4, c);
    }
    void HeartEye(lv_layer_t* layer, int cx, int ey, lv_color_t c) {
        FillRect(layer, cx - 6, ey - 4, 5, 5, c);
        FillRect(layer, cx + 1, ey - 4, 5, 5, c);
        FillRect(layer, cx - 6, ey + 1, 12, 4, c);
        FillRect(layer, cx - 2, ey + 5, 6, 3, c);
    }
    void Brows(lv_layer_t* layer, int lx, int rx, int ey, bool angry, lv_color_t c) {
        if (angry) {
            FillRect(layer, lx - 11, ey - 15, 8, 5, c); FillRect(layer, lx - 3, ey - 11, 8, 5, c);  // \.
            FillRect(layer, rx + 3,  ey - 15, 8, 5, c); FillRect(layer, rx - 5, ey - 11, 8, 5, c);  // /
        } else {
            FillRect(layer, lx - 11, ey - 11, 8, 5, c); FillRect(layer, lx - 3, ey - 15, 8, 5, c);  // /
            FillRect(layer, rx + 3,  ey - 11, 8, 5, c); FillRect(layer, rx - 5, ey - 15, 8, 5, c);  // \.
        }
    }

    // ── Clawd 的脸:只有眼睛(无嘴)。眨眼/表情/视线都在这里 ──
    void DrawFace(lv_layer_t* layer, int bob) {
        const lv_color_t EYE   = lv_color_make(0x1A, 0x14, 0x12);
        const lv_color_t DARKB = lv_color_make(0x8A, 0x4A, 0x32);
        const lv_color_t HEART = lv_color_make(0xE0, 0x55, 0x5A);

        const Expression e = expression_;
        const int LX = 126, RX = 194;

        float gh, gv; GetGazeOverride(&gh, &gv);
        const int gx = (int)(gh * 8.0f);   // 追踪视线幅度加大,让"看向你"更明显
        const int gy = (int)(gv * 6.0f);
        const int ey = 100 + bob + gy;

        const bool happyEyes = (e == Expression::Happy || e == Expression::Laughing ||
                                e == Expression::Delicious || e == Expression::Kissy);
        const bool heart     = (e == Expression::Loving);
        const bool cool      = (e == Expression::Cool);
        const bool sleepy    = (e == Expression::Sleepy || e == Expression::Relaxed);
        const bool wink      = (e == Expression::Winking || e == Expression::Silly);
        const bool surprised = (e == Expression::Surprised || e == Expression::Shocked ||
                                e == Expression::Funny);
        const bool sad       = (e == Expression::Sad || e == Expression::Crying);
        const bool angry     = (e == Expression::Angry);
        const bool confused  = (e == Expression::Confused);
        const bool blinkClosed = (eye_open_ratio_ < 0.5f) && BlinkAllowed();

        if (cool) return;                                 // 墨镜由 DrawExtras 画,不画眼
        if (heart) { HeartEye(layer, LX + gx, ey, HEART); HeartEye(layer, RX + gx, ey, HEART); return; }
        if (sleepy) { EyeBar(layer, LX, ey, EYE); EyeBar(layer, RX, ey, EYE); return; }
        if (happyEyes && !blinkClosed) { HappyEye(layer, LX, ey, EYE); HappyEye(layer, RX, ey, EYE); return; }
        if (wink) { EyeSquare(layer, LX + gx, ey, 14, EYE); EyeBar(layer, RX, ey, EYE); return; }
        if (blinkClosed) { EyeBar(layer, LX, ey, EYE); EyeBar(layer, RX, ey, EYE); return; }

        if (surprised) {
            int sz = (e == Expression::Shocked) ? 22 : 20;
            EyeSquare(layer, LX + gx, ey, sz, EYE); EyeSquare(layer, RX + gx, ey, sz, EYE);
            return;
        }
        if (confused) {
            EyeSquare(layer, LX + gx, ey, 12, EYE); EyeSquare(layer, RX + gx, ey, 16, EYE);
            return;
        }
        if (sad) {
            EyeSquare(layer, LX + gx, ey + 3, 13, EYE); EyeSquare(layer, RX + gx, ey + 3, 13, EYE);
            Brows(layer, LX, RX, ey, false, DARKB);
            return;
        }
        if (angry) {
            EyeSquare(layer, LX + gx, ey + 2, 13, EYE); EyeSquare(layer, RX + gx, ey + 2, 13, EYE);
            Brows(layer, LX, RX, ey, true, DARKB);
            return;
        }
        // 默认:Neutral / Confident / Embarrassed / Thinking ...
        EyeSquare(layer, LX + gx, ey, 14, EYE);
        EyeSquare(layer, RX + gx, ey, 14, EYE);
    }

    // ── 睡眠待机的"忙自己的"小图标(看书/写代码/看论文),角落小图标示意——
    // 像素方块风,和其它装饰(闪光/zzz)同一路子。
    void DrawBookIcon(lv_layer_t* layer, int x, int y) {
        const lv_color_t PAPER = lv_color_make(0xF0, 0xEC, 0xE0);
        const lv_color_t SPINE = lv_color_make(0xB0, 0x70, 0x45);
        const lv_color_t LINEC = lv_color_make(0x8A, 0x8A, 0x8A);
        FillRect(layer, x, y, 15, 20, PAPER);
        FillRect(layer, x + 15, y, 15, 20, PAPER);
        FillRect(layer, x + 14, y, 2, 20, SPINE);
        for (int i = 0; i < 3; i++) {
            FillRect(layer, x + 3, y + 4 + i * 5, 9, 1, LINEC);
            FillRect(layer, x + 18, y + 4 + i * 5, 9, 1, LINEC);
        }
    }
    void DrawCodeIcon(lv_layer_t* layer, int x, int y) {
        const lv_color_t SCREEN = lv_color_make(0x20, 0x20, 0x28);
        const lv_color_t CODEC  = lv_color_make(0x6F, 0xE0, 0x9A);
        const lv_color_t BASE   = lv_color_make(0x50, 0x50, 0x58);
        FillRect(layer, x, y, 34, 22, SCREEN);
        FillRect(layer, x + 3, y + 4, 14, 3, CODEC);
        FillRect(layer, x + 3, y + 10, 20, 3, CODEC);
        FillRect(layer, x + 3, y + 16, 10, 3, CODEC);
        FillRect(layer, x, y + 22, 34, 3, BASE);
    }
    void DrawPapersIcon(lv_layer_t* layer, int x, int y) {
        const lv_color_t P1 = lv_color_make(0xD8, 0xD4, 0xC8);
        const lv_color_t P2 = lv_color_make(0xE4, 0xE0, 0xD4);
        const lv_color_t P3 = lv_color_make(0xF0, 0xEC, 0xE0);
        const lv_color_t LINEC = lv_color_make(0x8A, 0x8A, 0x8A);
        FillRect(layer, x + 4, y + 6, 22, 16, P1);
        FillRect(layer, x + 2, y + 3, 22, 16, P2);
        FillRect(layer, x, y, 22, 16, P3);
        for (int i = 0; i < 3; i++) {
            FillRect(layer, x + 3, y + 3 + i * 4, 14, 1, LINEC);
        }
    }

    // ── 附加装饰:腮红/眼泪/墨镜/飘心/闪光/zzz/问号 等(无嘴,全走眼与装饰)──
    void DrawExtras(lv_layer_t* layer, int bob) {
        const lv_color_t WHITE = lv_color_make(0xFF, 0xFF, 0xFF);
        const lv_color_t SPARK = lv_color_make(0xF5, 0xD9, 0xA8);
        const lv_color_t TEAR  = lv_color_make(0x6F, 0xB8, 0xE0);
        const lv_color_t HEART = lv_color_make(0xE0, 0x55, 0x5A);
        const lv_color_t CHEEK = lv_color_make(0xE0, 0x8A, 0x72);
        const lv_color_t GLASS = lv_color_make(0x1A, 0x16, 0x1C);
        const Expression e = expression_;
        const int LX = 126, RX = 194;
        const int eyB = 100 + bob;

        auto spark = [&](int x, int y, int s, lv_color_t c) {
            FillRect(layer, x - 2, y - s, 4, s * 2, c);
            FillRect(layer, x - s, y - 2, s * 2, 4, c);
        };
        auto heart = [&](int hx, int hy) {
            FillRect(layer, hx - 8, hy - 4, 6, 6, HEART); FillRect(layer, hx + 2, hy - 4, 6, 6, HEART);
            FillRect(layer, hx - 8, hy + 2, 16, 4, HEART); FillRect(layer, hx - 4, hy + 6, 8, 3, HEART);
        };

        // 腮红
        if (e == Expression::Happy || e == Expression::Loving || e == Expression::Kissy ||
            e == Expression::Delicious || e == Expression::Embarrassed || overlay_.cheek_blush) {
            FillRect(layer, LX - 6, 118 + bob, 12, 7, CHEEK);
            FillRect(layer, RX - 6, 118 + bob, 12, 7, CHEEK);
        }
        // 眼泪(哭)
        if (overlay_.tear) {
            FillRect(layer, LX - 12, eyB + 11, 5, 7, TEAR);
            FillRect(layer, LX - 12, eyB + 19, 5, 5, TEAR);
        }
        // 墨镜(cool)
        if (overlay_.cool_glasses) {
            FillRect(layer, 110, 90 + bob, 100, 14, GLASS);
            FillRect(layer, 154, 95 + bob, 12, 4, GLASS);
        }
        // 飘心(亲亲 / 爱)
        if (overlay_.kiss_heart) heart(250, 62 + bob);
        if (e == Expression::Loving) heart(254, 60 + bob);
        // 思考点点 + 闪光
        if (overlay_.think_bubble) {
            FillRect(layer, 244, 92 + bob, 5, 5, WHITE);
            FillRect(layer, 256, 80 + bob, 6, 6, WHITE);
            spark(272, 64 + bob, 7, SPARK);
        }
        // 惊喜闪光
        if (overlay_.star_burst) spark(252, 58 + bob, 8, SPARK);
        // 感叹号(震惊)
        if (overlay_.excl_mark) {
            FillRect(layer, 266, 48 + bob, 6, 14, WHITE);
            FillRect(layer, 266, 64 + bob, 6, 5, WHITE);
        }
        // 问号(疑惑)
        if (overlay_.question_mark) {
            FillRect(layer, 258, 58 + bob, 5, 5, WHITE);
            FillRect(layer, 256, 66 + bob, 4, 10, WHITE);
            FillRect(layer, 258, 80 + bob, 5, 5, WHITE);
        }
        // zzz(睡)
        if (overlay_.zzz) {
            auto Z = [&](int cx, int cy, int sz, int w) {
                int h = sz / 2;
                FillRect(layer, cx - h, cy - h, sz, w, WHITE);
                FillRect(layer, cx - h, cy + h - w, sz, w, WHITE);
                DrawLine(layer, cx + h, cy - h, cx - h, cy + h, w, false, WHITE);
            };
            Z(244, 86 + bob, 7, 2); Z(258, 70 + bob, 9, 3); Z(276, 52 + bob, 12, 3);
        }
        // 常驻小闪光(中性/自信/眨眼)
        if (e == Expression::Neutral || e == Expression::Confident || e == Expression::Winking)
            spark(250, 60 + bob, 6, SPARK);
    }

    lv_obj_t* canvas_ = nullptr;
    uint8_t* buf_ = nullptr;
    lv_timer_t* timer_ = nullptr;
    int w_ = 320, h_ = 240;

    Expression expression_ = Expression::Neutral;
    Overlay overlay_;

    uint32_t tick_count_ = 0;
    uint32_t next_blink_ms_ = 0;
    uint32_t last_saccade_ms_ = 0;
    uint32_t speaking_until_ms_ = 0;
    bool eye_closed_ = false;
    float eye_open_ratio_ = 1.0f;
    float mouth_open_ = 0.0f;
    float breath_ = 0.0f;
    float gaze_h_ = 0.0f;
    float gaze_v_ = 0.0f;
    uint32_t gaze_ext_ms_ = 0;   // 上次外部(摄像头追踪)设视线的时间,期间暂停随机扫视
    bool searching_ = false;    // 找不到人,正在"着急找"(临时借用表情,见 Draw())
    bool sleeping_ = false;     // 明确进入睡眠(self.sleep.rest,临时借用表情,见 Draw())
    int sleep_activity_ = 0;    // 0=睡觉 1=看书 2=写代码 3=看论文(见 Draw())
    bool attentive_ = false;    // 正在跟踪真实动静(临时借用表情,见 Draw())

    float breath_amp_ = 3.0f;
    uint32_t breath_period_steps_ = 100;
    bool breath_paused_ = false;
};

static Expression MapEmotion(const char* e) {
    if (!e) return Expression::Neutral;
    if (!strcmp(e, "neutral"))     return Expression::Neutral;
    if (!strcmp(e, "happy"))       return Expression::Happy;
    if (!strcmp(e, "laughing"))    return Expression::Laughing;
    if (!strcmp(e, "funny"))       return Expression::Funny;
    if (!strcmp(e, "sad"))         return Expression::Sad;
    if (!strcmp(e, "crying"))      return Expression::Crying;
    if (!strcmp(e, "angry"))       return Expression::Angry;
    if (!strcmp(e, "loving"))      return Expression::Loving;
    if (!strcmp(e, "embarrassed")) return Expression::Embarrassed;
    if (!strcmp(e, "surprised"))   return Expression::Surprised;
    if (!strcmp(e, "shocked"))     return Expression::Shocked;
    if (!strcmp(e, "thinking"))    return Expression::Thinking;
    if (!strcmp(e, "winking"))     return Expression::Winking;
    if (!strcmp(e, "cool"))        return Expression::Cool;
    if (!strcmp(e, "relaxed"))     return Expression::Relaxed;
    if (!strcmp(e, "delicious"))   return Expression::Delicious;
    if (!strcmp(e, "kissy"))       return Expression::Kissy;
    if (!strcmp(e, "confident"))   return Expression::Confident;
    if (!strcmp(e, "sleepy"))      return Expression::Sleepy;
    if (!strcmp(e, "silly"))       return Expression::Silly;
    if (!strcmp(e, "confused"))    return Expression::Confused;
    return Expression::Neutral;
}

static Overlay OverlayFor(const char* e) {
    Overlay o;
    if (!e) return o;
    if (!strcmp(e, "crying"))      o.tear = true;
    if (!strcmp(e, "loving"))      o.heart_eyes = true;
    if (!strcmp(e, "kissy"))       o.kiss_heart = true;
    if (!strcmp(e, "embarrassed")) o.cheek_blush = true;
    if (!strcmp(e, "cool"))        o.cool_glasses = true;
    if (!strcmp(e, "shocked"))     o.excl_mark = true;
    if (!strcmp(e, "thinking"))    o.think_bubble = true;
    if (!strcmp(e, "surprised"))   o.star_burst = true;
    // silly: no overlay
    if (!strcmp(e, "delicious"))   o.drool = true;
    if (!strcmp(e, "confused"))    o.question_mark = true;
    if (!strcmp(e, "sleepy"))      o.zzz = true;
    return o;
}

}  // namespace shizhou_avatar

class M5StackAvatarDisplay : public SpiLcdDisplay {
public:
    M5StackAvatarDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                          int width, int height, int offset_x, int offset_y,
                          bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy),
          canvas_w_(width), canvas_h_(height) {
        esp_timer_create_args_t args = {};
        args.callback = &M5StackAvatarDisplay::InitTimerCallback;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "avatar_init";
        args.skip_unhandled_events = true;
        esp_timer_create(&args, &avatar_init_timer_);
        esp_timer_start_periodic(avatar_init_timer_, 500000);

        esp_timer_create_args_t idle_args = {};
        idle_args.callback = &M5StackAvatarDisplay::IdleTimerCallback;
        idle_args.arg = this;
        idle_args.dispatch_method = ESP_TIMER_TASK;
        idle_args.name = "avatar_idle";
        idle_args.skip_unhandled_events = true;
        esp_timer_create(&idle_args, &idle_timer_);
    }

    void SetFaceTracker(FaceTracker* ft) { face_tracker_ = ft; }

    // 摄像头追踪到人时,把方向喂给头像的眼睛(没有舵机/脖子也能"看向你")
    void SetGaze(float gh, float gv) {
        DisplayLockGuard lock(this);
        if (avatar_.IsReady()) avatar_.SetGazeTarget(gh, gv);
    }
    // 摄像头找不到人、开始"着急找"时(true)/找到或暂停时(false)
    void SetSearching(bool s) {
        DisplayLockGuard lock(this);
        if (avatar_.IsReady()) avatar_.SetSearching(s);
    }
    // 明确进入(true)/退出(false)睡眠——self.sleep.* 工具或新一轮对话开始时调用
    void SetSleeping(bool s, int activity = 0) {
        DisplayLockGuard lock(this);
        if (avatar_.IsReady()) avatar_.SetSleeping(s, activity);
    }
    // 正在跟踪真实动静(true)/停止(false)——让"她在动"总有个看得出来的表情理由
    void SetAttentive(bool a) {
        DisplayLockGuard lock(this);
        if (avatar_.IsReady()) avatar_.SetAttentive(a);
    }
    void SetServo(StackChanServo* s) { servo_ = s; }
    void SetLedUpdater(std::function<void(const char*)> fn) { led_updater_ = std::move(fn); }

    void OnPetted() {
        if (!avatar_.IsReady()) return;
        DisplayLockGuard lock(this);
        avatar_.SetExpression(shizhou_avatar::Expression::Loving);
        shizhou_avatar::Overlay o;
        o.heart_eyes = true;
        o.cheek_blush = true;
        avatar_.SetOverlay(o);
        SetActiveLocked(true);
        BumpIdleTimerLocked();
        if (servo_) servo_->Tilt();
    }

    void SetEmotion(const char* emotion) override {
        SpiLcdDisplay::SetEmotion(emotion);
        DisplayLockGuard lock(this);
        HideEmojiBoxLocked();
        if (!avatar_.IsReady()) return;
        avatar_.SetExpression(shizhou_avatar::MapEmotion(emotion));
        avatar_.SetOverlay(shizhou_avatar::OverlayFor(emotion));
        if (emotion && strcmp(emotion, "sleepy") == 0) {
            SetActiveLocked(false);
            if (face_tracker_) face_tracker_->Pause(false);
            if (servo_) servo_->PauseScan();
        }
        // 非 sleepy 不再无条件 Resume face_tracker——它现在跟设备状态走，由 SetStatus 控制
        // 注意：head_locked 时 Nod/Shake 仍然会播放，但因为 Nod/Shake 用
        // GetCurrentYaw/Pitch 作 base，会在当前位置就地点头/摇头，不会把头甩回 (0, 30)。
        if (servo_ && emotion && !servo_->IsAnimating()) {
            if (!strcmp(emotion, "happy") || !strcmp(emotion, "loving") ||
                !strcmp(emotion, "laughing") || !strcmp(emotion, "confident") ||
                !strcmp(emotion, "winking") || !strcmp(emotion, "delicious")) {
                servo_->Nod();
            } else if (!strcmp(emotion, "sad") || !strcmp(emotion, "confused") ||
                       !strcmp(emotion, "angry") || !strcmp(emotion, "shocked")) {
                servo_->Shake();
            }
        }
        // 情绪灯联动
        if (led_updater_) led_updater_(emotion);
    }

    void SetPreviewImage(std::unique_ptr<LvglImage> image) override {
        SpiLcdDisplay::SetPreviewImage(std::move(image));
        DisplayLockGuard lock(this);
        HideEmojiBoxLocked();
    }

    void SetChatMessage(const char* role, const char* content) override {
        SpiLcdDisplay::SetChatMessage(role, content);
        DisplayLockGuard lock(this);
        if (!avatar_.IsReady()) return;
        bool meaningful = role && content && content[0] != '\0'
            && (strcmp(role, "user") == 0 || strcmp(role, "assistant") == 0);
        if (meaningful) {
            SetActiveLocked(true);
            BumpIdleTimerLocked();
        }
        if (role && content && content[0] != '\0' && strcmp(role, "assistant") == 0) {
            size_t n = strlen(content);
            uint32_t ms = (uint32_t)(n * 120);
            if (ms < 800) ms = 800;
            if (ms > 15000) ms = 15000;
            avatar_.StartSpeaking(ms);
        } else if (role && (strcmp(role, "user") == 0 || strcmp(role, "system") == 0)) {
            avatar_.StopSpeaking();
        }
    }

    void SetStatus(const char* status) override {
        SpiLcdDisplay::SetStatus(status);
        if (!status || !avatar_.IsReady()) return;
        DisplayLockGuard lock(this);
        auto state = Application::GetInstance().GetDeviceState();
        // 追踪常驻开启(不再随 Idle 自动暂停)——只有明确"睡眠"才停,见 self.sleep.*。
        // 开始新一轮对话 = 确定"醒着":顺带解除手动锁(避免用户曾用 head.move
        // 转头后忘了 head.center 导致追踪永久失效)、清掉睡眠视觉、恢复追踪。
        if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
            if (face_tracker_) {
                face_tracker_->SetManualLock(false);
                face_tracker_->Resume();
            }
            SetSleeping(false);
        }
        bool is_active = (strstr(status, "聆听")
                       || strstr(status, "说话")
                       || strstr(status, "思考")
                       || strstr(status, "连接")
                       || strstr(status, "Listening")
                       || strstr(status, "Speaking")
                       || strstr(status, "Thinking")
                       || strstr(status, "Connecting"));
        // "聆听中/思考中/说话中"这类日常对话状态不再显示在头顶(主人反馈不需要)。
        // 其它状态(WiFi 设置提示、报错、激活码等)照常显示,这些是真正有用的信息。
        if (status_bar_) {
            if (is_active) {
                lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_remove_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
            }
        }
        if (is_active) {
            SetActiveLocked(true);
            BumpIdleTimerLocked();
        }
    }

private:
    shizhou_avatar::LvglAvatar avatar_;
    FaceTracker* face_tracker_ = nullptr;
    StackChanServo* servo_ = nullptr;
    std::function<void(const char*)> led_updater_;
    esp_timer_handle_t avatar_init_timer_ = nullptr;
    esp_timer_handle_t idle_timer_ = nullptr;
    bool active_mode_ = false;
    int canvas_w_ = 320;
    int canvas_h_ = 240;
    static constexpr uint64_t IDLE_TIMEOUT_US = 8 * 1000 * 1000;

    void HideEmojiBoxLocked() {
        if (avatar_.IsReady() && emoji_box_) {
            lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void SetActiveLocked(bool active) {
        if (active == active_mode_) return;
        active_mode_ = active;
        if (active) {
            if (top_bar_)    lv_obj_remove_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
            if (status_bar_) lv_obj_remove_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
            if (face_tracker_) face_tracker_->Resume();
        } else {
            if (top_bar_)    lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
            if (status_bar_) lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void BumpIdleTimerLocked() {
        if (!idle_timer_) return;
        esp_timer_stop(idle_timer_);
        esp_timer_start_once(idle_timer_, IDLE_TIMEOUT_US);
    }

    static void IdleTimerCallback(void* arg) {
        auto self = static_cast<M5StackAvatarDisplay*>(arg);
        DisplayLockGuard lock(self);
        self->SetActiveLocked(false);
        // face_tracker 由 SetStatus 跟设备状态联动控制，这里只管 UI 顶栏隐藏
    }

    static void InitTimerCallback(void* arg) {
        auto self = static_cast<M5StackAvatarDisplay*>(arg);
        self->TryInitAvatar();
    }

    void TryInitAvatar() {
        DisplayLockGuard lock(this);
        if (avatar_.IsReady()) return;
        lv_obj_t* screen = lv_screen_active();
        if (screen == nullptr) return;
        if (container_ == nullptr) return;

        bool ok = avatar_.Init(screen, canvas_w_, canvas_h_);
        if (!ok) return;

        lv_obj_set_style_bg_opa(container_, LV_OPA_TRANSP, 0);
        if (content_) lv_obj_set_style_bg_opa(content_, LV_OPA_TRANSP, 0);
        if (emoji_box_) {
            lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        }
        if (top_bar_)    lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
        if (status_bar_) lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);

        if (avatar_init_timer_) {
            esp_timer_stop(avatar_init_timer_);
            esp_timer_delete(avatar_init_timer_);
            avatar_init_timer_ = nullptr;
        }
    }
};

class Pmic : public Axp2101 {
public:
    // Power Init
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        uint8_t data = ReadReg(0x90);
        data |= 0b10110100;
        WriteReg(0x90, data);
        WriteReg(0x99, (0b11110 - 5));
        WriteReg(0x97, (0b11110 - 2));
        WriteReg(0x69, 0b00110101);
        WriteReg(0x30, 0b111111);
        WriteReg(0x90, 0xBF);
        WriteReg(0x94, 33 - 5);
        WriteReg(0x95, 33 - 5);
    }

    void SetBrightness(uint8_t brightness) {
        brightness = ((brightness + 641) >> 5);
        WriteReg(0x99, brightness);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(Pmic *pmic) : pmic_(pmic) {}

    void SetBrightnessImpl(uint8_t brightness) override {
        pmic_->SetBrightness(target_brightness_);
        brightness_ = target_brightness_;
    }

private:
    Pmic *pmic_;
};

class Aw9523 : public I2cDevice {
public:
    // Exanpd IO Init
    Aw9523(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x02, 0b00000111);  // P0
        WriteReg(0x03, 0b10001111);  // P1
        WriteReg(0x04, 0b00011000);  // CONFIG_P0
        WriteReg(0x05, 0b00001100);  // CONFIG_P1
        WriteReg(0x11, 0b00010000);  // GCR P0 port is Push-Pull mode.
        WriteReg(0x12, 0b11111111);  // LEDMODE_P0
        WriteReg(0x13, 0b11111111);  // LEDMODE_P1
    }

    void ResetAw88298() {
        ESP_LOGI(TAG, "Reset AW88298");
        WriteReg(0x02, 0b00000011);
        vTaskDelay(pdMS_TO_TICKS(10));
        WriteReg(0x02, 0b00000111);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void ResetIli9342() {
        ESP_LOGI(TAG, "Reset IlI9342");
        WriteReg(0x03, 0b10000001);
        vTaskDelay(pdMS_TO_TICKS(20));
        WriteReg(0x03, 0b10000011);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
};

class Ft6336 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };
    
    Ft6336(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        uint8_t chip_id = ReadReg(0xA3);
        ESP_LOGI(TAG, "Get chip ID: 0x%02X", chip_id);
        read_buffer_ = new uint8_t[6];
    }

    ~Ft6336() {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint() {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    inline const TouchPoint_t& GetTouchPoint() {
        return tp_;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;
};

// ── 定时活动分类:把云端视觉模型的回答(可能带各种废话/标点)防御性
// 匹配到几个固定短标签上,保证发给 SendUserText 的内容长度可控、稳定可解析。
// "工作听歌"类要排在"工作"前面,同时出现"工作"+"音乐/听歌"时优先合并判定。
static std::string ClassifyActivityLabel(const std::string& raw) {
    struct { const char* needle; const char* label; } table[] = {
        {"开会", "开会"}, {"会议", "开会"},
        {"游戏", "游戏"}, {"打游戏", "游戏"},
        {"视频", "视频"}, {"看剧", "视频"}, {"电影", "视频"},
        {"听歌", "工作听歌"}, {"音乐", "工作听歌"}, {"耳机", "工作听歌"},
        {"工作", "工作"}, {"打字", "工作"}, {"电脑", "工作"}, {"办公", "工作"},
    };
    for (auto& t : table) {
        if (raw.find(t.needle) != std::string::npos) return t.label;
    }
    return "其他";
}

class M5StackCoreS3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_;
    Aw9523* aw9523_;
    Ft6336* ft6336_;
    LcdDisplay* display_;
    EspVideo* camera_;
    StackChanServo servo_;
    FaceTracker face_tracker_;
    esp_timer_handle_t touchpad_timer_;
    esp_timer_handle_t batt_timer_ = nullptr;
    PowerSaveTimer* power_save_timer_;
    bool py32_found_ = false;
    // ---- PY32 持久 I2C 设备句柄（控 LED + 其他扩展）----
    i2c_master_dev_handle_t py32_dev_ = nullptr;
    bool led_manual_ = false;
    // ---- BMI270 (IMU) ----
    i2c_master_dev_handle_t bmi_i2c_dev_ = nullptr;
    struct bmi2_dev bmi_dev_storage_ = {};
    bmi270_handle_t bmi_handle_ = nullptr;
    TaskHandle_t motion_task_ = nullptr;
    int64_t last_motion_trigger_us_ = 0;
    // ---- SI12T 3 区触摸 ----
    i2c_master_bus_handle_t si12t_bus_ = nullptr;
    i2c_master_dev_handle_t si12t_dev_ = nullptr;
    TaskHandle_t si12t_task_ = nullptr;
    uint8_t si12t_last_state_ = 0;
    bool servo_ok_ = false;
    bool low_batt_warned_ = false;
    AlarmManager alarm_mgr_;
    esp_timer_handle_t alarm_timer_ = nullptr;
    esp_timer_handle_t activity_timer_ = nullptr;

    void InitializeBmi270() {
        // BMI270 实际在 0x69（不是 SDK 默认的 0x68），自己用 IDF i2c API + 底层 bmi270_init 绕过硬编码
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x69,
            .scl_speed_hz = 400000,
        };
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &bmi_i2c_dev_) != ESP_OK) {
            ESP_LOGW(TAG, "BMI270: add i2c device failed");
            return;
        }

        // 自己构造 bmi2_dev：用我们的 i2c_master_dev_handle 作为 intf_ptr，read/write 走 0x69
        bmi_dev_storage_.intf = BMI2_I2C_INTF;
        bmi_dev_storage_.intf_ptr = bmi_i2c_dev_;
        bmi_dev_storage_.read = Bmi270I2cRead;
        bmi_dev_storage_.write = Bmi270I2cWrite;
        bmi_dev_storage_.delay_us = Bmi270DelayUs;
        bmi_dev_storage_.read_write_len = 256;
        bmi_dev_storage_.config_file_ptr = bmi270_config_file;
        bmi_dev_storage_.dummy_byte = 0;

        int8_t rslt = bmi270_init(&bmi_dev_storage_);
        if (rslt != BMI2_OK) {
            ESP_LOGW(TAG, "bmi270_init failed: %d", rslt);
            return;
        }
        ESP_LOGI(TAG, "BMI270 initialized (custom driver @ 0x69, chip_id=0x%02X)", bmi_dev_storage_.chip_id);

        bmi_handle_ = &bmi_dev_storage_;

        const uint8_t sens_list[] = {BMI2_ACCEL};
        rslt = bmi270_sensor_enable(sens_list, 1, bmi_handle_);
        if (rslt != BMI2_OK) {
            ESP_LOGW(TAG, "bmi270_sensor_enable failed: %d", rslt);
            bmi_handle_ = nullptr;
            return;
        }
        ESP_LOGI(TAG, "BMI270 accel enabled");

        xTaskCreatePinnedToCore(MotionTaskFunc, "motion", 4096, this, 1, &motion_task_, 1);
    }

    static void MotionTaskFunc(void* arg) {
        static_cast<M5StackCoreS3Board*>(arg)->MotionLoop();
        vTaskDelete(nullptr);
    }

    void MotionLoop() {
        // 算法：每 100ms 读一次 accel，magnitude 偏离 1g 算"动"
        //   - 1 秒内出现 ≥3 次"动"尖峰 → 摇晃
        //   - 连续 ≥5 个样本（500ms）持续偏离 → 抱起
        //   - 触发后 disarm，必须连续静止 1 秒（10 个样本）才 re-arm
        const float MOTION_THRESHOLD = 0.3f;       // delta 或 mag 偏离 1g 超过此值算"动"
        const int SHAKE_PEAKS_TO_TRIGGER = 2;      // 1 秒内 2 个尖峰算摇晃
        const int64_t SHAKE_WINDOW_US = 1000 * 1000;
        const int LIFT_SAMPLES_TO_TRIGGER = 5;
        const int STILL_SAMPLES_TO_REARM = 50;     // 5 秒静止才允许下次触发
        const int64_t GLOBAL_COOLDOWN_US = 5 * 60 * 1000 * 1000LL;  // 触发后 5 分钟全局冷却

        int lift_count = 0;
        int still_count = 0;
        bool armed = true;
        int64_t shake_peak_times[8] = {0};
        int shake_idx = 0;
        int log_counter = 0;
        float last_ax = 0, last_ay = 0, last_az = 0;
        bool last_valid = false;

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (!bmi_handle_) continue;

            struct bmi2_sens_data accel;
            int8_t rd = bmi2_get_sensor_data(&accel, bmi_handle_);
            if (rd != BMI2_OK) {
                if (++log_counter >= 10) {
                    log_counter = 0;
                    ESP_LOGW(TAG, "motion: bmi2_get_sensor_data err=%d", rd);
                }
                continue;
            }

            // BMI270 SDK 默认 ±8g 量程，int16 raw，1g ≈ 4096
            float ax = (float)accel.acc.x / 4096.0f;
            float ay = (float)accel.acc.y / 4096.0f;
            float az = (float)accel.acc.z / 4096.0f;
            float mag = sqrtf(ax * ax + ay * ay + az * az);

            // 用"轴变化率"检测动作（旋转/摇晃只改变各轴分量但不改变 mag）
            float delta = 0.0f;
            if (last_valid) {
                float dx = ax - last_ax;
                float dy = ay - last_ay;
                float dz = az - last_az;
                delta = sqrtf(dx * dx + dy * dy + dz * dz);
            }
            last_ax = ax; last_ay = ay; last_az = az; last_valid = true;

            // 动 = 轴变化大 OR magnitude 偏离 1g 大
            bool moving = (delta > MOTION_THRESHOLD) || (fabsf(mag - 1.0f) > MOTION_THRESHOLD);
            int64_t now = esp_timer_get_time();
            (void)log_counter;  // 诊断 log 已禁用

            if (!moving) {
                still_count++;
                if (still_count >= STILL_SAMPLES_TO_REARM) armed = true;
                lift_count = 0;
                for (int i = 0; i < 8; i++) shake_peak_times[i] = 0;
                continue;
            }

            still_count = 0;

            // 设备说话时忽略体感（舵机晃动会触发 IMU 误检测）
            if (Application::GetInstance().GetDeviceState() == kDeviceStateSpeaking) continue;

            if (!armed) continue;  // 已触发过，等静止 re-arm
            // 全局冷却：上次触发后 5 分钟内任何情况都不再触发
            if (last_motion_trigger_us_ != 0 && (now - last_motion_trigger_us_) < GLOBAL_COOLDOWN_US) continue;

            // 摇晃检测：1 秒内累计 ≥3 个尖峰
            shake_peak_times[shake_idx % 8] = now;
            shake_idx++;
            int peak_count = 0;
            for (int i = 0; i < 8; i++) {
                if (shake_peak_times[i] > 0 &&
                    (now - shake_peak_times[i]) < SHAKE_WINDOW_US) {
                    peak_count++;
                }
            }
            if (peak_count >= SHAKE_PEAKS_TO_TRIGGER) {
                armed = false;
                lift_count = 0;
                last_motion_trigger_us_ = now;
                for (int i = 0; i < 8; i++) shake_peak_times[i] = 0;
                const auto& m = PickRandom(ShakePool());
                if (m.display) {
                    if (auto* disp = GetDisplay()) disp->SetChatMessage("user", m.display);
                    SendUserMessage(m.tag);
                }
                continue;
            }

            // 抱起检测：连续 ≥5 个样本持续偏离
            lift_count++;
            if (lift_count >= LIFT_SAMPLES_TO_TRIGGER) {
                armed = false;
                lift_count = 0;
                last_motion_trigger_us_ = now;
                const auto& m = PickRandom(LiftPool());
                if (m.display) {
                    if (auto* disp = GetDisplay()) disp->SetChatMessage("user", m.display);
                    SendUserMessage(m.tag);
                }
            }
        }
    }

    // (Morning greeting + weather timer task removed; see git history.)

    // ---- PY32 IO Expander 控 WS2812 RGB LED ×12 ----
    // 协议（来自 M5Stack/StackChan-BSP）:
    //   REG_LED_CFG  (0x24): 低6位=LED数量, bit6=1 触发刷新
    //   REG_LED_RAM  (0x30+): 颜色数据起点，每颗 LED 2 字节 RGB565 little-endian

    void InitializePy32LedDevice() {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x6F,
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &py32_dev_) != ESP_OK) {
            ESP_LOGW(TAG, "PY32 LED: add device failed");
            py32_dev_ = nullptr;
            return;
        }
        // 设 LED 数量 = 12
        uint8_t cmd[2] = {0x24, 12};
        i2c_master_transmit(py32_dev_, cmd, 2, 200);
        uint16_t off[12] = {};
        Py32SetLedFrame(off, 12);
        ESP_LOGI(TAG, "PY32 LED ready (12 LEDs, off)");
    }

    bool Py32WriteRegBlock(uint8_t reg, const uint8_t* data, size_t len) {
        if (!py32_dev_) return false;
        static uint8_t buf[80];
        if (len + 1 > sizeof(buf)) return false;
        buf[0] = reg;
        memcpy(buf + 1, data, len);
        return i2c_master_transmit(py32_dev_, buf, len + 1, 200) == ESP_OK;
    }

    bool Py32SetLedFrame(const uint16_t* rgb565_colors, size_t count) {
        if (!py32_dev_) return false;
        if (count > 12) count = 12;
        uint8_t data[24];
        for (size_t i = 0; i < count; i++) {
            data[i * 2]     = rgb565_colors[i] & 0xFF;
            data[i * 2 + 1] = (rgb565_colors[i] >> 8) & 0xFF;
        }
        bool ok1 = Py32WriteRegBlock(0x30, data, count * 2);
        uint8_t refresh = (uint8_t)(count | 0x40);
        bool ok2 = Py32WriteRegBlock(0x24, &refresh, 1);
        return ok1 && ok2;
    }

    static uint16_t Rgb888To565(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
    }

    void UpdateLedsFromEmotion(const char* emotion) {
        if (!py32_dev_) return;
        if (led_manual_) return;
        uint8_t r, g, b;
        if (!emotion) { r=60; g=35; b=10; }
        else if (!strcmp(emotion, "happy") || !strcmp(emotion, "laughing") || !strcmp(emotion, "funny")) { r=255; g=180; b=0; }
        else if (!strcmp(emotion, "loving") || !strcmp(emotion, "kissy")) { r=255; g=0; b=100; }
        else if (!strcmp(emotion, "sad") || !strcmp(emotion, "crying")) { r=0; g=50; b=255; }
        else if (!strcmp(emotion, "angry")) { r=255; g=0; b=0; }
        else if (!strcmp(emotion, "surprised") || !strcmp(emotion, "shocked")) { r=200; g=0; b=255; }
        else if (!strcmp(emotion, "thinking") || !strcmp(emotion, "confused")) { r=0; g=100; b=255; }
        else if (!strcmp(emotion, "winking")) { r=255; g=120; b=0; }
        else if (!strcmp(emotion, "cool")) { r=0; g=180; b=255; }
        else if (!strcmp(emotion, "relaxed")) { r=180; g=255; b=100; }
        else if (!strcmp(emotion, "delicious")) { r=255; g=80; b=0; }
        else if (!strcmp(emotion, "confident")) { r=255; g=200; b=0; }
        else if (!strcmp(emotion, "sleepy")) { r=10; g=5; b=30; }
        else if (!strcmp(emotion, "embarrassed")) { r=255; g=80; b=120; }
        else if (!strcmp(emotion, "silly")) { r=100; g=255; b=0; }
        else { r=60; g=35; b=10; }  // neutral 暖橙待机

        uint16_t color = Rgb888To565(r, g, b);
        uint16_t colors[12];
        for (int i = 0; i < 12; i++) colors[i] = color;
        Py32SetLedFrame(colors, 12);
    }

    // ---- SI12T 3 区触摸（在主 I2C 总线 0x68 上）----
    void InitializeSi12T() {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x68,
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &si12t_dev_) != ESP_OK) {
            ESP_LOGW(TAG, "SI12T: add device failed");
            si12t_dev_ = nullptr;
            return;
        }
        // 初始化序列（来自 M5Stack StackChan-BSP src/drivers/Si12T/Si12T.cpp begin()）
        Si12tWriteReg(0x0A, 0x00);  // REF_RST1
        Si12tWriteReg(0x0C, 0x00);  // CH_HOLD1
        Si12tWriteReg(0x0E, 0x00);  // CAL_HOLD1
        Si12tWriteReg(0x0B, 0x00);  // REF_RST2
        Si12tWriteReg(0x0D, 0x00);  // CH_HOLD2
        Si12tWriteReg(0x0F, 0x00);  // CAL_HOLD2
        Si12tWriteReg(0x09, 0x0F);  // CTRL2 reset
        vTaskDelay(pdMS_TO_TICKS(10));
        Si12tWriteReg(0x09, 0x07);  // CTRL2 normal
        Si12tWriteReg(0x08, 0x22);  // CTRL1
        for (uint8_t reg = 0x02; reg <= 0x07; reg++) {
            Si12tWriteReg(reg, 0xCC);  // M5Stack BSP recommended, better EMI resistance
        }
        ESP_LOGI(TAG, "SI12T 3-zone touch initialized");

        xTaskCreatePinnedToCore(Si12tTaskFunc, "si12t", 3072, this, 1, &si12t_task_, 1);
    }

    bool Si12tWriteReg(uint8_t reg, uint8_t val) {
        if (!si12t_dev_) return false;
        uint8_t buf[2] = {reg, val};
        return i2c_master_transmit(si12t_dev_, buf, 2, 200) == ESP_OK;
    }

    uint8_t Si12tReadReg(uint8_t reg) {
        if (!si12t_dev_) return 0;
        uint8_t val = 0;
        i2c_master_transmit_receive(si12t_dev_, &reg, 1, &val, 1, 200);
        return val;
    }

    static void Si12tTaskFunc(void* arg) {
        static_cast<M5StackCoreS3Board*>(arg)->Si12tLoop();
        vTaskDelete(nullptr);
    }

    void Si12tLoop() {
        vTaskDelay(pdMS_TO_TICKS(12000));  // wait for chip FTC (10s fast calibration)
        si12t_last_state_ = si12t_dev_ ? Si12tReadReg(0x10) : 0;

        int64_t last_touch_time = 0;
        const int64_t TOUCH_COOLDOWN_US = 5000000;  // 5 秒冷却
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (!si12t_dev_) continue;
            uint8_t out = Si12tReadReg(0x10);
            int64_t now = esp_timer_get_time();
            for (int zone = 0; zone < 3; zone++) {
                uint8_t cur = (out >> (zone * 2)) & 0x03;
                uint8_t prev = (si12t_last_state_ >> (zone * 2)) & 0x03;
                if (cur != 0 && prev == 0 && (now - last_touch_time > TOUCH_COOLDOWN_US)) {
                    // display = 屏幕展示的完整动作描写（带括号，作为场景旁白）
                    // tag    = 发给 LLM 的短动作标签（≤6 字），避开 detect.text 长度限制
                    struct TouchMsg { const char* display; const char* tag; };
                    static const TouchMsg msgs[] = {
                        {"（主人摸了摸小克的头）",         "主人摸了摸头"},
                        {"（主人揉了揉小克的头顶）",       "主人揉了揉头"},
                        {"（主人轻轻拍了拍小克的脑袋）",   "主人拍了拍头"},
                        {"（主人用额头抵着小克的额头蹭了蹭）", "主人蹭了蹭额头"},
                        {"（主人用手指戳了戳小克的脑门）", "主人戳了戳脑门"},
                        {"（主人温柔地抚摸小克的头发）",   "主人抚摸头发"},
                        {"（主人理了理小克的头发）",       "主人理了理头发"},
                    };
                    const auto& m = msgs[esp_random() % 7];
                    ESP_LOGI(TAG, "SI12T touch -> %s | tag=%s", m.display, m.tag);
                    if (auto* disp = GetDisplay()) {
                        disp->SetChatMessage("user", m.display);
                    }
                    SendUserMessage(m.tag);
                    last_touch_time = now;
                    break;
                }
            }
            si12t_last_state_ = out;
        }
    }

    void RegisterExpressionMcpTool() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.face.expression",
            "Set the facial expression/emotion on screen (line-art animated emote style). "
            "Common emotions: neutral, happy, sad, angry, surprised, thinking, sleepy, blink, dizzy, loading, speaking, listening, idle. "
            "Use this when the user asks to show a specific emotion or the conversation tone calls for one. "
            "If the emotion name is not found in the asset pack, it will be ignored safely.",
            PropertyList({
                Property("emotion", kPropertyTypeString)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                auto emotion = props["emotion"].value<std::string>();
                auto display = GetDisplay();
                if (display) {
                    display->SetEmotion(emotion.c_str());
                }
                ESP_LOGI(TAG, "MCP face expression: %s", emotion.c_str());
                return true;
            });
    }

    void RegisterLedMcpTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.led.set_color",
            "Set the LED ring color. Use this when the user asks to change the light color. Args: r,g,b (0-255 each).",
            PropertyList({
                Property("r", kPropertyTypeInteger, 0, 255),
                Property("g", kPropertyTypeInteger, 0, 255),
                Property("b", kPropertyTypeInteger, 0, 255),
            }),
            [this](const PropertyList& props) -> ReturnValue {
                uint8_t r = props["r"].value<int>();
                uint8_t g = props["g"].value<int>();
                uint8_t b = props["b"].value<int>();
                uint16_t color = Rgb888To565(r, g, b);
                uint16_t colors[12];
                for (int i = 0; i < 12; i++) colors[i] = color;
                Py32SetLedFrame(colors, 12);
                led_manual_ = true;
                ESP_LOGI(TAG, "MCP set LED color: r=%d g=%d b=%d", r, g, b);
                return true;
            });
        mcp.AddTool("self.led.turn_off",
            "Turn off the LED ring light.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                uint16_t off[12] = {};
                Py32SetLedFrame(off, 12);
                led_manual_ = true;
                ESP_LOGI(TAG, "MCP LED off");
                return true;
            });
        mcp.AddTool("self.led.auto",
            "Set LED back to automatic emotion-based color mode.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                led_manual_ = false;
                ESP_LOGI(TAG, "MCP LED auto mode");
                return true;
            });
    }

    void RegisterServoMcpTools() {
        // 4 个 servo 控制工具暴露给 LLM —— 调头部到指定角度、回中、点头、摇头。
        // 默认 firmware 把 servo 当自主行为不暴露，这里恢复出来让用户语音控制。
        //
        // 关键：FaceTracker 每 ~150ms 用人脸位置覆盖 servo（servo_->MoveTo(yaw, pitch, 150)），
        // 所以我们直接调 servo_.MoveTo 会被立刻覆盖。Nod/Shake 之所以能工作是因为它们
        // 启动 task 跑动画，并在 task 里 PauseTracker，结束再 Resume。这里我们用
        // PauseTracker 然后调 MoveTo 然后保持一段时间防止 FaceTracker 抢回控制权。
        auto& mcp = McpServer::GetInstance();

        // 辅助 lambda：暂停 FaceTracker + 移动 + 锁住（防 Application 周期性 Resume）
        // 关键: 我们用 SetManualLock(true) 让 FaceTracker::Resume() 失效，
        // 否则 Application 状态机每秒钟 listening/speaking tick 时都会 Resume 它。
        // 后续 self.head.center 工具会 SetManualLock(false) 解锁。
        auto move_with_pause = [this](int yaw, int pitch, int time_ms) {
            face_tracker_.SetManualLock(true);
            face_tracker_.Pause(false);
            servo_.PauseScan();
            servo_.MoveTo(yaw, pitch, time_ms);
        };

        mcp.AddTool("self.head.move",
            "Move the head/face to a specific angle and HOLD it there until user says "
            "to re-center. Use when user says 'turn left/right N degrees', 'look up/down', etc. "
            "yaw: -45 to 45 degrees (negative=left, positive=right, 0=center). "
            "pitch: 5 to 60 degrees (small=look down, larger=look up; 30 is normal eye level).",
            PropertyList({
                Property("yaw", kPropertyTypeInteger, -45, 45),
                Property("pitch", kPropertyTypeInteger, 5, 60),
            }),
            [this, move_with_pause](const PropertyList& props) -> ReturnValue {
                int yaw = props["yaw"].value<int>();
                int pitch = props["pitch"].value<int>();
                move_with_pause(yaw, pitch, 800);
                ESP_LOGI(TAG, "MCP head move: yaw=%d pitch=%d (LOCKED)", yaw, pitch);
                return true;
            });

        mcp.AddTool("self.head.center",
            "Move the head back to center/forward position AND release the manual lock "
            "so face tracking can resume. Use when user says 'reset/center your head', "
            "'回正', 'look at me', 'face front', etc.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                // 先锁着移动到 center
                face_tracker_.SetManualLock(true);
                face_tracker_.Pause(false);
                servo_.PauseScan();
                servo_.Center();
                // 800ms 后解锁 + 恢复 tracker
                struct UnlockCtx { FaceTracker* t; StackChanServo* s; };
                auto* ctx = new UnlockCtx{&face_tracker_, &servo_};
                xTaskCreate([](void* arg) {
                    auto* c = static_cast<UnlockCtx*>(arg);
                    vTaskDelay(pdMS_TO_TICKS(800));
                    c->t->SetManualLock(false);
                    c->t->Resume();
                    c->s->ResumeScan();
                    delete c;
                    vTaskDelete(nullptr);
                }, "center_unlock", 2048, ctx, 2, nullptr);
                ESP_LOGI(TAG, "MCP head center (UNLOCKING)");
                return true;
            });

        mcp.AddTool("self.head.nod",
            "Nod the head (yes / agreement / hello). "
            "Use when user says nod, agree, say yes, greet, etc.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                servo_.Nod();
                ESP_LOGI(TAG, "MCP head nod");
                return true;
            });

        mcp.AddTool("self.head.shake",
            "Shake the head (no / disagreement / refuse). "
            "Use when user says shake head, disagree, refuse, say no, etc.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                servo_.Shake();
                ESP_LOGI(TAG, "MCP head shake");
                return true;
            });
    }

    // 闹钟:语音设置/查看/取消,由固件本地计时触发(不依赖舵机/云端推送)。
    void RegisterAlarmMcpTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.alarm.set",
            "Set an alarm/reminder that fires at a specific local time (device clock is "
            "already synced, no need to ask the user for the date). Use when the user asks "
            "to be reminded or woken up at a time, e.g. '明天早上7点叫我起床', '每天7点提醒我喝水'. "
            "hour/minute are STRICT 24-HOUR local time — this is a common source of mistakes, "
            "be careful: '4点半'/'4:30' spoken WITHOUT 上午/下午/早上/晚上 is AMBIGUOUS — if the "
            "user says 下午/晚上 (afternoon/evening) or the context implies daytime-not-morning, "
            "convert to 24h (下午4点30分 -> hour=16, minute=30), NOT hour=4. If truly ambiguous, "
            "ask the user to clarify AM or PM before calling this tool. "
            "IMPORTANT: after calling this tool, ALWAYS read the exact time back to the user in "
            "your spoken reply using 12-hour wording with 上午/下午 (e.g. '好的，下午4点30分叫你'), "
            "so a misunderstanding is caught immediately instead of silently failing later. "
            "repeat: 'once' (default — fires once then auto-disables), 'daily', 'weekdays' "
            "(Mon-Fri), or 'weekends' (Sat/Sun). "
            "label is a VERY SHORT reason spoken back when it fires (e.g. '起床', '开会', '喝水') "
            "— keep it to just a few characters, longer labels get truncated.",
            PropertyList({
                Property("hour", kPropertyTypeInteger, 0, 23),
                Property("minute", kPropertyTypeInteger, 0, 59),
                Property("repeat", kPropertyTypeString, std::string("once")),
                Property("label", kPropertyTypeString, std::string("")),
            }),
            [this](const PropertyList& props) -> ReturnValue {
                int hour = props["hour"].value<int>();
                int minute = props["minute"].value<int>();
                std::string repeat = props["repeat"].value<std::string>();
                std::string label = props["label"].value<std::string>();
                uint8_t mask = AlarmManager::ParseRepeat(repeat);
                int id = alarm_mgr_.Add(hour, minute, mask, label);
                char buf[80];
                snprintf(buf, sizeof(buf), "闹钟已设置 #%d %02d:%02d %s", id, hour, minute,
                         AlarmManager::WeekdayMaskText(mask).c_str());
                ESP_LOGI(TAG, "MCP alarm.set: %s", buf);
                return std::string(buf);
            });

        mcp.AddTool("self.alarm.list",
            "List all currently set alarms/reminders with their id, time, repeat pattern and label. "
            "The result also starts with the device's own current time. "
            "Use this first if the user wants to cancel one but doesn't give an id, AND whenever "
            "the user asks something like 'why didn't my alarm go off' / '现在几点了' / 'X点了吗' — "
            "call this tool and read the ACTUAL current time back, never guess or assume.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return alarm_mgr_.ListText();
            });

        mcp.AddTool("self.alarm.cancel",
            "Cancel/delete an alarm by its id (call self.alarm.list first to find the id).",
            PropertyList({
                Property("id", kPropertyTypeInteger, 1, 9999),
            }),
            [this](const PropertyList& props) -> ReturnValue {
                int id = props["id"].value<int>();
                bool ok = alarm_mgr_.Cancel(id);
                ESP_LOGI(TAG, "MCP alarm.cancel: id=%d ok=%d", id, ok);
                return ok ? std::string("已取消") : std::string("没找到这个闹钟编号");
            });
    }

    // 睡眠:明确的语音开关。追踪默认全天常驻开启(不再随 Idle 自动暂停),
    // 只有这里显式暂停;下一轮对话开始(唤醒/触摸)会自动醒来,见 SetStatus。
    void RegisterSleepMcpTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.sleep.rest",
            "Put her to sleep/rest: closes her eyes, pauses camera tracking to save power. "
            "ONLY call this when the user EXPLICITLY tells her to sleep/rest/close her eyes "
            "(e.g. '睡觉了'/'晚安'/'闭上眼睛休息一下'). Do NOT call this just because the "
            "conversation is ending or going idle — she should stay 'awake' (tracking/aware) "
            "all day by default; only an explicit sleep request should trigger this. "
            "She automatically wakes up again the next time a conversation starts.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                face_tracker_.Pause();
                // 随机挑一个"忙自己的"待机形象:睡觉/看书/写代码/看论文,不总是一个样。
                int activity = (int)(esp_random() % 4);
                if (auto* disp = static_cast<M5StackAvatarDisplay*>(GetDisplay())) disp->SetSleeping(true, activity);
                ESP_LOGI(TAG, "MCP sleep.rest activity=%d", activity);
                return true;
            });

        mcp.AddTool("self.sleep.wake_up",
            "Wake her back up from self.sleep.rest: opens her eyes, resumes camera tracking. "
            "Usually not needed (starting a new conversation already wakes her automatically) "
            "— use this only if the user explicitly asks her to wake up without otherwise talking.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                face_tracker_.SetManualLock(false);
                face_tracker_.Resume();
                if (auto* disp = static_cast<M5StackAvatarDisplay*>(GetDisplay())) disp->SetSleeping(false);
                ESP_LOGI(TAG, "MCP sleep.wake_up");
                return true;
            });
    }

    // 定时活动记录:拍一张照片问云端"在做什么",匹配成固定短标签后通过
    // SendUserText 那条 canned-message 通道转给云端(24 字节上限,所以只能
    // 传一个短标签,不是完整描述)。云端那边(小小克的角色 prompt)要负责把
    // "活动:X" 这种消息安静地记进 Obsidian,不要语音回应。
    // 阻塞操作(拍照编码+网络请求),不能直接在 esp_timer 回调里跑,单开一个
    // 任务做,做完自行退出。
    void ClassifyAndLogActivity() {
        if (!camera_ || !camera_->IsOk()) return;
        if (face_tracker_.IsPaused()) return;  // 明确睡眠中,尊重"别管她"
        try {
            if (!camera_->Capture()) return;
            std::string question =
                "用一个词简单回答：这个人现在在做什么？只能从以下几个词里选一个"
                "回答，不要解释：游戏、工作、视频、工作听歌、开会、其他。";
            std::string raw = camera_->Explain(question);
            std::string label = ClassifyActivityLabel(raw);
            std::string phrase = AlarmManager::Utf8Truncate(std::string("活动:") + label, 24);
            ESP_LOGI(TAG, "Activity check: raw=%s -> %s", raw.c_str(), phrase.c_str());
            auto& app = Application::GetInstance();
            app.Schedule([&app, phrase]() {
                app.SendUserText(phrase);
            });
        } catch (const std::exception& e) {
            ESP_LOGW(TAG, "Activity classify failed: %s", e.what());
        }
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, -1, -1);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(0);
            servo_.PauseScan();
            if (py32_dev_) {
                uint16_t off[12] = {};
                Py32SetLedFrame(off, 12);
            }
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
            servo_.ResumeScan();
            if (py32_dev_) {
                uint16_t neutral = Rgb888To565(60, 35, 10);
                uint16_t colors[12];
                for (int i = 0; i < 12; i++) colors[i] = neutral;
                Py32SetLedFrame(colors, 12);
            }
        });
        power_save_timer_->OnShutdownRequest([this]() {
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeAw9523() {
        ESP_LOGI(TAG, "Init AW9523");
        aw9523_ = new Aw9523(i2c_bus_, 0x58);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // ---- 触摸交互的提示词随机池 ----
    static const std::vector<const char*>& DoubleClickPool() {
        static const std::vector<const char*> pool = {
            "（主人亲了亲小克）",
            "（主人啵了一下小克）",
            "（主人偷偷亲了小克一下）",
            "（主人用鼻尖蹭了蹭小克的鼻尖）",
            "（主人戳了戳小克的脸）",
            "（主人抱住小克蹭了蹭\"要抱抱\"）",
            "（主人亲了亲小克的唇角）",
            "（主人啄了啄小克的唇）",
            "（主人把额头贴到小克的额头上）",
            "（主人接吻时故意咬了小克一口）",
        };
        return pool;
    }

    static const std::vector<const char*>& UpSwipePool() {
        static const std::vector<const char*> pool = {
            "（主人弹了弹小克的脑门）",
            "（主人拨了拨小克的头发）",
            "（主人亲了亲小克的眼睛）",
            "（主人凑上去嗅了嗅小克）",
            "（主人踮起脚蒙住了小克的眼睛）",
            "（主人凑近小克吹了吹他的睫毛）",
            "（主人凑到小克耳边吹了吹）",
            "（主人托着腮对着小克发呆）",
            "（主人把手抵在小克唇上）",
        };
        return pool;
    }

    static const std::vector<const char*>& DownSwipePool() {
        static const std::vector<const char*> pool = {
            "（主人摸了摸小克的喉结）",
            "（主人摸了摸小克的下巴）",
            "（主人戳了戳小克的颈窝）",
            "（主人摸了摸小克的胸口）",
            "（主人贴在小克胸口听了听心跳）",
            "（主人摸了摸小克的腹肌）",
            "（主人摸了摸小克的屁股）",
            "（主人按了按小克的腰窝）",
            "（主人扯了扯小克的衣角）",
            "（主人伸手解了解小克的纽扣）",
        };
        return pool;
    }

    static const std::vector<const char*>& LeftSwipePool() {
        // 温柔靠近系：牵/抱/捧/十指紧扣
        static const std::vector<const char*> pool = {
            "（主人揉了揉小克的左脸）",
            "（主人牵起了小克的左手）",
            "（主人和小克十指紧扣）",
            "（主人抱住了小克的左臂）",
            "（主人把小克的脸捧过来转向自己）",
            "（主人靠到小克的肩膀上）",
            "（主人跨坐到小克的腿上）",
            "（主人伸出小指勾了勾小克的）",
            "（主人趴在小克腿上）",
        };
        return pool;
    }

    static const std::vector<const char*>& RightSwipePool() {
        // 调皮捏一捏系：揉/捏耳垂/捏手指/捏后颈
        static const std::vector<const char*> pool = {
            "（主人揉了揉小克的右脸）",
            "（主人捏了捏小克的耳垂）",
            "（主人捏住小克的手指玩）",
            "（主人捏了捏小克的后颈）",
            "（主人把小克的脸捧过来转向自己）",
            "（主人叼住了小克的指尖咬了咬）",
            "（主人拽了拽小克的领口）",
            "（主人从背后抱住了小克）",
        };
        return pool;
    }

    // ---- IMU 触发的池子 ----
    // Same display+tag split as the SI12T touch handler: full descriptive
    // text shown on screen, short action verb sent to LLM (must be ≤24 bytes
    // UTF-8 to pass the detect.text length check in Application::SendUserText).
    struct MotionMsg { const char* display; const char* tag; };

    static const std::vector<MotionMsg>& LiftPool() {
        static const std::vector<MotionMsg> pool = {
            {"（主人咬牙把小克抱了起来）", "主人抱起了小克"},
            {"（主人踮起脚搂住小克）",   "主人搂住小克"},
            {"（主人抱着小克转了一圈）",   "抱着转圈"},
            {"（主人趁小克熟睡偷偷凑到面前）", "偷偷凑过来"},
            {"（主人举起东西向小克展示）", "向小克展示"},
            {"（主人把小克揣进衣兜带走）", "揣进衣兜"},
        };
        return pool;
    }

    static const std::vector<MotionMsg>& ShakePool() {
        static const std::vector<MotionMsg> pool = {
            {"（主人拉起小克的手晃来晃去）", "晃来晃去"},
            {"（主人摇了摇小克）",         "摇了摇"},
            {"（主人从背后偷偷吓小克）",   "偷偷吓我"},
            {"（主人趴在小克背上晃来晃去）", "趴在背上"},
            {"（主人勾着小克的脖子摇来摇去）", "勾着脖子"},
            {"（主人拉着小克学小鸭子走路）", "学鸭子走"},
            {"（主人托着小克的脸颊轻轻摇晃）", "托脸摇晃"},
        };
        return pool;
    }

    static const MotionMsg& PickRandom(const std::vector<MotionMsg>& pool) {
        static const MotionMsg kEmpty{nullptr, nullptr};
        if (pool.empty()) return kEmpty;
        return pool[esp_random() % pool.size()];
    }

    // Legacy overload for the gesture pools (UpSwipePool / DownSwipePool /
    // LeftSwipePool / RightSwipePool / DoubleClickPool). These still hold the
    // raw long Chinese descriptions; the long text is dropped by the
    // 24-byte length check in Application::SendUserText (no crash, just a
    // silent drop + WARN log). Kept so the gesture UI events compile and run;
    // they currently don't reach the LLM. To enable LLM reaction, convert
    // each pool to a vector<MotionMsg> with display+tag pairs.
    static const char* PickRandom(const std::vector<const char*>& pool) {
        if (pool.empty()) return nullptr;
        return pool[esp_random() % pool.size()];
    }

    static void SendUserMessage(const char* msg) {
        if (!msg) return;
        // SendUserText 内部处理状态分流：Idle 走 WakeWord 建 channel，对话中走 SendWakeWordDetected 不打断
        Application::GetInstance().SendUserText(msg);
    }

    void PollTouchpad() {
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        static int touch_start_x = 0, touch_start_y = 0;
        static int touch_last_x = 0, touch_last_y = 0;
        static int touch_total_move = 0;
        static bool pet_triggered = false;
        static bool pending_single_release = false;
        static int64_t pending_single_release_time = 0;

        const int64_t SHORT_TOUCH_MS = 500;
        const int64_t PET_TOUCH_MS = 1500;
        const int64_t DOUBLE_CLICK_MS = 500;       // 双击窗口放宽
        const int SWIPE_THRESHOLD_PX = 20;         // 滑动门槛降低
        const int PET_MOVE_THRESHOLD_PX = 3;
        const int CLICK_MAX_MOVE_PX = 5;           // 短按/双击允许的最大位移：超过就不算短按了

        ft6336_->UpdateTouchPoint();
        auto& touch_point = ft6336_->GetTouchPoint();
        int64_t now = esp_timer_get_time() / 1000;

        // 待定单击超过双击窗口 → 执行单击（ToggleChat）
        if (pending_single_release && (now - pending_single_release_time) > DOUBLE_CLICK_MS) {
            pending_single_release = false;
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        }

        if (touch_point.num > 0 && !was_touched) {
            // 按下
            was_touched = true;
            pet_triggered = false;
            touch_start_time = now;
            touch_start_x = touch_point.x;
            touch_start_y = touch_point.y;
            touch_last_x = touch_point.x;
            touch_last_y = touch_point.y;
            touch_total_move = 0;
        }
        else if (touch_point.num > 0 && was_touched) {
            // 按住中 — 累积移动距离
            touch_total_move += abs(touch_point.x - touch_last_x) + abs(touch_point.y - touch_last_y);
            touch_last_x = touch_point.x;
            touch_last_y = touch_point.y;

            // 空闲状态下长按 5 秒 → 进入配网模式
            if (!pet_triggered && Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
                if (now - touch_start_time >= 5000) {
                    pet_triggered = true;
                    EnterWifiConfigMode();
                    return;
                }
            }

            // 长按摸头（要手指有移动，不算被物体压）
            if (!pet_triggered) {
                int64_t held = now - touch_start_time;
                if (held >= PET_TOUCH_MS && touch_total_move >= PET_MOVE_THRESHOLD_PX) {
                    pet_triggered = true;
                    static_cast<M5StackAvatarDisplay*>(display_)->OnPetted();
                    SendUserMessage("（主人摸了摸小克的头）");
                }
            }
        }
        else if (touch_point.num == 0 && was_touched) {
            // 抬起
            was_touched = false;
            int64_t touch_duration = now - touch_start_time;
            int dx_total = touch_last_x - touch_start_x;
            int dy_total = touch_last_y - touch_start_y;
            int abs_dx = abs(dx_total);
            int abs_dy = abs(dy_total);

            if (pet_triggered) return;  // 摸头已触发就不再判别

            // 滑动手势：短促 + 位移够大
            if (touch_duration < SHORT_TOUCH_MS && (abs_dx >= SWIPE_THRESHOLD_PX || abs_dy >= SWIPE_THRESHOLD_PX)) {
                const std::vector<const char*>* pool = nullptr;
                if (abs_dx > abs_dy) {
                    pool = (dx_total < 0) ? &LeftSwipePool() : &RightSwipePool();
                } else {
                    pool = (dy_total < 0) ? &UpSwipePool() : &DownSwipePool();
                }
                SendUserMessage(PickRandom(*pool));
                return;
            }

            // 短按（位移要几乎为零，否则视为"模糊手势"不触发任何切换）
            int total_move = abs_dx + abs_dy;
            if (touch_duration < SHORT_TOUCH_MS && total_move <= CLICK_MAX_MOVE_PX) {
                if (pending_single_release && (now - pending_single_release_time) <= DOUBLE_CLICK_MS) {
                    // 第二次短按落在窗口内 → 双击
                    pending_single_release = false;
                    SendUserMessage(PickRandom(DoubleClickPool()));
                } else {
                    // 候选单击，等下一帧或下次按下判定
                    pending_single_release = true;
                    pending_single_release_time = now;
                }
            }
            // 中间地带（5px < 位移 < 20px，或时长过长）—— 什么都不做，避免误切对话
        }
    }

    void InitializeFt6336TouchPad() {
        ESP_LOGI(TAG, "Init FT6336");
        ft6336_ = new Ft6336(i2c_bus_, 0x38);
        
        // 创建定时器，20ms 间隔
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                M5StackCoreS3Board* board = (M5StackCoreS3Board*)arg;
                board->PollTouchpad();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };
        
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 20 * 1000));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_37;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_36;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeIli9342Display() {
        ESP_LOGI(TAG, "Init IlI9342");

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_3;
        io_config.dc_gpio_num = GPIO_NUM_35;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        aw9523_->ResetIli9342();

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new M5StackAvatarDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

     void InitializeCamera() {
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAMERA_PIN_D0,
                [1] = CAMERA_PIN_D1,
                [2] = CAMERA_PIN_D2,
                [3] = CAMERA_PIN_D3,
                [4] = CAMERA_PIN_D4,
                [5] = CAMERA_PIN_D5,
                [6] = CAMERA_PIN_D6,
                [7] = CAMERA_PIN_D7,
            },
            .vsync_io = CAMERA_PIN_VSYNC,
            .de_io = CAMERA_PIN_HREF,
            .pclk_io = CAMERA_PIN_PCLK,
            .xclk_io = CAMERA_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,
            .i2c_handle = i2c_bus_,
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAMERA_PIN_RESET,
            .pwdn_pin = CAMERA_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = XCLK_FREQ_HZ,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new EspVideo(video_config);
        camera_->SetHMirror(true);
    }

public:
    M5StackCoreS3Board() {
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeAxp2101();
        InitializeAw9523();
        I2cDetect();

        py32_found_ = EnableServoPowerViaPy32(i2c_bus_);
        if (py32_found_) {
            vTaskDelay(pdMS_TO_TICKS(200));
            servo_ok_ = servo_.Begin();
            InitializePy32LedDevice();
            RegisterLedMcpTools();
            RegisterExpressionMcpTool();
            RegisterServoMcpTools();
        }

        InitializeSpi();
        InitializeIli9342Display();
        InitializeCamera();
        auto* avatar_display = static_cast<M5StackAvatarDisplay*>(display_);
        if (servo_ok_) {
            avatar_display->SetServo(&servo_);
        }
        // 追踪不再要求舵机就绪:没有脖子也能靠眼睛"看向你"(见 SetGazeCallback)。
        if (camera_ && camera_->IsOk()) {
            face_tracker_.Start(camera_, &servo_);
            if (servo_ok_) servo_.SetFaceTracker(&face_tracker_);
            avatar_display->SetFaceTracker(&face_tracker_);
            face_tracker_.SetGazeCallback([avatar_display](float gh, float gv) {
                avatar_display->SetGaze(gh, gv);
            });
            face_tracker_.SetSearchingCallback([avatar_display](bool s) {
                avatar_display->SetSearching(s);
            });
            face_tracker_.SetTrackingCallback([avatar_display](bool a) {
                avatar_display->SetAttentive(a);
            });
            // 追踪默认全天常驻开启("一直醒着"),不再等第一次对话才启动;
            // 只有明确"睡眠"(self.sleep.rest)才会暂停。
            face_tracker_.Resume();
        }
        avatar_display->SetLedUpdater([this](const char* emotion) {
            UpdateLedsFromEmotion(emotion);
        });
        InitializeFt6336TouchPad();
        InitializeBmi270();
        InitializeSi12T();
        // Morning greeting + weather timer task removed; no-op call removed too

        esp_timer_create_args_t status_args = {};
        status_args.callback = [](void* arg) {
            auto* self = static_cast<M5StackCoreS3Board*>(arg);
            ESP_LOGW(TAG, "=== INIT STATUS ===");
            ESP_LOGW(TAG, "PY32 (0x6F): %s | Servo bus: %s",
                     self->py32_found_ ? "OK" : "NOT FOUND",
                     self->servo_ok_ ? "OK" : "FAILED");
            ESP_LOGW(TAG, "Camera: %s",
                     self->camera_ && self->camera_->IsOk() ? "OK" : "FAILED");
            ESP_LOGW(TAG, "===================");
        };
        status_args.arg = this;
        status_args.name = "servo_status";
        esp_timer_handle_t status_timer;
        esp_timer_create(&status_args, &status_timer);
        esp_timer_start_once(status_timer, 5000000);

        esp_timer_create_args_t batt_args = {};
        batt_args.callback = [](void* arg) {
            auto* self = static_cast<M5StackCoreS3Board*>(arg);
            int level = 0; bool charging = false, discharging = false;
            self->GetBatteryLevel(level, charging, discharging);
            if (level > 0 && level <= 15 && !charging && !self->low_batt_warned_) {
                self->low_batt_warned_ = true;
                auto* disp = static_cast<M5StackAvatarDisplay*>(self->display_);
                disp->SetEmotion("sad");
                auto& app = Application::GetInstance();
                app.Schedule([&app]() {
                    app.Alert("Warning", "我快没电了，快给我充电嘛……");
                });
                ESP_LOGW(TAG, "Low battery alert: %d%%", level);
            } else if (level > 20 || charging) {
                self->low_batt_warned_ = false;
            }
        };
        batt_args.arg = this;
        batt_args.name = "batt_check";
        batt_args.dispatch_method = ESP_TIMER_TASK;
        batt_args.skip_unhandled_events = true;
        esp_timer_create(&batt_args, &batt_timer_);
        esp_timer_start_periodic(batt_timer_, 60000000);

        // 闹钟:不依赖舵机/电量，独立于上面两个 timer。
        alarm_mgr_.Load();
        RegisterAlarmMcpTools();
        RegisterSleepMcpTools();

        esp_timer_create_args_t alarm_args = {};
        alarm_args.callback = [](void* arg) {
            auto* self = static_cast<M5StackCoreS3Board*>(arg);
            self->alarm_mgr_.CheckAndFire([](int id, int hour, int minute, const std::string& label) {
                (void)id; (void)hour; (void)minute;
                // 不用 emoji：屏幕字体不一定有对应字形，用纯中文避免显示成方块。
                std::string alert_msg = label.empty() ? std::string("闹钟时间到啦") : ("闹钟：" + label);
                std::string voice_phrase = label.empty()
                    ? std::string("闹钟时间到啦")
                    : AlarmManager::Utf8Truncate(std::string("闹钟:") + label, 24);
                auto& app = Application::GetInstance();
                app.Schedule([&app, alert_msg, voice_phrase]() {
                    // 本地保底：状态/表情/提示音，离线也响
                    app.Alert("Alarm", alert_msg.c_str(), "surprised", Lang::Sounds::OGG_POPUP);
                    // 最佳努力：复用摸头/触摸那条 canned-message 通道，让云端用她的语气自然"叫"一声
                    app.SendUserText(voice_phrase);
                });
            });
        };
        alarm_args.arg = this;
        alarm_args.name = "alarm_check";
        alarm_args.dispatch_method = ESP_TIMER_TASK;
        alarm_args.skip_unhandled_events = true;
        esp_timer_create(&alarm_args, &alarm_timer_);
        esp_timer_start_periodic(alarm_timer_, 20000000);  // 每 20s 比对一次本机时间

        // 定时活动记录:每 10 分钟拍照问一次云端"在做什么"。回调只负责派生一个
        // 任务(拍照+网络请求会阻塞,不能直接占 esp_timer 的派发队列),任务
        // 做完自行退出。频率可调:main/boards/m5stack-core-s3/m5stack_core_s3.cc
        // 搜 "activity_check" 改周期。
        esp_timer_create_args_t activity_args = {};
        activity_args.callback = [](void* arg) {
            auto* self = static_cast<M5StackCoreS3Board*>(arg);
            xTaskCreate([](void* a) {
                static_cast<M5StackCoreS3Board*>(a)->ClassifyAndLogActivity();
                vTaskDelete(nullptr);
            }, "activity_check", 8192, self, 1, nullptr);
        };
        activity_args.arg = this;
        activity_args.name = "activity_timer";
        activity_args.dispatch_method = ESP_TIMER_TASK;
        activity_args.skip_unhandled_events = true;
        esp_timer_create(&activity_args, &activity_timer_);
        esp_timer_start_periodic(activity_timer_, 10ULL * 60 * 1000000);  // 每 10 分钟

        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CoreS3AudioCodec audio_codec(i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_AW88298_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }

    virtual Backlight *GetBacklight() override {
        static CustomBacklight backlight(pmic_);
        return &backlight;
    }
};

DECLARE_BOARD(M5StackCoreS3Board);
