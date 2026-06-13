/**
 * IoT Fall Detection IMU streamer - ESP32 NodeMCU-32S
 *
 * WT61PC frames are read from UART2 and converted into latest-value snapshots.
 * Complete snapshots are queued for a BLE client. The queue always favors the
 * newest data so a slow link cannot build an unbounded realtime delay.
 *
 * BLE packet layout (61 bytes, little-endian):
 *   [0-1]   uint8[2]  magic = 0xAA 0x55
 *   [2-3]   uint16    sequence
 *   [4-15]  float[3]  ax, ay, az (m/s^2)
 *   [16-27] float[3]  gx, gy, gz (deg/s)
 *   [28-39] float[3]  roll, pitch, yaw (deg)
 *   [40-55] float[4]  q0, q1, q2, q3
 *   [56-59] uint32    ESP32 timestamp (ms)
 *   [60]    uint8     XOR checksum of bytes [0..59]
 *
 * There is no battery field and no battery ADC logic.
 */

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#ifndef IMU_SERIAL_DEBUG
#define IMU_SERIAL_DEBUG 0
#endif

// Pin and UART configuration.
#define IMU_RX_PIN 16
#define IMU_TX_PIN 17
#define LED_PIN 2

static const uint32_t kImuBauds[] = {9600, 115200};
static const size_t kImuBaudCnt = sizeof(kImuBauds) / sizeof(kImuBauds[0]);
static const uint32_t kImuTimeoutMs = 1500;
static const uint32_t kBaudRetryMs = 1500;

// BLE configuration.
#define BLE_DEVICE_NAME "FallDetect-IMU"
#define SERVICE_UUID "12345678-1234-1234-1234-123456789012"
#define CHAR_IMU_UUID "12345678-1234-1234-1234-123456789abc"
#define CHAR_STATUS_UUID "12345678-1234-1234-1234-123456789def"
#define BLE_LOCAL_MTU 512
#define BLE_PACKET_MIN_MTU 64

// FreeRTOS state.
#define EVT_BLE_CONN BIT0
#define EVT_IMU_READY BIT1
#define EVT_IMU_TIMEOUT BIT2
#define IMU_Q_LEN 8

struct ImuFrame {
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    float q0, q1, q2, q3;
    uint32_t esp_ms;
};

#pragma pack(push, 1)
struct BlePacket {
    uint8_t magic[2];
    uint16_t seq;
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    float q0, q1, q2, q3;
    uint32_t esp_ms;
    uint8_t csum;
};
#pragma pack(pop)

static_assert(sizeof(BlePacket) == 61, "BlePacket must remain 61 bytes");

static QueueHandle_t g_imuQ = nullptr;
static EventGroupHandle_t g_events = nullptr;
static BLEServer *g_server = nullptr;
static BLECharacteristic *g_charImu = nullptr;
static BLECharacteristic *g_charStat = nullptr;
static BLE2902 *g_notifyDescriptor = nullptr;
static TaskHandle_t g_taskImu = nullptr;
static TaskHandle_t g_taskBleTx = nullptr;
static TaskHandle_t g_taskLed = nullptr;

// These aligned scalar values are read by BLE callbacks and written by tasks.
static volatile uint16_t g_seq = 0;
static volatile uint16_t g_peerMtu = 23;
static volatile uint32_t g_checksumErrorCount = 0;
static volatile uint32_t g_queueDropCount = 0;
static volatile uint32_t g_packetsSentCount = 0;
static volatile uint32_t g_notifyErrorCount = 0;
static volatile bool g_advertising = true;
static volatile bool g_quatValid = false;
static volatile bool g_notifyEnabled = false;

// Parser state is owned by taskImuReader.
static struct {
    uint8_t buf[11];
    uint8_t idx;
    size_t baudIdx;
    uint32_t lastValidMs;
    uint32_t lastSnapshotMs;
    bool hasAcc;
    bool hasGyro;
    ImuFrame frame;
} g_imu;

static inline int16_t i16le(const uint8_t *b, int i)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(b[i]) |
        (static_cast<uint16_t>(b[i + 1]) << 8));
}

static uint32_t imuAgeMs()
{
    const uint32_t last = g_imu.lastValidMs;
    return last == 0 ? UINT32_MAX : millis() - last;
}

static void updateStatusValue(BLECharacteristic *characteristic)
{
    if (characteristic == nullptr || g_events == nullptr) {
        return;
    }

    const EventBits_t bits = xEventGroupGetBits(g_events);
    char status[384];
    snprintf(
        status,
        sizeof(status),
        "{\"ble_connected\":%s,\"advertising\":%s,"
        "\"imu_ready\":%s,\"imu_timeout\":%s,"
        "\"last_valid_imu_age_ms\":%lu,\"seq\":%u,"
        "\"checksum_error_count\":%lu,\"queue_drop_count\":%lu,"
        "\"packets_sent_count\":%lu,\"notify_error_count\":%lu,"
        "\"mtu\":%u,\"mtu_ok\":%s,\"notify_enabled\":%s,"
        "\"quat_valid\":%s}",
        (bits & EVT_BLE_CONN) ? "true" : "false",
        g_advertising ? "true" : "false",
        (bits & EVT_IMU_READY) ? "true" : "false",
        (bits & EVT_IMU_TIMEOUT) ? "true" : "false",
        static_cast<unsigned long>(imuAgeMs()),
        static_cast<unsigned>(g_seq),
        static_cast<unsigned long>(g_checksumErrorCount),
        static_cast<unsigned long>(g_queueDropCount),
        static_cast<unsigned long>(g_packetsSentCount),
        static_cast<unsigned long>(g_notifyErrorCount),
        static_cast<unsigned>(g_peerMtu),
        g_peerMtu >= BLE_PACKET_MIN_MTU ? "true" : "false",
        g_notifyEnabled ? "true" : "false",
        g_quatValid ? "true" : "false");
    characteristic->setValue(status);
}

static void printImuSample(const ImuFrame &frame)
{
#if IMU_SERIAL_DEBUG
    Serial.printf(
        "{\"esp_ms\":%lu,\"imu_baud\":%lu,"
        "\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,"
        "\"gx\":%.5f,\"gy\":%.5f,\"gz\":%.5f,"
        "\"roll\":%.5f,\"pitch\":%.5f,\"yaw\":%.5f,"
        "\"q0\":%.6f,\"q1\":%.6f,\"q2\":%.6f,\"q3\":%.6f,"
        "\"quat_valid\":%s}\n",
        static_cast<unsigned long>(frame.esp_ms),
        static_cast<unsigned long>(kImuBauds[g_imu.baudIdx]),
        frame.ax, frame.ay, frame.az,
        frame.gx, frame.gy, frame.gz,
        frame.roll, frame.pitch, frame.yaw,
        frame.q0, frame.q1, frame.q2, frame.q3,
        g_quatValid ? "true" : "false");
#else
    (void)frame;
#endif
}

static void enqueueLatest(const ImuFrame &frame)
{
    if (xQueueSend(g_imuQ, &frame, 0) == pdTRUE) {
        return;
    }

    // Realtime policy: remove the oldest queued sample and retain the newest.
    ImuFrame discarded;
    if (xQueueReceive(g_imuQ, &discarded, 0) == pdTRUE) {
        g_queueDropCount++;
    }
    if (xQueueSend(g_imuQ, &frame, 0) != pdTRUE) {
        g_queueDropCount++;
    }
}

static void imuDecodeFrame(const uint8_t *b)
{
    switch (b[1]) {
    case 0x51:
        g_imu.frame.ax = i16le(b, 2) / 32768.0f * 16.0f * 9.80665f;
        g_imu.frame.ay = i16le(b, 4) / 32768.0f * 16.0f * 9.80665f;
        g_imu.frame.az = i16le(b, 6) / 32768.0f * 16.0f * 9.80665f;
        g_imu.hasAcc = true;
        break;
    case 0x52:
        g_imu.frame.gx = i16le(b, 2) / 32768.0f * 2000.0f;
        g_imu.frame.gy = i16le(b, 4) / 32768.0f * 2000.0f;
        g_imu.frame.gz = i16le(b, 6) / 32768.0f * 2000.0f;
        g_imu.hasGyro = true;
        break;
    case 0x53:
        g_imu.frame.roll = i16le(b, 2) / 32768.0f * 180.0f;
        g_imu.frame.pitch = i16le(b, 4) / 32768.0f * 180.0f;
        g_imu.frame.yaw = i16le(b, 6) / 32768.0f * 180.0f;
        break;
    case 0x59:
        g_imu.frame.q0 = i16le(b, 2) / 32768.0f;
        g_imu.frame.q1 = i16le(b, 4) / 32768.0f;
        g_imu.frame.q2 = i16le(b, 6) / 32768.0f;
        g_imu.frame.q3 = i16le(b, 8) / 32768.0f;
        g_quatValid = true;
        break;
    default:
        break;
    }
}

static void imuProcessByte(uint8_t byte)
{
    uint8_t *buf = g_imu.buf;
    uint8_t &idx = g_imu.idx;

    if (idx == 0) {
        if (byte == 0x55) {
            buf[idx++] = byte;
        }
        return;
    }

    buf[idx++] = byte;
    if (idx == 2) {
        if (buf[1] < 0x50 || buf[1] > 0x5A) {
            idx = byte == 0x55 ? 1 : 0;
            if (idx == 1) {
                buf[0] = 0x55;
            }
        }
        return;
    }
    if (idx < 11) {
        return;
    }

    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum = static_cast<uint8_t>(sum + buf[i]);
    }

    const uint8_t type = buf[1];
    const bool checksumOk = sum == buf[10];
    idx = 0;
    if (!checksumOk) {
        g_checksumErrorCount++;
        return;
    }

    const uint32_t now = millis();
    g_imu.lastValidMs = now;
    imuDecodeFrame(buf);

    // A snapshot uses fresh accel + gyro and the latest angle/quaternion values.
    // WT61PC quaternion frame 0x59 commonly follows angle frame 0x53, so the
    // quaternion may be from the preceding sensor cycle. quat_valid reports
    // whether any valid quaternion frame has been received since startup/loss.
    if (type == 0x53 && g_imu.hasAcc && g_imu.hasGyro) {
        const bool recovering =
            xEventGroupGetBits(g_events) & EVT_IMU_TIMEOUT;
        g_imu.frame.esp_ms = now;
        g_imu.lastSnapshotMs = now;
        g_imu.hasAcc = false;
        g_imu.hasGyro = false;
        xEventGroupClearBits(g_events, EVT_IMU_TIMEOUT);
        xEventGroupSetBits(g_events, EVT_IMU_READY);
        if (recovering) {
            Serial.printf(
                "{\"imu\":\"recovered\",\"baud\":%lu}\n",
                static_cast<unsigned long>(kImuBauds[g_imu.baudIdx]));
        }
        enqueueLatest(g_imu.frame);
        printImuSample(g_imu.frame);
    }
}

class StatusCallbacks : public BLECharacteristicCallbacks {
    void onRead(
        BLECharacteristic *characteristic,
        esp_ble_gatts_cb_param_t *) override
    {
        updateStatusValue(characteristic);
    }
};

class ImuNotifyCallbacks : public BLECharacteristicCallbacks {
    void onStatus(
        BLECharacteristic *,
        BLECharacteristicCallbacks::Status status,
        uint32_t code) override
    {
        if (status == BLECharacteristicCallbacks::SUCCESS_NOTIFY) {
            g_packetsSentCount++;
            return;
        }

        g_notifyErrorCount++;
        if (g_notifyErrorCount == 1 || (g_notifyErrorCount % 100) == 0) {
            Serial.printf(
                "{\"ble_notify_error\":%d,\"code\":%lu,\"count\":%lu}\n",
                static_cast<int>(status),
                static_cast<unsigned long>(code),
                static_cast<unsigned long>(g_notifyErrorCount));
        }
    }
};

class NotifyDescriptorCallbacks : public BLEDescriptorCallbacks {
    void onWrite(BLEDescriptor *) override
    {
        g_notifyEnabled =
            g_notifyDescriptor != nullptr &&
            g_notifyDescriptor->getNotifications();
        Serial.printf(
            "{\"ble\":\"notify_subscription\",\"enabled\":%s}\n",
            g_notifyEnabled ? "true" : "false");
    }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(
        BLEServer *,
        esp_ble_gatts_cb_param_t *param) override
    {
        g_peerMtu = 23;
        g_advertising = false;
        g_notifyEnabled = false;
        xEventGroupSetBits(g_events, EVT_BLE_CONN);
        Serial.printf(
            "{\"ble\":\"connected\",\"conn_id\":%u,\"mtu\":23}\n",
            static_cast<unsigned>(param->connect.conn_id));
    }

    void onDisconnect(
        BLEServer *server,
        esp_ble_gatts_cb_param_t *param) override
    {
        xEventGroupClearBits(g_events, EVT_BLE_CONN);
        g_peerMtu = 23;
        g_advertising = true;
        g_notifyEnabled = false;
        Serial.printf(
            "{\"ble\":\"disconnected\",\"reason\":%u}\n",
            static_cast<unsigned>(param->disconnect.reason));
        server->startAdvertising();
    }

    void onMtuChanged(
        BLEServer *,
        esp_ble_gatts_cb_param_t *param) override
    {
        g_peerMtu = param->mtu.mtu;
        Serial.printf(
            "{\"ble\":\"mtu_changed\",\"mtu\":%u,\"packet_mtu_ok\":%s}\n",
            static_cast<unsigned>(g_peerMtu),
            g_peerMtu >= BLE_PACKET_MIN_MTU ? "true" : "false");
    }
};

static BlePacket makePacket(const ImuFrame &frame)
{
    BlePacket packet = {};
    packet.magic[0] = 0xAA;
    packet.magic[1] = 0x55;
    packet.seq = g_seq++;
    packet.ax = frame.ax;
    packet.ay = frame.ay;
    packet.az = frame.az;
    packet.gx = frame.gx;
    packet.gy = frame.gy;
    packet.gz = frame.gz;
    packet.roll = frame.roll;
    packet.pitch = frame.pitch;
    packet.yaw = frame.yaw;
    packet.q0 = frame.q0;
    packet.q1 = frame.q1;
    packet.q2 = frame.q2;
    packet.q3 = frame.q3;
    packet.esp_ms = frame.esp_ms;

    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&packet);
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(BlePacket) - 1; i++) {
        checksum ^= raw[i];
    }
    packet.csum = checksum;
    return packet;
}

static void markImuTimedOut(uint32_t now)
{
    const EventBits_t bits = xEventGroupGetBits(g_events);
    const bool wasReady = bits & EVT_IMU_READY;
    if (!wasReady && (bits & EVT_IMU_TIMEOUT)) {
        return;
    }

    xEventGroupClearBits(g_events, EVT_IMU_READY);
    xEventGroupSetBits(g_events, EVT_IMU_TIMEOUT);
    g_imu.hasAcc = false;
    g_imu.hasGyro = false;
    g_quatValid = false;

    const UBaseType_t staleCount = uxQueueMessagesWaiting(g_imuQ);
    if (staleCount > 0) {
        g_queueDropCount += staleCount;
        xQueueReset(g_imuQ);
    }

    Serial.printf(
        "{\"imu\":\"timeout\",\"age_ms\":%lu}\n",
        static_cast<unsigned long>(now - g_imu.lastSnapshotMs));
}

static void taskImuReader(void *)
{
    uint32_t retryMs = millis();

    for (;;) {
        while (Serial2.available() > 0) {
            imuProcessByte(static_cast<uint8_t>(Serial2.read()));
        }

        const uint32_t now = millis();
        if (now - g_imu.lastSnapshotMs >= kImuTimeoutMs) {
            markImuTimedOut(now);
        }

        if (!(xEventGroupGetBits(g_events) & EVT_IMU_READY) &&
            now - retryMs >= kBaudRetryMs) {
            retryMs = now;
            g_imu.baudIdx = (g_imu.baudIdx + 1) % kImuBaudCnt;
            g_imu.idx = 0;
            Serial2.end();
            vTaskDelay(pdMS_TO_TICKS(20));
            Serial2.begin(
                kImuBauds[g_imu.baudIdx],
                SERIAL_8N1,
                IMU_RX_PIN,
                IMU_TX_PIN);
            Serial.printf(
                "{\"imu\":\"searching\",\"baud\":%lu}\n",
                static_cast<unsigned long>(kImuBauds[g_imu.baudIdx]));
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void taskBleTx(void *)
{
    ImuFrame frame = {};
    bool mtuWarningPrinted = false;

    for (;;) {
        if (xQueueReceive(g_imuQ, &frame, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        // Coalesce backlog to the newest sample before transmission.
        ImuFrame newer;
        while (xQueueReceive(g_imuQ, &newer, 0) == pdTRUE) {
            frame = newer;
            g_queueDropCount++;
        }

        const EventBits_t bits = xEventGroupGetBits(g_events);
        if (!(bits & EVT_BLE_CONN) ||
            !(bits & EVT_IMU_READY) ||
            !g_notifyEnabled) {
            continue;
        }

        if (g_peerMtu < BLE_PACKET_MIN_MTU) {
            if (!mtuWarningPrinted) {
                Serial.printf(
                    "{\"ble\":\"mtu_too_small\",\"mtu\":%u,"
                    "\"required\":%u}\n",
                    static_cast<unsigned>(g_peerMtu),
                    static_cast<unsigned>(BLE_PACKET_MIN_MTU));
                mtuWarningPrinted = true;
            }
            continue;
        }
        mtuWarningPrinted = false;

        BlePacket packet = makePacket(frame);
        g_charImu->setValue(
            reinterpret_cast<uint8_t *>(&packet),
            sizeof(packet));
        g_charImu->notify();
    }
}

// LED patterns:
//   connected + IMU ready: 900 ms on/off
//   connected + IMU lost:  300 ms on/off
//   advertising + ready:    100 ms on/off
//   searching/timeout:      double blink, then pause
static void taskLed(void *)
{
    for (;;) {
        const EventBits_t bits = xEventGroupGetBits(g_events);
        const bool connected = bits & EVT_BLE_CONN;
        const bool ready = bits & EVT_IMU_READY;

        if (connected && ready) {
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(900));
            digitalWrite(LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(900));
        } else if (connected) {
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(300));
            digitalWrite(LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(300));
        } else if (ready) {
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            for (int i = 0; i < 2; i++) {
                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(80));
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(80));
            }
            vTaskDelay(pdMS_TO_TICKS(700));
        }
    }
}

static void initBLE()
{
    BLEDevice::init(BLE_DEVICE_NAME);
    const esp_err_t mtuResult = BLEDevice::setMTU(BLE_LOCAL_MTU);
    Serial.printf(
        "{\"ble_local_mtu\":%u,\"result\":%d,"
        "\"edge_required_mtu\":%u}\n",
        static_cast<unsigned>(BLE_LOCAL_MTU),
        static_cast<int>(mtuResult),
        static_cast<unsigned>(BLE_PACKET_MIN_MTU));

    g_server = BLEDevice::createServer();
    g_server->setCallbacks(new ServerCallbacks());

    BLEService *service = g_server->createService(SERVICE_UUID);
    g_charImu = service->createCharacteristic(
        CHAR_IMU_UUID,
        BLECharacteristic::PROPERTY_NOTIFY);
    g_notifyDescriptor = new BLE2902();
    g_notifyDescriptor->setCallbacks(new NotifyDescriptorCallbacks());
    g_charImu->addDescriptor(g_notifyDescriptor);
    g_charImu->setCallbacks(new ImuNotifyCallbacks());

    g_charStat = service->createCharacteristic(
        CHAR_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ);
    g_charStat->setCallbacks(new StatusCallbacks());
    updateStatusValue(g_charStat);

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();
}

static bool createTasks()
{
    bool ok = true;
    ok &= xTaskCreatePinnedToCore(
              taskImuReader, "imu", 4096, nullptr, 3, &g_taskImu, 0) == pdPASS;
    ok &= xTaskCreatePinnedToCore(
              taskBleTx, "bletx", 8192, nullptr, 2, &g_taskBleTx, 1) == pdPASS;
    ok &= xTaskCreatePinnedToCore(
              taskLed, "led", 2048, nullptr, 1, &g_taskLed, 1) == pdPASS;
    return ok;
}

static void stopCreatedTasks()
{
    if (g_taskImu != nullptr) {
        vTaskDelete(g_taskImu);
        g_taskImu = nullptr;
    }
    if (g_taskBleTx != nullptr) {
        vTaskDelete(g_taskBleTx);
        g_taskBleTx = nullptr;
    }
    if (g_taskLed != nullptr) {
        vTaskDelete(g_taskLed);
        g_taskLed = nullptr;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("{\"status\":\"boot\"}");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    g_imuQ = xQueueCreate(IMU_Q_LEN, sizeof(ImuFrame));
    g_events = xEventGroupCreate();
    if (g_imuQ == nullptr || g_events == nullptr) {
        Serial.println("{\"fatal\":\"rtos_object_creation_failed\"}");
        for (;;) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    const uint32_t now = millis();
    g_imu.lastValidMs = 0;
    g_imu.lastSnapshotMs = now;
    Serial2.begin(
        kImuBauds[g_imu.baudIdx],
        SERIAL_8N1,
        IMU_RX_PIN,
        IMU_TX_PIN);
    Serial.printf(
        "{\"imu_baud_start\":%lu,\"serial_debug\":%s}\n",
        static_cast<unsigned long>(kImuBauds[g_imu.baudIdx]),
        IMU_SERIAL_DEBUG ? "true" : "false");

    initBLE();

    if (!createTasks()) {
        Serial.println("{\"fatal\":\"task_creation_failed\"}");
        stopCreatedTasks();
        for (;;) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    Serial.println("{\"status\":\"running\"}");
}

void loop()
{
    updateStatusValue(g_charStat);
    Serial.printf(
        "{\"diag\":\"health\",\"imu_stack_words\":%u,"
        "\"bletx_stack_words\":%u,\"led_stack_words\":%u,"
        "\"queue_depth\":%u,\"queue_drops\":%lu,"
        "\"packets_sent\":%lu,\"mtu\":%u}\n",
        static_cast<unsigned>(uxTaskGetStackHighWaterMark(g_taskImu)),
        static_cast<unsigned>(uxTaskGetStackHighWaterMark(g_taskBleTx)),
        static_cast<unsigned>(uxTaskGetStackHighWaterMark(g_taskLed)),
        static_cast<unsigned>(uxQueueMessagesWaiting(g_imuQ)),
        static_cast<unsigned long>(g_queueDropCount),
        static_cast<unsigned long>(g_packetsSentCount),
        static_cast<unsigned>(g_peerMtu));
    vTaskDelay(pdMS_TO_TICKS(5000));
}
