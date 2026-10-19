/* USER CODE BEGIN Includes */
#include "main.h"
#include <math.h>

// Nhúng toàn bộ 4 mảnh ghép vào main:
#include "dwt_stm32.h"
#include "icm42688.h"
#include "adaptive_kalman.h"
#include "telemetry_protocol.h"
/* USER CODE END Includes */

/* Khai báo biến ngoại vi từ STM32CubeMX tạo ra */
extern SPI_HandleTypeDef hspi1;  // SPI nối với ICM-42688
extern UART_HandleTypeDef huart1;// UART nối với ESP32
extern DMA_HandleTypeDef hdma_usart1_tx;

#define RAD_TO_DEG 57.29577951308232f

/* Đối tượng cấu trúc dữ liệu */
ICM42688_t        imu;
IMUData_t         imu_raw;
AdaptiveKalman_t  kfRoll, kfPitch;
TelemetryPacket_t tx_packet;
TelemetryPacket_t dma_tx_buffer;

volatile uint8_t  dma_busy = 0;
uint32_t          packet_seq = 0;
uint32_t          last_us = 0;

/* Callback khi DMA gửi xong gói tin UART sang ESP32 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        dma_busy = 0; // Đánh dấu đã gửi xong, sẵn sàng cho gói tiếp theo
    }
}

/* Đánh giá sự cố / Tai nạn */
CrashStatus_e Detect_Accident(float roll, float pitch, float acc_mag, float gyro_mag) {
    static uint32_t fall_timer = 0;

    // 1. Va chạm trực diện cực mạnh (> 4.5G)
    if (acc_mag > 4.5f) return CRASH_STATE_COLLISION;

    // 2. Xe bị ngã nghiêng quá 55 độ và nằm bất động
    if ((fabsf(roll) > 55.0f || fabsf(pitch) > 60.0f) && (gyro_mag < 15.0f)) {
        if (fall_timer == 0) fall_timer = HAL_GetTick();
        else if (HAL_GetTick() - fall_timer > 1500) return CRASH_STATE_FALLEN; // Nằm yên quá 1.5s
    } else {
        fall_timer = 0;
    }

    return CRASH_STATE_NORMAL;
}

int main(void) {
    HAL_Init();
    SystemClock_Config(); // Chạy 84MHz

    // Khởi tạo ngoại vi do CubeMX sinh ra (GPIO, SPI1, USART1, DMA)
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();

    // 1. Bật đồng hồ siêu chính xác DWT
    DWT_Init();

    // 2. Bật & Cấu hình bộ lọc phần cứng AAF cho ICM-42688 (Chân CS là PA4)
    if (!ICM42688_Init(&imu, &hspi1, GPIOA, GPIO_PIN_4)) {
        // Nếu không tìm thấy cảm biến, treo máy và nháy đèn báo lỗi
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(100);
        }
    }

    // 3. Khởi tạo thuật toán Adaptive Kalman
    AKF_Init(&kfRoll, 0.001f, 0.003f, 0.05f);
    AKF_Init(&kfPitch, 0.001f, 0.003f, 0.05f);

    // Chuẩn bị Header cố định cho gói tin
    tx_packet.magic_1 = SYNC_BYTE_1; // 0xAA
    tx_packet.magic_2 = SYNC_BYTE_2; // 0x55

    last_us = DWT_GetMicroseconds();

    /* VÒNG LẶP CHÍNH */
    while (1) {
        uint32_t current_us = DWT_GetMicroseconds();

        // Chu kỳ lấy mẫu cố định 100Hz = 10,000 Microseconds (10ms)
        if (current_us - last_us >= 10000) {
            float dt = (float)(current_us - last_us) / 1000000.0f;
            last_us = current_us;

            // BƯỚC 1: Lấy dữ liệu đã lọc phần cứng từ ICM-42688
            ICM42688_ReadData(&imu, &imu_raw);

            // BƯỚC 2: Tính độ lớn vector gia tốc và con quay hồi chuyển
            float acc_mag = sqrtf(imu_raw.ax * imu_raw.ax + 
                                  imu_raw.ay * imu_raw.ay + 
                                  imu_raw.az * imu_raw.az);
            float gyro_mag = sqrtf(imu_raw.gx * imu_raw.gx + 
                                   imu_raw.gy * imu_raw.gy + 
                                   imu_raw.gz * imu_raw.gz);

            // BƯỚC 3: Tính góc sơ bộ từ trọng lực
            float roll_acc  = atan2f(imu_raw.ay, imu_raw.az + 1e-6f) * RAD_TO_DEG;
            float pitch_acc = atan2f(-imu_raw.ax, sqrtf(imu_raw.ay * imu_raw.ay + imu_raw.az * imu_raw.az) + 1e-6f) * RAD_TO_DEG;

            // BƯỚC 4: Lọc thích ứng qua Adaptive Kalman (Loại trừ quán tính khi rẽ/phanh)
            float roll_clean  = AKF_Update(&kfRoll, roll_acc, imu_raw.gx, dt, acc_mag);
            float pitch_clean = AKF_Update(&kfPitch, pitch_acc, imu_raw.gy, dt, acc_mag);

            // BƯỚC 5: Đánh giá trạng thái ngã/tai nạn
            CrashStatus_e status = Detect_Accident(roll_clean, pitch_clean, acc_mag, gyro_mag);

            // BƯỚC 6: Đóng gói dữ liệu Telemetry
            tx_packet.seq_num       = ++packet_seq;
            tx_packet.timestamp_us  = current_us;
            tx_packet.accel[0]      = imu_raw.ax;
            tx_packet.accel[1]      = imu_raw.ay;
            tx_packet.accel[2]      = imu_raw.az;
            tx_packet.gyro[0]       = imu_raw.gx;
            tx_packet.gyro[1]       = imu_raw.gy;
            tx_packet.gyro[2]       = imu_raw.gz;
            tx_packet.roll          = roll_clean;
            tx_packet.pitch         = pitch_clean;
            tx_packet.crash_state   = (uint8_t)status;

            // Tính checksum chống nhiễu trên đường truyền UART
            tx_packet.crc16 = Calculate_CRC16((uint8_t*)&tx_packet, sizeof(TelemetryPacket_t) - 2);

            // BƯỚC 7: Đẩy sang ESP32 qua DMA (Không đứng chờ, CPU tiếp tục chạy ngay)
            if (!dma_busy) {
                dma_busy = 1;
                dma_tx_buffer = tx_packet; // Copy dữ liệu an toàn (Atomic snapshot)
                HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&dma_tx_buffer, sizeof(TelemetryPacket_t));
            }
        }
    }
}