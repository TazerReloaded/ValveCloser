#include "main.h"

void setup() {
    // set CPU clock
    HAL_Init();
    SystemInit();
    SystemClock_Config_HSE_8M_SYSCLK_72M();

    // initialize motor pins
    pinMode(COIL_A_DIR_1_PIN, OUTPUT);
    pinMode(COIL_A_DIR_2_PIN, OUTPUT);
    pinMode(COIL_B_DIR_1_PIN, OUTPUT);
    pinMode(COIL_B_DIR_2_PIN, OUTPUT);

    // initialize serial port
    Serial.setTx(USART1_TX);
    Serial.setRx(USART1_RX);
    Serial.begin(SERIAL_BAUD);

    // initialize PWM outputs and start timer
    mPWMTimer.setMode(mChannelA, TIMER_OUTPUT_COMPARE_PWM1, COIL_A_POWER_OUTPUT_PIN);
    mPWMTimer.setMode(mChannelB, TIMER_OUTPUT_COMPARE_PWM1, COIL_B_POWER_OUTPUT_PIN);
    mPWMTimer.setOverflow(PWM_FREQ, HERTZ_FORMAT);
    mPWMTimer.setCaptureCompare(mChannelA, 0, PWM_COMPARE_FORMAT);
    mPWMTimer.setCaptureCompare(mChannelB, 0, PWM_COMPARE_FORMAT);
    mPWMTimer.resume();

    // initialize magnetic angle sensor
    TLE5012B_init();
    if (mStatus != ControlStatus::NO_ERROR) {
        // could not configure angle sensor, HALT and output error code
        while (true) {
            Serial.write(uint8_t(mStatus));
            delay(1000);
        }
    }
}

void loop() {
    if (micros() - mLastStep > STEP_INTERVAL) {
        // execute pending step if interval elapsed
        mLastStep = micros();
        step();
    }

    if (micros() - mLastSensorRead > ANGLE_INTERVAL) {
        // read encoder angle
        mLastSensorRead = micros();
        uint16_t data;
        TLE5012B_read(ENCODER_ANGLE_REG, &data);
        mOutputAngle = (360.0f / POW_2_15) * (float)(data & KEEP_15_BITS);
    }

    if (mDriving && millis() - mLastStallCheck > STALL_INTERVAL) {
        // check if motor moves while it should (stall check)
        mLastStallCheck = millis();
        if (mDeviation - abs(mTargetAngle - mOutputAngle) < STALL_MIN_ANGLE) {
            // we didn't move more than the threshold since last check, something is binding
            mStatus = ControlStatus::STALL_ERROR;
        }
        mDeviation = abs(mTargetAngle - mOutputAngle);
        // check board temperature (overtemp protection)
        uint16_t data;
        TLE5012B_read(ENCODER_TEMP_REG, &data);
        mSensorTemp = ((data & KEEP_8_BITS) - (data & (1 << 8)) + TEMP_OFFSET) / TEMP_DIV;
        if (mSensorTemp > MAX_TEMPERATURE) {
            // sensor temperature exceeds threshold, initiate shutdown
            mStatus = ControlStatus::OVERTEMP_ERROR;
        }
    }

    while (Serial.available()) {
        // read incoming bytes from serial port
        int incoming = Serial.read();
        if (incoming >= 0 && incoming <= 100) {
            // set target angle and deviation from wanted percentage
            mTargetAngle = 190.0f - (incoming * 0.8f);
            mDeviation = abs(mTargetAngle - mOutputAngle);
            mLastStallCheck = millis();
            // execute one step to set status byte
            mLastStep = micros();
            step();
            // return status
            if (mDriving) {
                Serial.write(uint8_t(mStatus) | uint8_t(1 << 7));
            } else {
                Serial.write(uint8_t(mStatus));
            }
        }
    }

#if DEBUG_PRINT
    if (millis() - mLastPrint > 1000) {
        mLastPrint = millis();
        Serial.printf(F("Angle: %.2f°, Temperature: %.2f°C, Status: "), mOutputAngle, mSensorTemp);
        switch (mStatus) {
        case ControlStatus::NO_ERROR:
            Serial.print(F("OK"));
            break;
        case ControlStatus::SYSTEM_ERROR:
            Serial.print(F("SYSTEM_ERROR"));
            break;
        case ControlStatus::INTERFACE_ACCESS_ERROR:
            Serial.print(F("INTERFACE_ACCESS_ERROR"));
            break;
        case ControlStatus::INVALID_ANGLE_ERROR:
            Serial.print(F("INVALID_ANGLE_ERROR"));
            break;
        case ControlStatus::STALL_ERROR:
            Serial.print(F("STALL_ERROR"));
            break;
        case ControlStatus::CRC_ERROR:
            Serial.print(F("CRC_ERROR"));
            break;
        case ControlStatus::OVERTEMP_ERROR:
            Serial.print(F("OVERTEMP_ERROR"));
            break;
        default:
            Serial.print(F("UNKNOWN"));
            break;
        }
        Serial.println();
    }
#endif
}

void step() {
    if (mTargetAngle < 0 || mStatus != ControlStatus::NO_ERROR) {
        // angle not yet configured or pending error, disable motor
        digitalWrite(COIL_A_DIR_1_PIN, LOW);
        digitalWrite(COIL_A_DIR_2_PIN, LOW);
        digitalWrite(COIL_B_DIR_1_PIN, LOW);
        digitalWrite(COIL_B_DIR_2_PIN, LOW);
        mDriving = false;
    } else if (mOutputAngle > mTargetAngle + 1) {
        // drive towards target CW
        output(++mMotorPosition, MOTOR_CURRENT);
        mSlackCompensation = MICROSTEPS;
        mDriving = true;
    } else if (mOutputAngle < mTargetAngle - 1) {
        // drive towards target CCW
        output(--mMotorPosition, MOTOR_CURRENT);
        mSlackCompensation = -MICROSTEPS;
        mDriving = true;
    } else if (mSlackCompensation > 0) {
        // drive a little more to compensate gear slack (CW)
        output(++mMotorPosition, MOTOR_CURRENT);
        mSlackCompensation--;
        mDriving = true;
    } else if (mSlackCompensation < 0) {
        // drive a little more to compensate gear slack (CCW)
        output(--mMotorPosition, MOTOR_CURRENT);
        mSlackCompensation++;
        mDriving = true;
    } else {
        // target angle reached, disable motor
        digitalWrite(COIL_A_DIR_1_PIN, LOW);
        digitalWrite(COIL_A_DIR_2_PIN, LOW);
        digitalWrite(COIL_B_DIR_1_PIN, LOW);
        digitalWrite(COIL_B_DIR_2_PIN, LOW);
        mDriving = false;
    }
}

void output(int32_t theta, uint16_t current) {
    uint16_t effort = currentToPWM(current);
    int16_t angle1 = modulo((4096 / MICROSTEPS) * theta, 4096);
    int16_t angle2 = angle1 + 1024;
    if (angle2 > 4096) {
        angle2 -= 4096;
    }
    int16_t coilA = effort * mSineLookup[angle1] / 1024;
    int16_t coilB = effort * mSineLookup[angle2] / 1024;
    if (coilA >= 0) {
        mPWMTimer.setCaptureCompare(mChannelA, coilA, PWM_COMPARE_FORMAT);
        digitalWrite(COIL_A_DIR_1_PIN, HIGH);
        digitalWrite(COIL_A_DIR_2_PIN, LOW);
    } else  {
        mPWMTimer.setCaptureCompare(mChannelA, -coilA, PWM_COMPARE_FORMAT);
        digitalWrite(COIL_A_DIR_1_PIN, LOW);
        digitalWrite(COIL_A_DIR_2_PIN, HIGH);
    }
    if (coilB >= 0) {
        mPWMTimer.setCaptureCompare(mChannelB, coilB, PWM_COMPARE_FORMAT);
        digitalWrite(COIL_B_DIR_1_PIN, HIGH);
        digitalWrite(COIL_B_DIR_2_PIN, LOW);
    } else {
        mPWMTimer.setCaptureCompare(mChannelB, -coilB, PWM_COMPARE_FORMAT);
        digitalWrite(COIL_B_DIR_1_PIN, LOW);
        digitalWrite(COIL_B_DIR_2_PIN, HIGH);
    }
}

int16_t modulo(int32_t x, int16_t m) {
    int16_t t = x % m;
    return t < 0 ? t + m : t;
}

uint16_t currentToPWM(uint16_t current) {
    return constrain((CURRENT_SENSE_RESISTOR * PWM_MAX_VALUE * current) / (BOARD_VOLTAGE * 100), 0, PWM_MAX_VALUE);
}

void SystemClock_Config_HSE_8M_SYSCLK_72M(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
    SystemCoreClockUpdate();
}

// initialize TLE5012B
void TLE5012B_init() {
    // set the chip select pin high, disabling the encoder's communication
    pinMode(ENCODER_CS_PIN, OUTPUT);
    digitalWrite(ENCODER_CS_PIN, HIGH);

    // snitialize communication pins as push/pull
    GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // enable the clock for the SPI bus
    __HAL_RCC_SPI1_CLK_ENABLE();

    // set the peripheral to be used
    mSPIConfig.Instance = SPI1;

    // configure the settings for transactions
    mSPIConfig.Init.Direction = SPI_DIRECTION_2LINES;
    mSPIConfig.Init.Mode = SPI_MODE_MASTER;
    mSPIConfig.Init.DataSize = SPI_DATASIZE_8BIT;
    mSPIConfig.Init.CLKPolarity = SPI_POLARITY_LOW;
    mSPIConfig.Init.CLKPhase = SPI_PHASE_2EDGE;
    mSPIConfig.Init.NSS = SPI_NSS_SOFT;
    mSPIConfig.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    mSPIConfig.Init.FirstBit = SPI_FIRSTBIT_MSB;
    mSPIConfig.Init.CRCPolynomial = 7;

    // initialize the SPI bus with the parameters we set
    if (HAL_SPI_Init(&mSPIConfig) != HAL_OK) {
        mStatus = ControlStatus::SYSTEM_ERROR;
    }
}

// read sensor register
void TLE5012B_read(uint16_t address, uint16_t* data) {
    // convert register address by adding read command and data length
    address |= ENCODER_READ_COMMAND | 0x01;
    // set up buffer, 2 bytes command, 2 bytes data, 2 bytes safety word
    uint8_t buf[] = { uint8_t(address >> 8), uint8_t(address), 0xFF, 0xFF, 0xFF, 0xFF };

    // disable interrupts, pull CS low to select encoder
    noInterrupts();
    digitalWrite(ENCODER_CS_PIN, LOW);

    // send address where we want to read, response seems to be equal to request
    HAL_SPI_TransmitReceive(&mSPIConfig, buf, buf, 2, 10);

    // set the MOSI pin to open drain
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // send 0xFFs (like BTT code), this returns the data and overwrites the response buffer
    HAL_SPI_TransmitReceive(&mSPIConfig, &buf[2], &buf[2], 4, 10);

    // set MOSI back to Push/Pull
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // deselect encoder, enable interrupts
    digitalWrite(ENCODER_CS_PIN, HIGH);
    interrupts();

    // write the received data into output pointer
    *data = buf[2] << 8 | buf[3];

    // check CRC and safety bits
    if (buf[5] != TLE5012B_CRC(buf, 4)) {
        mStatus = ControlStatus::CRC_ERROR;
    } else if ((buf[4] & (1 << 6)) == 0) {
        mStatus = ControlStatus::SYSTEM_ERROR;
    } else if ((buf[4] & (1 << 5)) == 0) {
        mStatus = ControlStatus::INTERFACE_ACCESS_ERROR;
    } else if ((buf[4] & (1 << 4)) == 0) {
        mStatus = ControlStatus::INVALID_ANGLE_ERROR;
    }
}

uint8_t TLE5012B_CRC(uint8_t *message, uint8_t length) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc = mCRCLookup[crc ^ message[i]];
    }
    return (~crc) & 0xFF;
}
