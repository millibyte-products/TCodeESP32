/* MIT License

Copyright (c) 2024 Millibyte LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "SettingsHandler.h"
#include "utils.h"
#include "enum.h"
#include "TagHandler.h"
#include "LogHandler.h"


using POWER_STATE_FUNCTION_PTR_T = void (*)(float vbus, float vservo);
/** Power Delivery configuration handler for CH224K
 */
class PowerHandler {
        public:
         PowerHandler() {
            message_callback = nullptr;
         }

         void requestPDLevel(int8_t setpoint) {
             switch ((PDLevels)setpoint) {
                 case PDLevels::PD20V:
                     writeMatrix({0, 1, 0});
                     break;
                 case PDLevels::PD15V:
                     writeMatrix({0, 1, 1});
                     break;
                 case PDLevels::PD12V:
                     writeMatrix({0, 0, 1});
                     break;
                 case PDLevels::PD9V:
                     writeMatrix({0, 0, 0});
                     break;
                 default:
                     setpoint = (int8_t)PDLevels::PD5V;
                 case PDLevels::PD5V:
                     writeMatrix({1, 0, 0});
                     break;
             }
             m_pd_setpoint = setpoint;
         }

         void setServoPowerEnable(bool enable) {
            if (enable && m_vservo_enable_pin != -1) {
                digitalWrite(m_vservo_enable_pin, HIGH);
            } else {
                digitalWrite(m_vservo_enable_pin, LOW);
            }
         }

         static void startLoop(void* parameter) {
             ((PowerHandler*)parameter)->loop();
         }

         bool setup() {
             LogHandler::info(_TAG, "Setup PD Handler");
             long timeout = millis() + 10000;
             if (m_initialized) {
                 return true;
             }
             SettingsFactory* settingsFactory = SettingsFactory::getInstance();
             m_pd_setpoint = (int8_t)PDLevels::PDDEFAULT;
             // TODO: Handle vservo/vbus separately
             m_vbus_fb_pin = BUS_VOLTAGE_PIN_DEFAULT;
             m_vservo_fb_pin = SERVO_VOLTAGE_PIN_DEFAULT;
             m_vservo_coeff = SERVO_VOLTAGE_COEFFICIENT_DEFAULT;
             m_vbus_coeff = BUS_VOLTAGE_COEFFICIENT_DEFAULT;
             m_vservo_enable_pin = SERVO_POWER_ENABLE_PIN_DEFAULT;
             settingsFactory->getValue(PD_REQUESTED_VOLTAGE, m_pd_setpoint);
             settingsFactory->getValue(BUS_VOLTAGE_PIN, m_vbus_fb_pin);
             settingsFactory->getValue(SERVO_VOLTAGE_PIN, m_vservo_fb_pin);
             settingsFactory->getValue(SERVO_VOLTAGE_COEFFICIENT,
                                       m_vservo_coeff);
             settingsFactory->getValue(BUS_VOLTAGE_COEFFICIENT, m_vbus_coeff);
             settingsFactory->getValue(SERVO_POWER_ENABLE_PIN,
                                       m_vservo_enable_pin);
             settingsFactory->getValueVector(PD_CFG_PINS, m_cfg_pins);
             if (m_cfg_pins.size() < 3) {
                 LogHandler::error(
                     _TAG, "Invalid cfg pin configuration, expected 3 pins");
             }
             for (auto idx : m_cfg_pins) {
                 pinMode(idx, OUTPUT);
             }
             if (m_vservo_enable_pin != -1) {
                 pinMode(m_vservo_enable_pin, OUTPUT);
             }
             requestPDLevel(m_pd_setpoint);
             setServoPowerEnable(false);
             LogHandler::debug(_TAG, "Complete");
             return true;
         }

         void setMessageCallback(POWER_STATE_FUNCTION_PTR_T f) {
            message_callback = f;
         }

         void loop() {
             _isRunning = true;
             LogHandler::debug(_TAG, "Power Handler task cpu core: %u",
                               xPortGetCoreID());
             TickType_t pxPreviousWakeTime = millis();
             while (_isRunning) {
                 if (millis() >= lastTick) {
                     LogHandler::verbose(_TAG, "Enter getVoltageLevel");
                     lastTick = millis() + UPDATE_TASK_PERIOD_MS;

                     if (m_vbus_fb_pin != -1) {
                        m_vbus = ((float)analogRead(m_vbus_fb_pin)) * m_vbus_coeff;
                     } else {
                        m_vbus = -1.0f;
                     }
                     if (m_vservo_fb_pin != -1) {
                        m_vservo = ((float)analogRead(m_vservo_fb_pin)) * m_vservo_coeff;
                     } else {
                        m_vservo = -1.0f;
                     }

                     LogHandler::verbose(_TAG, "Bus Voltage: %f",
                                         m_vbus);
                     LogHandler::verbose(_TAG, "Servo voltage: %f",
                                         m_vservo);
                     if (message_callback)
                         message_callback(m_vbus, m_vservo);
                 }
                 xTaskDelayUntil(&pxPreviousWakeTime,
                                 PowerHandler::UPDATE_TASK_PERIOD_MS / portTICK_PERIOD_MS);
             }

             vTaskDelete(NULL);
         }

        private:
         static constexpr uint32_t UPDATE_TASK_PERIOD_MS = 5000;
         void writeMatrix(std::initializer_list<int> v) {
            std::vector<int> values(v);
             if (values.size() < 3) {
                return;
             }
            digitalWrite(m_cfg_pins[0], values[0]);
            digitalWrite(m_cfg_pins[1], values[1]);
            digitalWrite(m_cfg_pins[2], values[2]);
         }

         static const char* _TAG;
         unsigned long lastTick = 0;
         bool _isRunning;
         bool m_initialized;
         int8_t m_pd_setpoint;
         float m_vbus_coeff;
         float m_vbus;
         int8_t m_vbus_fb_pin;
         float m_vservo_coeff;
         float m_vservo;
         int8_t m_vservo_fb_pin;
         int8_t m_vservo_enable_pin;
         std::vector<int> m_cfg_pins;
         POWER_STATE_FUNCTION_PTR_T message_callback;
     };

const char* PowerHandler::_TAG = TagHandler::PowerHandler;
