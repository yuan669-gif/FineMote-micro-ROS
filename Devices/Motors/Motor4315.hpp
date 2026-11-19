/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MOTOR4315_H
#define FINEMOTE_MOTOR4315_H

#include "Motors/MotorBase.hpp"
#include "Bus/RS485_Base.hpp"
#include "Verification/CRC.hpp"

inline constexpr size_t motor4315MapLength = 10;

template <uint8_t ID>
class Motor4315Proxy {
public:
    Motor4315Proxy(MotorBase* motor, uint8_t addr) {
        RS485_Base<ID>::GetInstance();
        getMotorMap().insert(etl::make_pair(addr, motor));
    }

    static void Transmit(uint8_t* data, size_t size) {
        static RS485_Agent<ID> rs485Agent(0x3C, [](uint8_t* data, size_t size) {
            Decode(data, size);
        });
        rs485Agent.Transmit(data, size);
    }

private:
    static etl::map<uint8_t, MotorBase*, motor4315MapLength>& getMotorMap() {
        static etl::map<uint8_t, MotorBase*, motor4315MapLength> instance;
        return instance;
    }

    static void Decode(uint8_t* data, size_t size) {
        if (size < 15) return;
        auto& motorMap = getMotorMap();
        if (CRCCalc<crc16_modbus>(data, 13u) == (data[13] | data[14] << 8u) && motorMap.contains(data[2])) {
            MotorBase* motor = motorMap[data[2]];
            if (data[3] == 0x55) {
                Motor_State_t newState;

                newState.position = -1 * ((data[7] | (data[8] << 8u) | (data[9] << 16u) | (data[10] << 24u)) * 360.0f / 16384.0f);
                newState.speed = -1 * static_cast<int16_t>(data[11] | (data[12] << 8u));
                newState.torque = 0;

                motor->CommitState(newState);
            }
        }
    }
};

template <uint8_t BusID>
class Motor4315 : public MotorBase {
public:
    template <typename T>
    Motor4315(const Motor_Param_t&& params, T& _controller, uint8_t addr, uint8_t divisionFactor=5)
            : MotorBase(std::forward<const Motor_Param_t>(params), divisionFactor), id(addr), commuAgent(this, addr) { // Todo: ID和地址分离逻辑
        ResetController(_controller);
    }

    void Handle() final {
        if (inhibited_.load()) {
            if (HasFeedback()) SendPosition(GetState().position);
            return;
        }
        SetFeedback();
        controller->Calc();
        MessageGenerate();
    }

private:
    uint8_t txbuf[11] = {};

    void SetFeedback() final {
        auto* s = &state;
        switch (this->params.ctrlType) {
            case Motor_Ctrl_Type_e::Position:
                controller->SetFeedbacks(&s->position);
                break;
        }
    }

    void MessageGenerate() {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Position: {
                ControllerOutputData output = controller->GetOutputs();
                SendPosition(output.dataPtr[0]);
                break;
            }
        }
    }

    void SendPosition(float degrees) {
        double counts = -static_cast<double>(degrees) * 16384.0 / 360.0;
        if (!(counts >= INT32_MIN && counts <= INT32_MAX)) return;
        int32_t txAngle = static_cast<int32_t>(counts);
        txbuf[0] = 0x3E;
        txbuf[1] = 0x00;
        txbuf[2] = id;
        txbuf[3] = 0x55;
        txbuf[4] = 0x04;
        txbuf[5] = txAngle;
        txbuf[6] = txAngle >> 8u;
        txbuf[7] = txAngle >> 16u;
        txbuf[8] = txAngle >> 24u;
        uint16_t crc16 = CRCCalc<crc16_modbus>(txbuf, 9u);
        txbuf[9] = crc16;
        txbuf[10] = crc16 >> 8u;
        commuAgent.Transmit(txbuf, 11);
    }

    const uint8_t id;
    Motor4315Proxy<BusID> commuAgent;
};

#endif
