/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/laikago_sdk.hpp"
#include <array>
#include <math.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace laikago;

class RobotInterface
{
public:
    RobotInterface() : control(LOWLEVEL), udp(LOW_CMD_LENGTH, LOW_STATE_LENGTH){
        // InitEnvironment();
    }
    LowState ReceiveObservation();
    void SendCommand(std::array<float, 60> motorcmd);
    void Initialize();

    UDP udp;
    Control control;
    LowState state = {0};
    LowCmd cmd = {0};
};

LowState RobotInterface::ReceiveObservation() {
    udp.Recv();
    udp.GetState(state);
    return state;
}

void RobotInterface::SendCommand(std::array<float, 60> motorcmd) {
    cmd.levelFlag = 0xff;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        cmd.motorCmd[motor_id].mode = 0x0A;
        cmd.motorCmd[motor_id].position = motorcmd[motor_id * 5];
        cmd.motorCmd[motor_id].positionStiffness = motorcmd[motor_id * 5 + 1];
        cmd.motorCmd[motor_id].velocity = motorcmd[motor_id * 5 + 2];
        cmd.motorCmd[motor_id].velocityStiffness = motorcmd[motor_id * 5 + 3];
        cmd.motorCmd[motor_id].torque = motorcmd[motor_id * 5 + 4];
    }
    safe.JointLimit(cmd);
    udp.Send(cmd);
}

namespace py = pybind11;

// TODO: Expose all of comm.h and the RobotInterface Class.

PYBIND11_MODULE(robot_interface, m) {
    m.doc() = R"pbdoc(
          Laikago Robot Interface Python Bindings
          -----------------------
          .. currentmodule:: laikago_robot_interface
          .. autosummary::
             :toctree: _generate
      )pbdoc";

    py::class_<Cartesian>(m, "Cartesian")
        .def(py::init<>())
        .def_readwrite("x", &Cartesian::x)
        .def_readwrite("y", &Cartesian::y)
        .def_readwrite("z", &Cartesian::z);

    py::class_<IMU>(m, "IMU")
        .def(py::init<>())
        .def_readwrite("quaternion", &IMU::quaternion)
        .def_readwrite("gyroscope", &IMU::gyroscope)
        .def_readwrite("accelerometer", &IMU::acceleration)
        .def_readwrite("rpy", &IMU::rpy)
        .def_readwrite("temperature", &IMU::temp);

    py::class_<LED>(m, "LED")
        .def(py::init<>())
        .def_readwrite("r", &LED::r)
        .def_readwrite("g", &LED::g)
        .def_readwrite("b", &LED::b);

    py::class_<MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("mode", &MotorState::mode)
        .def_readwrite("q", &MotorState::position)
        .def_readwrite("dq", &MotorState::velocity)
        .def_readwrite("tauEst", &MotorState::torque)
        .def_readwrite("temperature", &MotorState::temperature)
        .def_readwrite("freserve", &MotorState::fReserve)
        .def_readwrite("ireserve", &MotorState::iReserve);

    py::class_<MotorCmd>(m, "MotorCmd")
        .def(py::init<>())
        .def_readwrite("mode", &MotorCmd::mode)
        .def_readwrite("q", &MotorCmd::position)
        .def_readwrite("dq", &MotorCmd::velocity)
        .def_readwrite("tau", &MotorCmd::torque)
        .def_readwrite("Kp", &MotorCmd::positionStiffness)
        .def_readwrite("Kd", &MotorCmd::velocityStiffness);

    py::class_<LowState>(m, "LowState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &LowState::levelFlag)
        .def_readwrite("imu", &LowState::imu)
        .def_readwrite("motorState", &LowState::motorState)
        .def_readwrite("footForce", &LowState::footForce)
        .def_readwrite("tick", &LowState::tick)
        .def_readwrite("wirelessRemote", &LowState::wirelessRemote)
        .def_readwrite("crc", &LowState::crc);

    py::class_<LowCmd>(m, "LowCmd")
        .def(py::init<>())
        .def_readwrite("levelFlag", &LowCmd::levelFlag)
        .def_readwrite("motorCmd", &LowCmd::motorCmd)
        .def_readwrite("led", &LowCmd::led)
        .def_readwrite("wirelessRemote", &LowCmd::wirelessRemote)
        .def_readwrite("crc", &LowCmd::crc);

    py::class_<HighState>(m, "HighState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighState::levelFlag)
        .def_readwrite("mode", &HighState::mode)
        .def_readwrite("imu", &HighState::imu)
        .def_readwrite("forwardSpeed", &HighState::forwardSpeed)
        .def_readwrite("sideSpeed", &HighState::sideSpeed)
        .def_readwrite("rotateSpeed", &HighState::rotateSpeed)
        .def_readwrite("bodyHeight", &HighState::bodyHeight)
        .def_readwrite("updownSpeed", &HighState::updownSpeed)
        .def_readwrite("forwardPosition", &HighState::forwardPosition)
        .def_readwrite("sidePosition", &HighState::sidePosition)
        .def_readwrite("footPosition2Body", &HighState::footPosition2Body)
        .def_readwrite("footSpeed2Body", &HighState::footSpeed2Body)
        .def_readwrite("footForce", &HighState::footForce)
        .def_readwrite("tick", &HighState::tick)
        .def_readwrite("wirelessRemote", &HighState::wirelessRemote)
        .def_readwrite("crc", &HighState::crc);

    py::class_<HighCmd>(m, "HighCmd")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighCmd::levelFlag)
        .def_readwrite("mode", &HighCmd::mode)
        .def_readwrite("forwardSpeed", &HighCmd::forwardSpeed)
        .def_readwrite("sideSpeed", &HighCmd::sideSpeed)
        .def_readwrite("rotateSpeed", &HighCmd::rotateSpeed)
        .def_readwrite("bodyHeight", &HighCmd::bodyHeight)
        .def_readwrite("footRaiseHeight", &HighCmd::footRaiseHeight)
        .def_readwrite("yaw", &HighCmd::yaw)
        .def_readwrite("pitch", &HighCmd::pitch)
        .def_readwrite("roll", &HighCmd::roll)
        .def_readwrite("led", &HighCmd::led)
        .def_readwrite("wirelessRemote", &HighCmd::wirelessRemote)
        .def_readwrite("crc", &HighCmd::crc);

    py::class_<UDPState>(m, "UDPState")
        .def(py::init<>())
        .def_readwrite("TotalCount", &UDPState::TotalCount)
        .def_readwrite("SendCount", &UDPState::SendCount)
        .def_readwrite("RecvCount", &UDPState::RecvCount)
        .def_readwrite("SendError", &UDPState::SendError)
        .def_readwrite("FlagError", &UDPState::FlagError)
        .def_readwrite("RecvCRCError", &UDPState::RecvCRCError)
        .def_readwrite("RecvLoseError", &UDPState::RecvLoseError);

    py::class_<RobotInterface>(m, "RobotInterface")
        .def(py::init<>())
        .def("receive_observation", &RobotInterface::ReceiveObservation)
        .def("send_command", &RobotInterface::SendCommand);

    #ifdef VERSION_INFO
      m.attr("__version__") = VERSION_INFO;
    #else
      m.attr("__version__") = "dev";
    #endif

      m.attr("TEST") = py::int_(int(42));

}
