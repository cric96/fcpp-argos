// FCPP <-> ARGoS glue: each foot-bot controller runs exactly one FCPP node.
//
// Every ControlStep():
//   range-and-bearing readings -> FCPP messages (range = nbr_dist, range+bearing = nbr_vec)
//   sensors -> node storage inputs  (source, heading, obstacle)
//   one aggregate round
//   FCPP export -> range-and-bearing payload, storage outputs (velocity, led, label) -> actuators
//
// Payload layout: [u32 sender uid][u16 export length][export bytes][zero padding up to rab_data_size].

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include "lib/beautify.hpp"
#include "lib/coordination.hpp"
#include "lib/data.hpp"
#include "lib/deployment.hpp"

namespace fcpp {

namespace coordination::tags {
    struct source {};   //!< in: whether this robot is the source (`uid == <params source="..."/>`)
    struct heading {};  //!< in: unit vector of the robot heading (global frame, from the compass)
    struct obstacle {}; //!< in: repulsion from close obstacles (global frame, from the proximity sensor)
    struct velocity {}; //!< out: desired velocity in m/s (global frame), clamped to max_speed
    struct led {};      //!< out: colour of the robot LEDs
    struct label {};    //!< out: text drawn above the robot in the GUI
}

//! @brief Connector handing messages to/from the controller instead of a real radio.
struct argos_connector {
    template <typename N>
    class network {
      public:
        struct data_type {
            std::vector<char> out;         //!< last export serialised by the node
            std::vector<message_type> in;  //!< messages received since the last round
        };

        template <typename S, typename T>
        network(N&, common::tagged_tuple<S,T> const&) {}

        data_type& data() { return m_data; }
        data_type const& data() const { return m_data; }

        void send(std::vector<char> m) { m_data.out = std::move(m); }
        std::vector<message_type> receive() { return std::exchange(m_data.in, {}); }

      private:
        data_type m_data;
    };
};

namespace component {

//! @brief Positioner providing `nbr_vec()` from range-and-bearing readings (no GPS needed).
template <class... Ts>
struct rab_positioner {
    template <typename F, typename P>
    struct component : public P {
        DECLARE_COMPONENT(positioner);
        REQUIRE_COMPONENT(positioner, calculus);

        class node : public P::node {
          public:
            using position_type = vec<2>;

            template <typename S, typename T>
            node(typename F::net& n, common::tagged_tuple<S,T> const& t) : P::node(n,t), m_nbr_vec(vec<2>{}) {}

            //! @brief Sets the vector towards a neighbour (call before the round).
            void nbr_vec(device_t d, vec<2> v) { fcpp::details::self(m_nbr_vec, d) = v; }

            //! @brief Vectors towards neighbours (global frame, metres).
            field<vec<2>> const& nbr_vec() const { return m_nbr_vec; }

            void round_start(times_t t) {
                P::node::round_start(t);
                align_inplace(m_nbr_vec, std::vector<device_t>(fcpp::details::get_ids(P::node::nbr_uid())));
            }

          private:
            field<vec<2>> m_nbr_vec;
        };

        using net = typename P::net;
    };
};

//! @brief `component::deployment` without logger/persister, plus the range-and-bearing positioner.
DECLARE_COMBINE(argos_deployment, rab_positioner, storage, hardware_connector, timer, hardware_identifier, randomizer, calculus, base);

}

//! @brief Network (holding a single node) running aggregate program `M` with export list `E`.
template <typename M, typename E>
using argos_net = typename component::argos_deployment<
    component::tags::program<M>,
    component::tags::exports<E>,
    component::tags::connector<argos_connector>,
    component::tags::message_push<false>,
    component::tags::parallel<false>,
    component::tags::round_schedule<sequence::periodic_n<1, 0, 1>>, // one round per ARGoS tick
    component::tags::tuple_store<
        coordination::tags::source,   bool,
        coordination::tags::heading,  vec<2>,
        coordination::tags::obstacle, vec<2>,
        coordination::tags::velocity, vec<2>,
        coordination::tags::led,      color,
        coordination::tags::label,    std::string
    >
>::net;

}

//! @brief Non-template base, so that the GUI can draw the state of any FCPP controller.
class CFcppController : public argos::CCI_Controller {
  public:
    std::string Label;
    argos::CColor Color;
};

//! @brief Foot-bot controller running aggregate program `M` (with export list `E`).
template <typename M, typename E>
class TFcppController : public CFcppController {
    using net_t = fcpp::argos_net<M, E>;
    using vec2 = fcpp::vec<2>;

  public:
    void Init(argos::TConfigurationNode& t_node) override {
        using namespace argos;
        m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
        m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
        m_pcRABSens = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
        m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
        m_pcProxSens = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
        GetNodeAttributeOrDefault(t_node, "source", m_nSource, m_nSource);
        GetNodeAttributeOrDefault(t_node, "max_speed", m_fMaxSpeed, m_fMaxSpeed);
        // "fb12" -> 12
        m_unUid = std::stoul(GetId().substr(GetId().find_first_of("0123456789")));
        Reset();
    }

    void Reset() override {
        m_unTick = 0;
        m_pcNet = std::make_unique<net_t>(fcpp::common::make_tagged_tuple<fcpp::component::tags::uid>(m_unUid));
    }

    void ControlStep() override {
        using namespace argos;
        using namespace fcpp::coordination;
        typename net_t::lock_type l;
        auto& node = m_pcNet->node_at(m_unUid, l);
        // compass: the only use of the positioning sensor
        CRadians cYaw, cPitch, cRoll;
        m_pcPosSens->GetReading().Orientation.ToEulerAngles(cYaw, cPitch, cRoll);
        // neighbour messages
        for (auto const& sPacket : m_pcRABSens->GetReadings()) {
            CByteArray cData(sPacket.Data);
            UInt32 unSender;
            UInt16 unLen;
            cData >> unSender >> unLen;
            if (unLen == 0 or unLen > cData.Size()) continue;
            char const* p = reinterpret_cast<char const*>(cData.ToCArray());
            Real fRange = sPacket.Range / 100; // cm -> m
            node.connector_data().in.push_back({fcpp::times_t(m_unTick), fcpp::device_t(unSender), fRange, {p, p + unLen}});
            CRadians cAngle = sPacket.HorizontalBearing + cYaw;
            node.nbr_vec(unSender, vec2{fRange * Cos(cAngle), fRange * Sin(cAngle)});
        }
        // sensor inputs
        CVector2 cObstacle;
        for (auto const& sReading : m_pcProxSens->GetReadings())
            cObstacle -= CVector2(sReading.Value, sReading.Angle + cYaw);
        node.storage(tags::source{}) = m_nSource == SInt32(m_unUid);
        node.storage(tags::heading{}) = vec2{Cos(cYaw), Sin(cYaw)};
        node.storage(tags::obstacle{}) = vec2{cObstacle.GetX(), cObstacle.GetY()};
        // aggregate round at time m_unTick, followed by the send event
        m_pcNet->run(m_unTick + 0.5);
        ++m_unTick;
        // outgoing message
        std::vector<char> const& cExport = node.connector_data().out;
        CByteArray cMsg;
        if (cExport.size() + 6 <= m_pcRABAct->GetSize()) {
            cMsg << UInt32(m_unUid) << UInt16(cExport.size());
            cMsg.AddBuffer(reinterpret_cast<UInt8 const*>(cExport.data()), cExport.size());
        } else LOGERR << GetId() << ": export of " << cExport.size() << " bytes exceeds rab_data_size" << std::endl;
        cMsg.Resize(m_pcRABAct->GetSize());
        m_pcRABAct->SetData(cMsg);
        // outputs
        fcpp::color const& c = node.storage(tags::led{});
        Color = CColor(c.red() * 255, c.green() * 255, c.blue() * 255);
        m_pcLEDs->SetAllColors(Color);
        Label = node.storage(tags::label{});
        vec2 v = node.storage(tags::velocity{});
        SetWheels(CVector2(v[0], v[1]).Rotate(-cYaw));
    }

    //! @brief Final label, so headless runs can be checked (see scripts/smoke.sh).
    void Destroy() override {
        argos::LOG << GetId() << " final: " << Label << std::endl;
        argos::LOG.Flush();
    }

  private:
    //! @brief Drives towards a velocity vector expressed in the robot frame (m/s).
    void SetWheels(argos::CVector2 const& c_vel) {
        using namespace argos;
        Real fSpeed = std::min(c_vel.Length(), m_fMaxSpeed) * 100; // m/s -> cm/s
        if (fSpeed < 0.5) {
            m_pcWheels->SetLinearVelocity(0, 0);
            return;
        }
        Real fAngle = c_vel.Angle().GetValue();                    // [-pi, pi], > 0 means turn left
        Real fForward = fSpeed * std::max<Real>(0, std::cos(fAngle));
        Real fTurn = fSpeed * fAngle;
        m_pcWheels->SetLinearVelocity(fForward - fTurn, fForward + fTurn);
    }

    argos::CCI_DifferentialSteeringActuator* m_pcWheels;
    argos::CCI_LEDsActuator* m_pcLEDs;
    argos::CCI_RangeAndBearingActuator* m_pcRABAct;
    argos::CCI_RangeAndBearingSensor* m_pcRABSens;
    argos::CCI_PositioningSensor* m_pcPosSens;
    argos::CCI_FootBotProximitySensor* m_pcProxSens;
    argos::SInt32 m_nSource = -1;
    argos::Real m_fMaxSpeed = 0.1; // m/s
    argos::UInt32 m_unUid;
    argos::UInt32 m_unTick;
    std::unique_ptr<net_t> m_pcNet;
};
