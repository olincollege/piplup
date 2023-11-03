#include "robotiq_epick/suction_force_model.h"

#include <drake/geometry/query_object.h>
#include <drake/geometry/query_results/signed_distance_pair.h>

#include <algorithm>
#include <limits>
namespace piplup {
namespace suction_gripper {

// ------------------- ExternallyAppliedSpatialForcePair -------------------

std::pair<drake::multibody::ExternallyAppliedSpatialForce<double>,
          drake::multibody::ExternallyAppliedSpatialForce<double>>
ExternallyAppliedSpatialForcePair::GetAsPair() {
    std::pair<drake::multibody::ExternallyAppliedSpatialForce<double>,
              drake::multibody::ExternallyAppliedSpatialForce<double>>
        pair;

    pair.first.body_index = body_index1_;
    pair.first.p_BoBq_B = p_BoBq_B1_;
    pair.first.F_Bq_W = drake::multibody::SpatialForce<double>(
        /*tau*/ trq_axis_ * tau_mag_, /*f*/ force_axis_ * f_mag_);

    pair.second.body_index = body_index2_;
    pair.second.p_BoBq_B = p_BoBq_B2_;
    pair.second.F_Bq_W = drake::multibody::SpatialForce<double>(
        /*tau*/ -trq_axis_ * tau_mag_, /*f*/ -force_axis_ * f_mag_);
    return pair;
}

// ------------------- CupPressureSource -------------------

CupPressureSource::CupPressureSource(double vacuum_source_pressure,
                                     double max_suction_dist,
                                     int num_suction_cups)
    : vacuum_source_pressure_(vacuum_source_pressure),
      max_suction_dist_(max_suction_dist),
      num_suction_cups_(num_suction_cups) {
    /// ----- Input Ports ----- ///
    suction_cup_obj_dist_input_port_idx_ =
        DeclareVectorInputPort(
            "cup_obj_dists",
            drake::systems::BasicVector<double>(num_suction_cups_))
            .get_index();
    suction_cmd_input_port_idx_ =
        DeclareVectorInputPort(
            "suction_cmds",
            drake::systems::BasicVector<double>(num_suction_cups_))
            .get_index();

    /// ----- Output Ports ----- ///
    suction_cup_pressure_output_port_idx_ =
        DeclareVectorOutputPort(
            "suction_cup_pressures",
            drake::systems::BasicVector<double>(num_suction_cups_),
            &CupPressureSource::CalcSuctionCupPressure,
            {all_input_ports_ticket()})
            .get_index();
}

void CupPressureSource::CalcSuctionCupPressure(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* suction_cup_pressure_ptr) const {
    const auto& cup_obj_dist_vec =
        GetCupObjDistInputPort().Eval<drake::systems::BasicVector<double>>(
            context);
    const auto& suction_cmd_vec =
        GetSuctionCmdInputPort().Eval<drake::systems::BasicVector<double>>(
            context);

    for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
         suction_cup_idx++) {
        double suction_cmd = suction_cmd_vec[suction_cup_idx];
        DRAKE_DEMAND(suction_cmd >= 0. && suction_cmd <= 1.);
        double dist = cup_obj_dist_vec[suction_cup_idx];
        double pressure = 0.;
        // use a simple linear pressure-distance model
        if (dist <= 0.) {
            pressure = vacuum_source_pressure_;
        } else if (dist <= max_suction_dist_) {
            pressure = (max_suction_dist_ - dist) * vacuum_source_pressure_ /
                       max_suction_dist_;
        } else {
            pressure = 0.;
        }
        drake::log()->info(pressure);
        (*suction_cup_pressure_ptr)[suction_cup_idx] = suction_cmd * pressure;
    }
}

// ------------------- CupObjInterface -------------------
CupObjInterface::CupObjInterface(
    double time_step, double suction_cup_area,
    const drake::multibody::BodyIndex& gripper_body,
    const std::pair<drake::geometry::FrameId, std::vector<drake::math::RigidTransformd>>& action_point_frames,
    const std::vector<drake::geometry::GeometryIdSet>& edge_points,
    const std::unordered_map<drake::geometry::GeometryId,
                             drake::multibody::BodyIndex>&
        obj_geom_id_to_body_idx_map)
    : suction_cup_area_(suction_cup_area),
      gripper_body_(gripper_body),
      action_point_frames_(action_point_frames),
      edge_points_(edge_points),
      obj_geom_id_to_body_idx_map_(obj_geom_id_to_body_idx_map),
      num_suction_cups_(action_point_frames.second.size()) {
    /// ----- Input Ports ----- ///
    geom_query_input_port_idx_ =
        DeclareAbstractInputPort(
            "geom_query", drake::Value<drake::geometry::QueryObject<double>>())
            .get_index();

    suction_cup_pressure_input_port_idx_ =
        DeclareVectorInputPort(
            "suction_cup_pressures",
            drake::systems::BasicVector<double>(num_suction_cups_))
            .get_index();

    /// ----- Output Ports ----- ///
    suction_force_output_port_idx_ =
        DeclareAbstractOutputPort(
            /*name*/ "suction_forces",
            /*alloc_function*/
            std::vector<
                drake::multibody::ExternallyAppliedSpatialForce<double>>(
                2 * num_suction_cups_),
            /*calc_function*/ &CupObjInterface::CalcSuctionForce,
            /*dependency*/
            {xa_ticket(), xd_ticket(),
             input_port_ticket(drake::systems::InputPortIndex(
                 suction_cup_pressure_input_port_idx_))})
            .get_index();

    suction_cup_obj_dist_output_port_idx_ =
        DeclareVectorOutputPort(
            /*name*/ "cup_obj_dists",
            /*alloc_function*/
            drake::systems::BasicVector<double>(num_suction_cups_),
            /*calc_function*/ &CupObjInterface::OutputCupObjDist,
            /*dependency*/ {xa_ticket(), xd_ticket()})
            .get_index();

    /// ----- States ----- ///
    // each entry is a signed dist pair between the center action point and the
    // closest object
    suction_cup_act_pt_closest_obj_signed_dist_pair_state_idx_ =
        DeclareAbstractState(
            drake::Value<
                std::vector<drake::geometry::SignedDistanceToPoint<double>>>(
                std::vector<drake::geometry::SignedDistanceToPoint<double>>(
                    num_suction_cups_)));

    // each entry is the mean distance between the suction cup edge points to
    // the closest object
    suction_cup_edge_pt_closest_obj_dist_state_idx_ =
        DeclareDiscreteState(num_suction_cups_);

    DeclareInitializationUnrestrictedUpdateEvent(&CupObjInterface::UpdateDists);
    DeclarePeriodicUnrestrictedUpdateEvent(time_step, 0,
                                           &CupObjInterface::UpdateDists);
}

drake::systems::EventStatus CupObjInterface::UpdateDists(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state_ptr) const {
    const auto& geom_query =
        GetGeomQueryInputPort().Eval<drake::geometry::QueryObject<double>>(
            context);
    auto& suction_cup_act_pt_closest_obj_signed_dist_state =
        state_ptr->get_mutable_abstract_state()
            .get_mutable_value(
                suction_cup_act_pt_closest_obj_signed_dist_pair_state_idx_)
            .get_mutable_value<
                std::vector<drake::geometry::SignedDistanceToPoint<double>>>();

    auto& suction_cup_edge_pt_closest_obj_dist_state =
        state_ptr->get_mutable_discrete_state(
            suction_cup_edge_pt_closest_obj_dist_state_idx_);

    for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
         suction_cup_idx++) {
        auto action_pt_pose = geom_query.GetPoseInWorld(action_point_frames_.first) * action_point_frames_.second.at(suction_cup_idx);
        drake::geometry::GeometryId closest_obj_geom_id;
        auto min_action_point_dist = std::numeric_limits<double>::infinity();

        auto all_signed_dists = geom_query.ComputeSignedDistanceToPoint(
            action_pt_pose.translation());

        for (const auto& signed_dist : all_signed_dists) {
            if (obj_geom_id_to_body_idx_map_.find(signed_dist.id_G) !=
                obj_geom_id_to_body_idx_map_.end()) {
                if (signed_dist.distance < min_action_point_dist) {
                    min_action_point_dist = signed_dist.distance;
                    closest_obj_geom_id = signed_dist.id_G;
                    suction_cup_act_pt_closest_obj_signed_dist_state.at(
                        suction_cup_idx) = signed_dist;
                }
            }
        };
        auto mean_edge_pt_obj_dist = 0.;
        for (const auto& suction_cup_edge_pt_geom_id :
             edge_points_[suction_cup_idx]) {
            auto signed_dist_pair =
                geom_query.ComputeSignedDistancePairClosestPoints(
                    suction_cup_edge_pt_geom_id, closest_obj_geom_id);
            mean_edge_pt_obj_dist += std::max(signed_dist_pair.distance, 0.);
        }
        mean_edge_pt_obj_dist /= edge_points_[suction_cup_idx].size();
        suction_cup_edge_pt_closest_obj_dist_state[suction_cup_idx] =
            mean_edge_pt_obj_dist;
    }
    return drake::systems::EventStatus::Succeeded();
}

void CupObjInterface::CalcSuctionForce(
    const drake::systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
        suction_force_vec_ptr) const {
    const auto& pressure_vec =
        GetSuctionCupPressureInputPort()
            .Eval<drake::systems::BasicVector<double>>(context);

    const auto& closest_obj_signed_dists = context.get_abstract_state<
        std::vector<drake::geometry::SignedDistanceToPoint<double>>>(
        suction_cup_act_pt_closest_obj_signed_dist_pair_state_idx_);

    for (int suction_cup_idx = 0; suction_cup_idx < num_suction_cups_;
         suction_cup_idx++) {
        auto signed_dist = closest_obj_signed_dists.at(suction_cup_idx);
        drake::geometry::GeometryId closest_obj_geom_id = signed_dist.id_G;
        Eigen::Vector3d p_GC = signed_dist.p_GN;  // geometry to closest point
        Eigen::Vector3d cup_act_pt_obj_vec = -signed_dist.grad_W;

        double f_mag = -pressure_vec[suction_cup_idx] * suction_cup_area_;
        auto obj_body_idx =
            obj_geom_id_to_body_idx_map_.at(closest_obj_geom_id);

        const auto& scene_graph_inspector =
            GetGeomQueryInputPort()
                .Eval<drake::geometry::QueryObject<double>>(context)
                .inspector();
        const auto& X_BG = scene_graph_inspector.GetPoseInFrame(
            closest_obj_geom_id);  // body to geometry

        auto X_Gripper_Cup = action_point_frames_.second.at(suction_cup_idx);
        ExternallyAppliedSpatialForcePair suction_force_pair(
            /*body_index1*/ gripper_body_,
            /*body_index2*/ obj_body_idx,
            /*p_BoBq_B1*/ X_Gripper_Cup.translation(),
            /*p_BoBq_B2*/
            (X_BG * drake::math::RigidTransform<double>(p_GC)).translation(),
            /*force_axis*/ cup_act_pt_obj_vec,
            /*f_mag*/ f_mag,
            /*trq_axis*/ Eigen::Vector3d::UnitZ(),
            /*tau_mag*/ 0);

        auto suction_force_pair_vec = suction_force_pair.GetAsPair();

        suction_force_vec_ptr->at(2 * suction_cup_idx) =
            suction_force_pair_vec.first;
        suction_force_vec_ptr->at(2 * suction_cup_idx + 1) =
            suction_force_pair_vec.second;
    }
}

void CupObjInterface::OutputCupObjDist(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* cup_obj_dist_vec_ptr) const {
    const auto& suction_cup_edge_pt_closest_obj_dist_state =
        context.get_discrete_state(
            suction_cup_edge_pt_closest_obj_dist_state_idx_);
    cup_obj_dist_vec_ptr->SetFromVector(
        suction_cup_edge_pt_closest_obj_dist_state.get_value());
}

}  // namespace suction_gripper
}  // namespace piplup