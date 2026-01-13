/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "dynamics/traffic_participant.hpp"

#include "adore_math/angles.h"
#include "adore_math/fast_trig.h"

namespace adore
{
namespace dynamics
{
math::Polygon2d
TrafficParticipant::get_corners_at_t( double t_abs, const double longitudinal_inflation, const double lateral_inflation ) const
{
  dynamics::VehicleStateDynamic s = state;

  if( trajectory.has_value() )
    s = trajectory->get_state_at_time( t_abs );

  const double half_length = physical_parameters.body_length / 2.0 + longitudinal_inflation;
  const double half_width  = physical_parameters.body_width / 2.0 + lateral_inflation;

  math::Polygon2d corners;
  corners.points = {
    math::Point2d( -half_length, -half_width ), // rear-right
    math::Point2d( -half_length, half_width ),  // rear-left
    math::Point2d( half_length, half_width ),   // front-left
    math::Point2d( half_length, -half_width )   // front-right
  };

  const double cos_yaw = math::fast_cos( s.yaw_angle );
  const double sin_yaw = math::fast_sin( s.yaw_angle );

  for( auto& corner : corners.points )
  {
    const double local_x = corner.x;
    const double local_y = corner.y;

    const double world_x = s.x + local_x * cos_yaw - local_y * sin_yaw;
    const double world_y = s.y + local_x * sin_yaw + local_y * cos_yaw;

    corner.x = world_x;
    corner.y = world_y;
  }

  return corners;
}

math::Polygon2d
TrafficParticipant::get_corners( const double longitudinal_inflation, const double lateral_inflation ) const
{
  return get_corners_at_t( state.time, longitudinal_inflation, lateral_inflation );
}

void
TrafficParticipantSet::update_traffic_participants( const TrafficParticipant& new_participant_data )
{
  // check if participant is within the validity area
  if( validity_area && !validity_area->point_inside( new_participant_data.state ) ) // Temporarily disabled untill further decision are made
  {
    return;
  }


  if( participants.count( new_participant_data.id ) == 0 )
  {
    participants[new_participant_data.id] = new_participant_data;
    return;
  }

  participants[new_participant_data.id].state               = new_participant_data.state;
  participants[new_participant_data.id].physical_parameters = new_participant_data.physical_parameters;

  if( new_participant_data.goal_point.has_value() )
  {
    participants[new_participant_data.id].goal_point = new_participant_data.goal_point.value();
  }
  if( new_participant_data.trajectory.has_value() )
  {
    participants[new_participant_data.id].trajectory = new_participant_data.trajectory.value();
  }
  if( new_participant_data.route.has_value() )
  {
    participants[new_participant_data.id].route = new_participant_data.route.value();
  }
};

void
TrafficParticipantSet::remove_old_participants( double max_age, double current_time )
{
  for( auto it = participants.begin(); it != participants.end(); )
  {
    if( current_time - it->second.state.time > max_age )
    {
      it = participants.erase( it );
    }
    else
    {
      ++it;
    }
  }
};

} // namespace dynamics


} // namespace adore
