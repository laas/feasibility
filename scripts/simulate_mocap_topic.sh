#!/bin/bash
rostopic pub -1 /evart/helmet/origin geometry_msgs/TransformStamped '{header: {frame_id: 'mocap_world'}, child_frame_id: 'id', transform: {translation: {x: .49, y: -0.5, z: 0.}, rotation: {x: 0., y: 0., z: 0., w: 1.}}}'
rostopic pub -1 /evart/chair/origin geometry_msgs/TransformStamped '{header: {frame_id: 'mocap_world'}, child_frame_id: 'id', transform: {translation: {x: .49, y: -0.8, z: 0.}, rotation: {x: 0., y: 0., z: 0., w: 1.}}}'
