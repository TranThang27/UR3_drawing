#!/usr/bin/env python3
"""Record RViz-goal navigation evidence; never send goals or velocity commands."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Path as NavPath, Odometry, OccupancyGrid
from nav2_msgs.msg import ParticleCloud
from tf2_ros import Buffer, TransformListener, TransformException
from capture_x11_window import WindowCapture

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=lambda s:int(s,0), required=True)
    parser.add_argument('--goals', type=int, default=3)
    parser.add_argument('--duration', type=float, default=480)
    parser.add_argument('--output', type=Path, default=ROOT/'reports/part4')
    parser.add_argument('--goal-source', choices=['rviz','ros_action'], default='rviz')
    parser.add_argument('--continue-after-preemption', action='store_true')
    args = parser.parse_args()
    out = args.output; (out/'frames').mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('navigation_evidence_recorder')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node); listener = TransformListener(buffer, node)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(NavPath, '/navigation_trail', latched)
    trail = NavPath(); trail.header.frame_id = 'map'
    goals = {}
    report = {'goal_source':('RViz2 Nav2 Goal mouse tool' if args.goal_source=='rviz'
                            else 'ROS2 NavigateToPose action client'), 'goals':[],
              'manual_velocity_commands_sent_by_recorder':0,
              'hazard_messages':0, 'bump_messages':0, 'plan_messages':0,
              'local_plan_messages':0, 'particle_messages':0, 'particle_count':0,
              'global_costmap_messages':0, 'local_costmap_messages':0,
              'minimum_clearance_m':None, 'frames':[], 'passed':False,
              'note':'Goal results are observed from Nav2 action status. Trail uses map/base_link '
                     'TF. Clearance is independently computed from Gazebo pose against room '
                     'walls, boxes and cylinder with radius 0.175 m; dock has been removed and other robot '
                     'geometry is excluded. Bumper reports and images supplement this check.'}
    current = None
    latest_pose = None
    start = time.monotonic()

    def statuses(msg):
        nonlocal current
        for status in msg.status_list:
            key = bytes(status.goal_info.goal_id.uuid).hex()
            if key not in goals:
                # Transient-local action status includes completed older goals.
                # Only record goals observed while accepted or executing.
                if status.status not in (1,2):
                    continue
                goals[key] = {'uuid':key, 'number':len(goals)+1, 'status':status.status,
                              'started_wall_s':time.monotonic()-start, 'plan_messages':0,
                              'bump_messages_before':report['bump_messages']}
            entry = goals[key]; old = entry['status']; entry['status'] = status.status
            if status.status in (1,2): current = key
            if status.status in (4,5,6) and 'finished_wall_s' not in entry:
                entry['finished_wall_s'] = time.monotonic()-start
                entry['final_map_xy'] = latest_pose
                entry['bump_messages'] = report['bump_messages']-entry['bump_messages_before']
                if latest_pose and 'target_map_xy' in entry:
                    entry['position_error_m'] = math.dist(latest_pose, entry['target_map_xy'])
                print('Goal terminal:',json.dumps(entry),flush=True)

    def plan(msg):
        if not msg.poses:return
        report['plan_messages'] += 1
        if current:
            entry=goals[current]; entry['plan_messages'] += 1
            p=msg.poses[-1].pose.position;entry['target_map_xy']=[p.x,p.y]

    def hazards(msg):
        report['hazard_messages'] += 1
        if any(d.type==HazardDetection.BUMP for d in msg.detections):report['bump_messages'] += 1

    def ground_truth(msg):
        p=msg.pose.pose.position;x,y=p.x,p.y
        distances=[3-abs(x)-.175,3-abs(y)-.175,math.hypot(x-1,y+1.2)-.25-.175]
        for cx,cy,hx,hy in [(-1.2,1,.25,.25),(1.2,1.4,.4,.2)]:
            distances.append(math.hypot(max(abs(x-cx)-hx,0),max(abs(y-cy)-hy,0))-.175)
        value=min(distances)
        if report['minimum_clearance_m'] is None or value<report['minimum_clearance_m']:
            report['minimum_clearance_m']=value
        report['latest_gazebo_xy']=[x,y]

    def particles(msg):
        report['particle_messages'] += 1;report['particle_count']=len(msg.particles)

    def count(key):
        def callback(msg):report[key] += 1
        return callback

    subs=[node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',statuses,latched),
          node.create_subscription(NavPath,'/plan',plan,10),
          node.create_subscription(NavPath,'/local_plan',count('local_plan_messages'),10),
          node.create_subscription(HazardDetectionVector,'/hazard_detection',hazards,qos_profile_sensor_data),
          node.create_subscription(Odometry,'/sim_ground_truth_pose',ground_truth,qos_profile_sensor_data),
          node.create_subscription(ParticleCloud,'/particle_cloud',particles,qos_profile_sensor_data),
          node.create_subscription(OccupancyGrid,'/global_costmap/costmap',count('global_costmap_messages'),latched),
          node.create_subscription(OccupancyGrid,'/local_costmap/costmap',count('local_costmap_messages'),latched)]
    capture=WindowCapture(args.window)
    next_path=next_frame=0; finished=None
    try:
        while time.monotonic()-start<args.duration:
            rclpy.spin_once(node,timeout_sec=.05)
            now=time.monotonic()
            if now>=next_path:
                next_path=now+.25
                try:
                    tf=buffer.lookup_transform('map','base_link',rclpy.time.Time())
                    p=tf.transform.translation;latest_pose=[p.x,p.y]
                    if not trail.poses or math.dist(latest_pose,[trail.poses[-1].pose.position.x,
                                                               trail.poses[-1].pose.position.y])>.025:
                        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
                        pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.orientation=tf.transform.rotation
                        trail.poses.append(pose)
                    trail.header.stamp=node.get_clock().now().to_msg();publisher.publish(trail)
                except TransformException:pass
            if now>=next_frame:
                next_frame=now+2
                name=f'frame_{len(report["frames"]):04d}.png';capture.grab().save(out/'frames'/name)
                report['frames'].append({'file':name,'wall_s':now-start,'robot_map_xy':latest_pose,
                                        'statuses':{str(e['number']):e['status'] for e in goals.values()},
                                        'path_points':len(trail.poses)})
                report['goals']=list(goals.values());report['path_points']=len(trail.poses)
                temporary=out/'navigation_progress.tmp'
                temporary.write_text(json.dumps(report,indent=2)+'\n')
                temporary.replace(out/'navigation_progress.json')
            completed=[e for e in goals.values() if e['status']==4]
            if any(e['status'] in (5,6) for e in goals.values()) and not args.continue_after_preemption:break
            if len(completed)>=args.goals:
                if finished is None:finished=now
                if now-finished>5:break
        report['goals']=list(goals.values());report['path_points']=len(trail.poses)
        completed=[e for e in goals.values() if e['status']==4]
        report['passed']=(len(completed)>=args.goals and all(e['plan_messages']>0
            and e.get('position_error_m',99)<.35 for e in completed)
            and report['hazard_messages']>0 and report['bump_messages']==0
            and report['minimum_clearance_m'] is not None and report['minimum_clearance_m']>0)
    finally:
        (out/'navigation_results.json').write_text(json.dumps(report,indent=2)+'\n')
        (out/'travelled_path.json').write_text(json.dumps([[p.pose.position.x,p.pose.position.y]
                                                         for p in trail.poses])+'\n')
        capture.grab().save(out/'navigation_final.png')
        print('Navigation passed:',report['passed'],flush=True)
        capture.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
