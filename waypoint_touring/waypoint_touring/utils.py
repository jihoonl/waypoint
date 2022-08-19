import yaml
from nav2_msgs.action import NavigateToPose
import geometry_msgs.msg as geometry_msgs
import visualization_msgs.msg as viz_msgs
import std_msgs.msg as std_msgs

id_count = 1


def get_waypoints(filename):
    with open(filename, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)

    return data['waypoints']


def create_geo_pose(p):
    pose = geometry_msgs.Pose()

    pose.position.x = float(p['pose']['position']['x'])
    pose.position.y = float(p['pose']['position']['y'])
    pose.position.z = float(p['pose']['position']['z'])
    pose.orientation.x = float(p['pose']['orientation']['x'])
    pose.orientation.y = float(p['pose']['orientation']['y'])
    pose.orientation.z = float(p['pose']['orientation']['z'])
    pose.orientation.w = float(p['pose']['orientation']['w'])
    return pose


def create_navigate_to_pose_goal(p, stamp):
    target = geometry_msgs.PoseStamped()
    target.header.frame_id = p['frame_id']
    target.header.stamp = stamp
    target.pose = create_geo_pose(p)

    goal = NavigateToPose.Goal(pose=target)
    return goal


def create_viz_markers(waypoints):
    marray = viz_msgs.MarkerArray()
    for w in waypoints:
        m_arrow = create_arrow(w)
        m_text = create_text(w)
        marray.markers.append(m_arrow)
        marray.markers.append(m_text)
    return marray


def create_marker(w):
    global id_count
    m = viz_msgs.Marker()
    m.header.frame_id = w['frame_id']
    m.ns = w['name']
    m.id = id_count
    m.action = viz_msgs.Marker.ADD
    m.pose = create_geo_pose(w)
    m.scale = geometry_msgs.Vector3(x=1.0, y=0.3, z=0.3)
    m.color = std_msgs.ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

    id_count = id_count + 1
    return m


def create_arrow(w):
    m = create_marker(w)
    m.type = viz_msgs.Marker.ARROW
    m.color = std_msgs.ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    return m


def create_text(w):
    m = create_marker(w)
    m.type = viz_msgs.Marker.TEXT_VIEW_FACING
    m.pose.position.z = 2.5
    m.text = w['name']
    return m
