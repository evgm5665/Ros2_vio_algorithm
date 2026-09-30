import sys
import os
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

#reads /vio/pose_base and /ground_truth/odom from LiveTestFLight bag and writes two text files.
#Extracts trajectories, not pair, align, nor plot them

# Configuration
BAG_DIR = os.path.expanduser("~/Ovio_ws/bags/LiveTestFlight2")
GT_TOPIC = "/ground_truth/odometry"
VIO_TOPIC = "/vio/pose_base"

# Output Text Files
GT_TXT = "ground_truth2.txt"
VIO_TXT = "estimate2.txt"

def extract_topic_to_txt(db_path, topic_name, output_txt, msg_type_str, is_odom=True):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get topic ID
    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
    row = cursor.fetchone()
    if not row:
        print(f"Topic {topic_name} not found in bag database.")
        conn.close()
        return
    topic_id = row[0]
    
    # Query messages
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topic_id,))
    rows = cursor.fetchall()
    
    msg_type = get_message(msg_type_str)
    
    with open(output_txt, "w") as f:
        for timestamp, data in rows:
            msg = deserialize_message(data, msg_type)
            
            # Extract header timestamp
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            t = sec + (nanosec * 1e-9)
            if t == 0:
                t = timestamp * 1e-9
            
            # Switch property path based on message structure
            if is_odom:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
            else:
                p = msg.pose.position
                q = msg.pose.orientation
            
            # Format: timestamp tx ty tz qx qy qz qw
            f.write(f"{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
            
    print(f"Successfully wrote {len(rows)} points to {output_txt}")
    conn.close()

if __name__ == "__main__":
    db_file = None
    for f in os.listdir(BAG_DIR):
        if f.endswith(".db3") or f.endswith(".sqlite3"):
            db_file = os.path.join(BAG_DIR, f)
            break
            
    if not db_file:
        print(f"Could not find an sqlite3 database file (.db3) in {BAG_DIR}")
        sys.exit(1)
        
    print(f"Reading bag database: {db_file}")
    # Extract Ground Truth (Odometry message type)
    extract_topic_to_txt(db_file, GT_TOPIC, GT_TXT, "nav_msgs/msg/Odometry", is_odom=True)
    # Extract VIO Base Pose (PoseStamped message type)
    extract_topic_to_txt(db_file, VIO_TOPIC, VIO_TXT, "geometry_msgs/msg/PoseStamped", is_odom=False)
