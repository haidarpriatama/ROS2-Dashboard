import datetime
import tkinter as tk
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point, Pose, PoseStamped

def get_available_topics(dashboard):
    """Use ROS2 API to get actual available topics."""
    try:
        old_topics = set(dashboard.available_topics) if hasattr(dashboard, 'available_topics') else set()
        dashboard.available_topics = []
        dashboard.available_types = {}
        
        topic_names_and_types = dashboard.node.get_topic_names_and_types()
        
        for topic_name, topic_types in topic_names_and_types:
            dashboard.available_topics.append(topic_name)
            # Store the first type for each topic
            if topic_types:
                topic_type = topic_types[0].split('/')[-1]
                if topic_type in ["String", "Int32", "Point"]:
                    dashboard.available_types[topic_name] = topic_type
                else:
                    # Default to String for other types
                    dashboard.available_types[topic_name] = "String"
        
        # Add an empty option at the beginning of the list
        dashboard.available_topics.insert(0, "")  # Empty topic
        
        # Update combo boxes if they exist
        for i in range(1, 7):  # Updated to support 6 topics
            combo_attr = f"topic{i}_combo"
            if hasattr(dashboard, combo_attr) and getattr(dashboard, combo_attr) is not None:
                current_value = getattr(dashboard, combo_attr).get()
                getattr(dashboard, combo_attr)['values'] = dashboard.available_topics
                if current_value in dashboard.available_topics:
                    getattr(dashboard, combo_attr).set(current_value)
        
        # Update output topic combobox
        if hasattr(dashboard, 'output_topics'):
            dashboard.output_topics['values'] = dashboard.available_topics
            
            # If previously selected, try to reselect
            if hasattr(dashboard, 'output_topic') and dashboard.output_topic in dashboard.available_topics:
                dashboard.output_topics.set(dashboard.output_topic)
        
        # Check for newly appeared robot status/battery topics
        new_topics = set(dashboard.available_topics) - old_topics
        if new_topics:
            print(f"New topics detected: {new_topics}")
            
            # Check for robot status topics
            for i in range(1, 3):
                status_topic = f"/ROBOT{i}/STATUS"
                battery_topic = f"/ROBOT{i}/battery"
                
                # Check if status topic is new or if we're not subscribed yet
                if status_topic in new_topics or (
                    status_topic in dashboard.available_topics and 
                    (not hasattr(dashboard, f"status_topic{i}") or 
                     not getattr(dashboard, f"status_topic{i}"))
                ):
                    if status_topic in dashboard.available_topics:
                        print(f"Auto-subscribing to newly detected topic: {status_topic}")
                        dashboard.subscribe_to_status(status_topic, i)
                
                # Check if battery topic is new or if we're not subscribed yet
                if battery_topic in new_topics or (
                    battery_topic in dashboard.available_topics and 
                    (not hasattr(dashboard, f"battery_topic{i}") or 
                     not getattr(dashboard, f"battery_topic{i}"))
                ):
                    if battery_topic in dashboard.available_topics:
                        print(f"Auto-subscribing to newly detected topic: {battery_topic}")
                        dashboard.subscribe_to_battery(battery_topic, i)
        
        return True
    except Exception as e:
        print(f"Error getting topics: {e}")
        return False

def subscribe_to_topic(dashboard, topic_num, topic_name):
    """Subscribe to a standard topic for data logging"""
    # Unsubscribe if already subscribed
    if getattr(dashboard, f"topic{topic_num}", None):
        if hasattr(dashboard, f"topic{topic_num}_sub"):
            dashboard.node.destroy_subscription(getattr(dashboard, f"topic{topic_num}_sub"))
    
    # Check if topic is valid
    if topic_name not in dashboard.available_topics:
        print(f"Topic {topic_name} not available. Skipping subscription.")
        setattr(dashboard, f"topic{topic_num}", None)
        return
    
    # Set topic name
    setattr(dashboard, f"topic{topic_num}", topic_name)
    
    # Update combobox if it exists
    combo_attr = f"topic{topic_num}_combo"
    if hasattr(dashboard, combo_attr):
        getattr(dashboard, combo_attr).set(topic_name)
    
    # Clear text widget
    text_widget = getattr(dashboard, f"topic{topic_num}_text")
    text_widget.config(state=tk.NORMAL)
    text_widget.delete(1.0, tk.END)
    text_widget.config(state=tk.DISABLED)
    
    # Reset data storage
    setattr(dashboard, f"topic{topic_num}_data", [])
    
    # Callback for handling messages
    def callback(msg):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Handling different message types
        if isinstance(msg, String):
            data = msg.data
        elif isinstance(msg, Int32):
            data = str(msg.data)
        elif isinstance(msg, Point):  # Point message
            # Format Point with all coordinates
            data = f"Point(x={msg.x}, y={msg.y}, z={msg.z})"
        else:
            # Fallback for other types, using str representation
            data = str(msg)
        
        # Prepare log entry
        log_entry = f"[{timestamp}] {data}"
        getattr(dashboard, f"topic{topic_num}_data").append(log_entry)
        
        # Update text widget
        text_widget = getattr(dashboard, f"topic{topic_num}_text")
        text_widget.config(state=tk.NORMAL)
        text_widget.delete(1.0, tk.END)
        for item in getattr(dashboard, f"topic{topic_num}_data")[-20:]:  # Show last 20 entries
            text_widget.insert(tk.END, f"{item}\n")
        text_widget.see(tk.END)  # Scroll to bottom
        text_widget.config(state=tk.DISABLED)
    
    # Subscribe to topic
    try:
        # Try to determine message type
        msg_type = String  # Default
        
        if topic_name in dashboard.available_types:
            type_name = dashboard.available_types[topic_name]
            if type_name == "Int32":
                msg_type = Int32
            elif type_name == "Point":
                msg_type = Point
        
        sub = dashboard.node.create_subscription(
            msg_type,
            topic_name,
            callback,
            10
        )
        setattr(dashboard, f"topic{topic_num}_sub", sub)
    except Exception as e:
        print(f"Error subscribing to topic {topic_name}: {e}")

def subscribe_to_status(dashboard, topic_name, robot_num=1):
    """Subscribe to status topic for specified robot"""
    # Always use the predefined topic name
    topic_name = f"/ROBOT{robot_num}/STATUS"
    
    # Attribute names for status subscriptions
    status_sub_attr = f"status_topic{robot_num}_sub"
    status_topic_attr = f"status_topic{robot_num}"
    
    # Unsubscribe if already subscribed
    if hasattr(dashboard, status_topic_attr) and getattr(dashboard, status_topic_attr) and hasattr(dashboard, status_sub_attr):
        dashboard.node.destroy_subscription(getattr(dashboard, status_sub_attr))
    
    # Check if topic is valid
    if topic_name not in dashboard.available_topics:
        print(f"Status topic {topic_name} not available for Robot {robot_num}. Skipping subscription.")
        setattr(dashboard, status_topic_attr, None)
        return
    
    # Set topic name
    setattr(dashboard, status_topic_attr, topic_name)
    print(f"Subscribing to robot {robot_num} status topic: {topic_name}")
    
    # Create subscription for status
    def callback(msg):
        # Parse status message (format: "ST CPU_USAGE MEMORY_USAGE CPU_TEMP")
        try:
            data = str(msg.data).split()
            if len(data) >= 4:
                # Extract CPU, Memory usage, and CPU temp
                # Format: "ST [CPU] [Memory] [CPU_TEMP]"
                if data[0].upper() == "ST":
                    cpu_usage = data[1]
                    memory_usage = data[2]
                    cpu_temp = data[3]
                    
                    # Update status labels for this robot
                    cpu_attr = f"cpu_usage_{robot_num}"
                    mem_attr = f"memory_usage_{robot_num}"
                    temp_attr = f"cpu_temp_{robot_num}"
                    
                    # Handle CPU usage - show percentage symbol and color-code based on value
                    if hasattr(dashboard, cpu_attr):
                        try:
                            # Try to convert to float to check if it's a number
                            cpu_value = float(cpu_usage)
                            
                            # Color code based on CPU usage ranges
                            if 0 <= cpu_value <= 45:
                                getattr(dashboard, cpu_attr).config(text=cpu_usage, fg="#00AA00")  # Green
                            elif 45 < cpu_value <= 80:
                                getattr(dashboard, cpu_attr).config(text=cpu_usage, fg="#FFEB3B")  # Yellow
                            elif 80 < cpu_value <= 90:
                                getattr(dashboard, cpu_attr).config(text=cpu_usage, fg="#FF8C00")  # Orange
                            elif cpu_value > 90:
                                getattr(dashboard, cpu_attr).config(text=cpu_usage, fg="#FF0000")  # Red
                            else:
                                getattr(dashboard, cpu_attr).config(text=cpu_usage, fg="black")
                            
                            # Show percentage symbol
                            getattr(dashboard, f"{cpu_attr}_pct").pack(side=tk.LEFT, padx=0)
                        except ValueError:
                            # Not a number (like N/A), don't show % symbol
                            getattr(dashboard, cpu_attr).config(text=cpu_usage, fg="black")
                            getattr(dashboard, f"{cpu_attr}_pct").pack_forget()
                    
                    # Handle Memory usage - show percentage symbol if it's a number
                    if hasattr(dashboard, mem_attr):
                        try:
                            # Try to convert to float to check if it's a number
                            float(memory_usage)
                            # Set text without % (will use separate % label)
                            getattr(dashboard, mem_attr).config(text=memory_usage)
                            # Show percentage symbol
                            getattr(dashboard, f"{mem_attr}_pct").pack(side=tk.LEFT, padx=0)
                        except ValueError:
                            # Not a number (like N/A), don't show % symbol
                            getattr(dashboard, mem_attr).config(text=memory_usage)
                            getattr(dashboard, f"{mem_attr}_pct").pack_forget()
                    
                    # Handle CPU temperature - show degree Celsius symbol
                    if hasattr(dashboard, temp_attr):
                        try:
                            # Try to convert to float to check if it's a number
                            float(cpu_temp)
                            # Set text without °C (will use separate label)
                            getattr(dashboard, temp_attr).config(text=cpu_temp)
                            # Show degree symbol
                            getattr(dashboard, f"{temp_attr}_unit").pack(side=tk.LEFT, padx=0)
                        except ValueError:
                            # Not a number (like N/A), don't show degree symbol
                            getattr(dashboard, temp_attr).config(text=cpu_temp)
                            getattr(dashboard, f"{temp_attr}_unit").pack_forget()
        except Exception as e:
            print(f"Error parsing status message for Robot {robot_num}: {e}")
    
    # Subscribe to topic (using appropriate message type)
    try:
        msg_type = String
        if topic_name in dashboard.available_types and dashboard.available_types[topic_name] == "Int32":
            msg_type = Int32
            
        sub = dashboard.node.create_subscription(
            msg_type,
            topic_name,
            callback,
            10
        )
        setattr(dashboard, status_sub_attr, sub)
    except Exception as e:
        print(f"Error subscribing to status topic {topic_name} for Robot {robot_num}: {e}")

def subscribe_to_battery(dashboard, topic_name, robot_num=1):
    """Subscribe to battery topic for specified robot"""
    # Always use the predefined topic name
    topic_name = f"/ROBOT{robot_num}/battery"
    
    # Attribute names for battery subscriptions
    battery_sub_attr = f"battery_topic{robot_num}_sub"
    battery_topic_attr = f"battery_topic{robot_num}"
    
    # Unsubscribe if already subscribed
    if hasattr(dashboard, battery_topic_attr) and getattr(dashboard, battery_topic_attr) and hasattr(dashboard, battery_sub_attr):
        dashboard.node.destroy_subscription(getattr(dashboard, battery_sub_attr))
    
    # Check if topic is valid
    if topic_name not in dashboard.available_topics:
        print(f"Battery topic {topic_name} not available for Robot {robot_num}. Skipping subscription.")
        setattr(dashboard, battery_topic_attr, None)
        return
    
    # Set topic name
    setattr(dashboard, battery_topic_attr, topic_name)
    print(f"Subscribing to robot {robot_num} battery topic: {topic_name}")
    
    # Create subscription for battery
    def callback(msg):
        try:
            # Handle both string and int messages
            if isinstance(msg, String):
                try:
                    battery_level = int(msg.data)
                    is_number = True
                except ValueError:
                    battery_level = msg.data
                    is_number = False
            else:  # Int32
                battery_level = msg.data
                is_number = True
            
            # Update battery label
            batt_attr = f"battery_{robot_num}"
            if hasattr(dashboard, batt_attr):
                # Set the text value
                getattr(dashboard, batt_attr).config(text=str(battery_level))
                
                # Show or hide the percentage symbol
                if is_number:
                    getattr(dashboard, f"{batt_attr}_pct").pack(side=tk.LEFT, padx=0)
                    # Update battery progress bar
                    getattr(dashboard, f"battery{robot_num}_progress")['value'] = battery_level
                else:
                    getattr(dashboard, f"{batt_attr}_pct").pack_forget()
            
        except Exception as e:
            print(f"Error parsing battery message for Robot {robot_num}: {e}")
    
    # Subscribe to topic (supporting both String and Int32)
    try:
        # Try to determine message type
        msg_type = String
        if topic_name in dashboard.available_types and dashboard.available_types[topic_name] == "Int32":
            msg_type = Int32
            
        sub = dashboard.node.create_subscription(
            msg_type,
            topic_name,
            callback,
            10
        )
        setattr(dashboard, battery_sub_attr, sub)
        print(f"Subscribed to {topic_name} for Robot {robot_num} battery using {msg_type.__name__}")
        
    except Exception as e:
        print(f"Error subscribing to battery topic {topic_name} for Robot {robot_num}: {e}")

def subscribe_to_position(dashboard, topic_name, robot_num=1):
    """Subscribe to position topic for specified robot"""
    # Attribute names for position subscriptions
    pos_sub_attr = f"pos_topic{robot_num}_sub"
    pos_topic_attr = f"pos_topic{robot_num}"
    
    # Unsubscribe if already subscribed
    if hasattr(dashboard, pos_topic_attr) and getattr(dashboard, pos_topic_attr) and hasattr(dashboard, pos_sub_attr):
        dashboard.node.destroy_subscription(getattr(dashboard, pos_sub_attr))
    
    # Check if topic is valid
    if topic_name not in dashboard.available_topics:
        print(f"Position topic {topic_name} not available for Robot {robot_num}. Skipping subscription.")
        setattr(dashboard, pos_topic_attr, None)
        return
    
    # Set topic name
    setattr(dashboard, pos_topic_attr, topic_name)
    
    # Create subscription for position
    def callback(msg):
        # Try to extract x,y coordinates
        try:
            # Handle different message types
            if hasattr(msg, 'pose'):  # PoseStamped
                x = msg.pose.position.x
                y = msg.pose.position.y
            elif hasattr(msg, 'position'):  # Pose
                x = msg.position.x
                y = msg.position.y
            elif hasattr(msg, 'x') and hasattr(msg, 'y'):  # Point
                x = msg.x
                y = msg.y
            else:
                print(f"Unsupported message format for position data: {type(msg)}")
                return
                
            # Pass raw coordinates to update_robot_position
            dashboard.update_robot_position(robot_num, x, y)
            
        except Exception as e:
            print(f"Error parsing position message for Robot {robot_num}: {e}")
    
    # Subscribe to topic (try to infer type)
    try:
        # Try to determine message type
        msg_type = None
        try:
            # For simplicity, we'll try common geometry message types
            topic_name_parts = topic_name.split('/')
            if any(geom_type in topic_name_parts for geom_type in ['pose', 'position']):
                if 'stamped' in topic_name.lower():
                    msg_type = PoseStamped
                else:
                    msg_type = Pose
            else:
                # Default to Point if we can't determine
                msg_type = Point
        except:
            # Default fallback
            msg_type = Point
        
        sub = dashboard.node.create_subscription(
            msg_type,
            topic_name,
            callback,
            10
        )
        setattr(dashboard, pos_sub_attr, sub)
        print(f"Subscribed to {topic_name} for Robot {robot_num} position using {msg_type.__name__}")
        
    except Exception as e:
        print(f"Error subscribing to position topic {topic_name} for Robot {robot_num}: {e}")

def update_robot_position(dashboard, robot_num, x, y):
    """Update robot position on the map and coordinate labels"""
    # Store original coordinates for display
    x_original = x
    y_original = y
    
    # Round original coordinates to 2 decimal places for display
    x_original_rounded = round(x_original, 2)
    y_original_rounded = round(y_original, 2)
    
    # Limit x and y to map boundaries for plotting
    x_plot = max(0, min(1500, x))  # Limit x between 0 and 1500
    y_plot = max(0, min(800, y))   # Limit y between 0 and 800

    # Update robot position on the map (with limited coordinates)
    if robot_num == 1:
        dashboard.robot1_pos.set_data(x_plot, y_plot)  # Update position for Robot 1 (limited)
        dashboard.robot1_coords.config(text=f"({x_original_rounded}, {y_original_rounded})")  # Show original coordinates
    elif robot_num == 2:
        dashboard.robot2_pos.set_data(x_plot, y_plot)  # Update position for Robot 2 (limited)
        dashboard.robot2_coords.config(text=f"({x_original_rounded}, {y_original_rounded})")  # Show original coordinates

    # Redraw the canvas
    dashboard.fig.canvas.draw_idle()
    dashboard.fig.canvas.flush_events()

def subscribe_to_graph_topic(dashboard, topic_name):
    """Subscribe to a graph topic with specific data format"""
    # Unsubscribe if already subscribed
    if hasattr(dashboard, 'graph_topic_sub'):
        dashboard.node.destroy_subscription(dashboard.graph_topic_sub)
    
    # Check if topic is valid
    if topic_name not in dashboard.available_topics:
        print(f"Graph topic {topic_name} not available. Skipping subscription.")
        if hasattr(dashboard, 'graph_topic'):
            delattr(dashboard, 'graph_topic')
        return
    
    # Set topic name
    dashboard.graph_topic = topic_name
    
    # Initialize data storage
    dashboard.graph_time_data = []
    dashboard.graph_x_data = []
    dashboard.graph_y_data = []
    dashboard.graph_spx_data = []
    dashboard.graph_spy_data = []
    
    # Initialize graph paused state if not exist
    if not hasattr(dashboard, 'graph_paused'):
        dashboard.graph_paused = False
    
    # Callback for handling messages
    def callback(msg):
        try:
            # Skip processing if graph is paused
            if hasattr(dashboard, 'graph_paused') and dashboard.graph_paused:
                return
                
            # Expected format: 'L [x] [y] [yaw] [spx] [spy] [spyaw]'
            parts = msg.data.split()
            if parts[0] == 'L' and len(parts) >= 7:
                x = float(parts[1])
                y = float(parts[2])
                spx = float(parts[4])
                spy = float(parts[5])
                
                # Get current time
                now = dashboard.node.get_clock().now().nanoseconds / 1e9
                
                # Store data if graph is active
                if hasattr(dashboard, 'graph_time_data'):
                    # Adjust time to start from 0
                    if not dashboard.graph_time_data:
                        dashboard.graph_start_time = now
                    
                    elapsed_time = now - dashboard.graph_start_time
                    
                    dashboard.graph_time_data.append(elapsed_time)
                    dashboard.graph_x_data.append(x)
                    dashboard.graph_y_data.append(y)
                    dashboard.graph_spx_data.append(spx)
                    dashboard.graph_spy_data.append(spy)
                    
                    # Keep only last 100 data points or last 10 seconds of data
                    if elapsed_time > 10:
                        # Remove old data points
                        while dashboard.graph_time_data and dashboard.graph_time_data[0] < elapsed_time - 10:
                            dashboard.graph_time_data.pop(0)
                            dashboard.graph_x_data.pop(0)
                            dashboard.graph_y_data.pop(0)
                            dashboard.graph_spx_data.pop(0)
                            dashboard.graph_spy_data.pop(0)
                    
                    # Update graph if possible
                    if hasattr(dashboard, 'update_graph'):
                        dashboard.update_graph()
        
        except Exception as e:
            print(f"Error parsing graph message: {e}")
    
    # Subscribe to topic
    try:
        sub = dashboard.node.create_subscription(
            String,  # Assuming the topic uses std_msgs/String
            topic_name,
            callback,
            10
        )
        dashboard.graph_topic_sub = sub
    except Exception as e:
        print(f"Error subscribing to graph topic {topic_name}: {e}")

def send_data(dashboard):
    """Send data to a ROS2 topic"""
    # Get input data and selected topic
    data = dashboard.input_entry.get()
    topic = dashboard.output_topics.get()
    
    if not data or not topic:
        return
    
    # Determine message type
    try:
        # First try to convert to integer
        int_val = int(data)
        msg_type = Int32
        msg = Int32()
        msg.data = int_val
    except ValueError:
        # If not an integer, try to convert to Point if it looks like coordinates
        try:
            # Try to parse as Point (x,y,z coordinates)
            coords = [float(x.strip()) for x in data.split(',')]
            
            # Check for valid coordinate input
            if len(coords) == 3:  # x,y,z
                msg_type = Point
                msg = Point()
                msg.x = coords[0]
                msg.y = coords[1]
                msg.z = coords[2]
            elif len(coords) == 2:  # x,y
                msg_type = Point
                msg = Point()
                msg.x = coords[0]
                msg.y = coords[1]
                msg.z = 0.0  # Default z to 0
            else:
                # Fallback to String if not a valid coordinate
                msg_type = String
                msg = String()
                msg.data = data
        except (ValueError, TypeError):
            # If not coordinates, use String
            msg_type = String
            msg = String()
            msg.data = data
    
    # Create publisher if not exists
    if not hasattr(dashboard, 'output_publisher') or dashboard.output_topic != topic:
        if hasattr(dashboard, 'output_publisher'):
            dashboard.node.destroy_publisher(dashboard.output_publisher)
        
        dashboard.output_publisher = dashboard.node.create_publisher(
            msg_type,
            topic,
            10
        )
        dashboard.output_topic = topic
        
        # Save the configuration when the output topic changes
        dashboard.save_config()
    
    # Publish message
    dashboard.output_publisher.publish(msg)
    
    # Add feedback text below the send button
    feedback_text = f"Sent data \"{data}\" to {topic}"
    if not hasattr(dashboard, 'feedback_label'):
        dashboard.feedback_label = tk.Label(dashboard.send_data_frame, text=feedback_text, wraplength=200)
        dashboard.feedback_label.grid(row=5, column=0, pady=5)
    else:
        dashboard.feedback_label.config(text=feedback_text)