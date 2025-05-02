import os
import json

def save_config(dashboard):
    """Save configuration to a JSON file"""
    # Create configuration dictionary
    config = {
        "topic1": dashboard.topic1,
        "topic2": dashboard.topic2,
        "topic3": dashboard.topic3,
        "topic4": dashboard.topic4,
        "topic5": dashboard.topic5,
        "topic6": dashboard.topic6,
        "status_topic1": getattr(dashboard, "status_topic1", None),  # Robot 1 status
        "status_topic2": getattr(dashboard, "status_topic2", None),  # Robot 2 status
        "battery_topic1": getattr(dashboard, "battery_topic1", None),  # Robot 1 battery
        "battery_topic2": getattr(dashboard, "battery_topic2", None),  # Robot 2 battery
        "pos_topic1": getattr(dashboard, "pos_topic1", None),  # Robot 1 position
        "pos_topic2": getattr(dashboard, "pos_topic2", None),  # Robot 2 position
        "output_topic": getattr(dashboard, "output_topic", None),  # Ensure output_topic is included
        "graph_topic": getattr(dashboard, "graph_topic", None)  # Graph topic
    }
    
    # Save to file
    try:
        with open(dashboard.config_file, 'w') as f:
            json.dump(config, f)
        print(f"Configuration saved to {dashboard.config_file}")
        if config.get('output_topic'):
            print(f"Output topic saved as: {config['output_topic']}")
        if config.get('graph_topic'):
            print(f"Graph topic saved as: {config['graph_topic']}")
        if config.get('status_topic1'):
            print(f"Robot 1 status topic saved as: {config['status_topic1']}")
        if config.get('status_topic2'):
            print(f"Robot 2 status topic saved as: {config['status_topic2']}")
    except Exception as e:
        print(f"Error saving configuration: {e}")

def load_config(dashboard):
    """Load configuration from a JSON file"""
    try:
        if os.path.exists(dashboard.config_file):
            with open(dashboard.config_file, 'r') as f:
                config = json.load(f)
            
            # Print loaded configuration for debugging
            print(f"Loaded configuration from {dashboard.config_file}")
            
            # Apply configuration to topics (1-6)
            for i in range(1, 7):  # Now supporting 6 topics
                topic_key = f"topic{i}"
                if topic_key in config and config[topic_key]:
                    if config[topic_key] in dashboard.available_topics:
                        dashboard.subscribe_to_topic(i, config[topic_key])
                        # Also update combobox in main UI
                        if hasattr(dashboard, f"topic{i}_combo"):
                            getattr(dashboard, f"topic{i}_combo").set(config[topic_key])
            
            # Load status topics for both robots
            for i in range(1, 3):
                status_key = f"status_topic{i}"
                if status_key in config and config[status_key]:
                    if config[status_key] in dashboard.available_topics:
                        dashboard.subscribe_to_status(config[status_key], i)
                else:
                    # Try to auto-subscribe to ROBOT1/STATUS or ROBOT2/STATUS
                    default_topic = f"/ROBOT{i}/STATUS"
                    if default_topic in dashboard.available_topics:
                        dashboard.subscribe_to_status(default_topic, i)
                        print(f"Auto-subscribed to {default_topic}")
            
            # Load battery topics for both robots
            for i in range(1, 3):
                battery_key = f"battery_topic{i}"
                if battery_key in config and config[battery_key]:
                    if config[battery_key] in dashboard.available_topics:
                        dashboard.subscribe_to_battery(config[battery_key], i)
            
            # Load position topics for both robots
            for i in range(1, 3):
                pos_key = f"pos_topic{i}"
                if pos_key in config and config[pos_key]:
                    if config[pos_key] in dashboard.available_topics:
                        dashboard.subscribe_to_position(config[pos_key], i)
            
            # Load output topic
            if "output_topic" in config and config["output_topic"]:
                dashboard.output_topic = config["output_topic"]
                if hasattr(dashboard, "output_topics"):
                    dashboard.output_topics.set(dashboard.output_topic)
                print(f"Loaded output topic: {dashboard.output_topic}")
            
            # Load graph topic
            if "graph_topic" in config and config["graph_topic"]:
                if config["graph_topic"] in dashboard.available_topics:
                    dashboard.subscribe_to_graph_topic(config["graph_topic"])
                    # Update graph topic combobox in main UI
                    if hasattr(dashboard, 'graph_topic_combo'):
                        dashboard.graph_topic_combo.set(config["graph_topic"])
                    print(f"Loaded graph topic: {config['graph_topic']}")
    
    except Exception as e:
        print(f"Error loading configuration: {e}")