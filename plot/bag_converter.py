import os
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import pandas as pd
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# 🔧 **Configurable Variables** (Modify these easily)
BAG_FILENAME = "rosbag2_2025_03_09-15_05_07_0.db3"  # Change to your actual ROS bag filename
BAG_PATH = os.path.join(os.getcwd(), BAG_FILENAME)  # Ensure correct file path

TOPICS = [
    '/bluerov2/cmd_vel',
    '/bluerov2/joy'
]  # Modify to include any additional topics you want to extract


def read_rosbag_to_dataframe(bag_path, topics):
    """Reads a ROS2 bag file and converts specified topics into Pandas DataFrames."""
    try:
        rclpy.init()

        # Initialize ROS bag reader
        reader = SequentialReader()
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')

        reader.open(storage_options, converter_options)

        # ✅ Extract topic metadata **before** iterating through messages
        topic_metadata = {t.name: t.type for t in reader.get_all_topics_and_types()}

        # Prepare data storage
        data = {topic: [] for topic in topics}

        print(f"🔍 Extracting topics from: {bag_path}")

        # Iterate through the bag messages
        while reader.has_next():
            try:
                (topic, msg, t) = reader.read_next()

                if topic in topics:
                    if topic in topic_metadata:
                        msg_type = get_message(topic_metadata[topic])
                        deserialized_msg = deserialize_message(msg, msg_type)

                        # Convert message to dictionary and add timestamp
                        msg_dict = {field: getattr(deserialized_msg, field) for field in deserialized_msg.__slots__}
                        msg_dict['timestamp'] = t
                        data[topic].append(msg_dict)

            except Exception as e:
                print(f"⚠️ Error processing message from topic '{topic}': {e}")

        # Convert to pandas DataFrames
        df_dict = {}
        for topic, messages in data.items():
            try:
                if messages:
                    df_dict[topic] = pd.DataFrame(messages)
                else:
                    print(f"⚠️ Warning: No messages found for topic '{topic}'")

            except Exception as e:
                print(f"⚠️ Error converting topic '{topic}' to DataFrame: {e}")

    except Exception as e:
        print(f"❌ Critical Error: Failed to process ROS bag. Reason: {e}")
        df_dict = {}

    finally:
        rclpy.shutdown()

    return df_dict


if __name__ == '__main__':
    try:
        df_dict = read_rosbag_to_dataframe(BAG_PATH, TOPICS)

        # Save extracted data to CSV files
        for topic, df in df_dict.items():
            try:
                filename = f"{topic.replace('/', '_')}.csv"
                file_path = os.path.join(os.getcwd(), filename)
                df.to_csv(file_path, index=False)
                print(f"✅ Saved: {file_path}")

            except Exception as e:
                print(f"⚠️ Error saving topic '{topic}' to CSV: {e}")

    except Exception as e:
        print(f"❌ Fatal Error: {e}")
