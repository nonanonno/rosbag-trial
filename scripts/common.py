import rosbag2_py

SERIALIZATION_FORMAT = "cdr"


def get_rosbag_options(path: str):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=SERIALIZATION_FORMAT,
        output_serialization_format=SERIALIZATION_FORMAT,
    )
    return storage_options, converter_options


def create_topic(writer: rosbag2_py.SequentialWriter, topic_name: str, topic_type: str):
    topic = rosbag2_py.TopicMetadata(
        name=topic_name, type=topic_type, serialization_format=SERIALIZATION_FORMAT
    )
    writer.create_topic(topic)


def to_timestamp(milli: int):
    return milli * 1000 * 1000
