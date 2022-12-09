module sensor_msgs.msg;

import builtin_interfaces.msg;

struct Image {
    Time stamp;
    string frame_id;
    uint height;
    uint width;
    string encoding;
    ubyte is_bigendian;
    uint step;
    ubyte[] data;
}

struct ChannelFloat32 {
    string name;
    float[] values;
}
