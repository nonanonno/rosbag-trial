module std_msgs.msg;

import builtin_interfaces.msg;

struct String {
    string data;
}

struct Int32 {
    int data;
}

struct Header {
    Time stamp;
    string frame_id;
}
