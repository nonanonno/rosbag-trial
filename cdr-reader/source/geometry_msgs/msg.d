module geometry_msgs.msg;

import std_msgs.msg;

struct Point {
    double x;
    double y;
    double z;
}

struct PointStamped {
    Header header;
    Point point;
}
