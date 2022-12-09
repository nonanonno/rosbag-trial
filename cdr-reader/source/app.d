import std.algorithm;
import std.stdio;
import std.range;
import std.format;

import d2sqlite3;
import tabletool;

import rosbag2;

import std_msgs.msg;
import geometry_msgs.msg;
import sensor_msgs.msg;

void main(string[] args) {
    const file = args[1];

    auto db = Database(file);
    Topic[long] topics;
    long[string] topics_ids;

    foreach (Row r; db.execute("SELECT * FROM topics")) {
        auto t = mapOR!Topic(r);
        topics[t.id] = t;
        topics_ids[t.name] = t.id;
    }
    writeln(tabulate(topics.values, Config(Justify.Left)));

    writeln("=============");

    String[] chatter;
    foreach (r; db.execute(
            "SELECT * FROM messages WHERE topic_id = %s".format(
            topics_ids["/chatter"]))) {
        auto message = r.mapOR!Message;

        String data;
        deserialize(message, data);
        chatter ~= data;
    }
    writeln("Displaying /chatter");
    writeln(tabulate(chatter, Config(Justify.Left)));

}
