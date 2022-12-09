module rosbag2;

import std.conv;
import std.traits;

import d2sqlite3;

/**
 * Map as OR mapper
 * Params:
 *      r = A record
 * Returns: Mapped record
 */
T mapOR(T)(Row r) {
    T v;
    static foreach (i, m; __traits(allMembers, T)) {
        __traits(getMember, v, m) = r[i].as!(typeof(__traits(getMember, v, m)));
    }
    return v;
}

unittest {
    struct Item {
        string name;
        long price;
    }

    auto db = Database(":memory:");
    db.run("CREATE TABLE items (name TEXT, price INTEGER);
            INSERT INTO items VALUES ('car', 20000);"
    );
    auto record = db.execute("SELECT * FROM items;").front();

    auto item = mapOR!Item(record);
    assert(item.name == "car");
    assert(item.price == 20_000);
}

// rosbag db schema
/// topics table
struct Topic {
    long id;
    string name;
    string type;
    string serialization_format;
    string offered_qos_profiles;
}

/// messages table
struct Message {
    long id;
    long topic_id;
    long timestamp;
    void[] data;
}

void deserialize(T)(Message message, out T dst) {
    auto buffer = new CDRBuffer(message.data);
    deserialize(buffer, dst);
}

private class CDRBuffer {
    int index;
    void[] data;

    this(void[] data) {
        this.data = data[4 .. $];
        index = 0;
    }

    void consume(T)(out T v) {
        assert(index < data.length);
        static if (is(T == string)) {
            int len;
            consume(len);
            v = (cast(char[]) data[index .. index + len]).to!string;
            index += len;
        }
        else static if (isDynamicArray!(T)) {
            int len;
            consume(len);
            v = cast(T) data[index .. index + len * v[0].sizeof];
            index += len * v[0].sizeof;
        }
        else {
            index = ceiling(index, T.sizeof);
            v = *(cast(T*) data[index .. index + T.sizeof]);
            index += T.sizeof;
        }
    }
}

private void deserialize(T)(CDRBuffer buffer, out T v) {
    static foreach (m; __traits(allMembers, T)) {
        static if (isAggregateType!(typeof(__traits(getMember, v, m)))) {
            deserialize(buffer, __traits(getMember, v, m));
        }
        else {
            buffer.consume(__traits(getMember, v, m));
        }
    }
}

private int ceiling(int x, int n) {
    return ((x + n - 1) / n) * n;
}
