//std::vector<std::unique_ptr<PacketBase>> pipeline;として呼ぶ想定
class PacketBase {
public:
    virtual ~PacketBase() = default;
};

template<typename T>
class Packet : public PacketBase {
public:
    T val;
    Packet(const T& v) : val(v) {}
};
