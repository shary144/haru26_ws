#pragma once
#include <stdio.h>
#include <functional>
#include <cmath>

class Mytf {
public:
    using Step = std::function<void(Mytf&)>;

    Mytf(float x=0, float y=0, float yaw=0)
        : _x(x), _y(y), _yaw(yaw) {}

    // 値を直接セット
    Mytf& set_value(float x, float y, float yaw) {
        _x = x;
        _y = y;
        _yaw = yaw;
        return *this;
    }

    // --- 参照渡し版の変換操作 ---
    Mytf& add_trans(float& x, float& y) {
        steps_.push_back([&x, &y](Mytf& tf){
            tf._x += x;
            tf._y += y;
        });
        _x += x;
        _y += y;
        return *this;
    }

    Mytf& sub_trans(float& x, float& y) {
        steps_.push_back([&x, &y](Mytf& tf){
            tf._x -= x;
            tf._y -= y;
        });
        _x -= x;
        _y -= y;
        return *this;
    }

    Mytf& add_rot(float& yaw) {
        steps_.push_back([&yaw](Mytf& tf){
            float c = std::cos(-yaw);
            float s = std::sin(-yaw);

            float nx = c * tf._x - s * tf._y;
            float ny = s * tf._x + c * tf._y;

            tf._x = nx;
            tf._y = ny;
            tf._yaw += yaw;  // 座標系の向きが変わる
        });

        // 自分自身にも即時適用
        float c = std::cos(-yaw);
        float s = std::sin(-yaw);
        float nx = c * _x - s * _y;
        float ny = s * _x + c * _y;
        _x = nx;
        _y = ny;
        _yaw += yaw;

        return *this;
    }

    Mytf& sub_rot(float& yaw) {
         steps_.push_back([&yaw](Mytf& tf){
            float c = std::cos(yaw);
            float s = std::sin(yaw);

            float nx = c * tf._x - s * tf._y;
            float ny = s * tf._x + c * tf._y;

            tf._x = nx;
            tf._y = ny;
            tf._yaw -= yaw;  // 座標系の向きが変わる
        });

        // 自分自身にも即時適用
        float c = std::cos(yaw);
        float s = std::sin(yaw);
        float nx = c * _x - s * _y;
        float ny = s * _x + c * _y;
        _x = nx;
        _y = ny;
        _yaw -= yaw;

        return *this;
    }


    float x() const { return _x; }
    float y() const { return _y; }
    float yaw() const { return _yaw; }

private:
    float _x, _y, _yaw;
    std::vector<Step> steps_;
};