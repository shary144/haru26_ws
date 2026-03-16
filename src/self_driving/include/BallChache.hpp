#pragma once
#include <vector>
#include <cmath>

namespace ballcache {

struct Ball {
    int color_id;   // 0,1,2
    float x;
    float y;
    bool onstage = false;
    size_t N = 0;   // 観測回数
};

class BallCache {
public:
    double inclusive_radius = 0.3; // 同一ボールとみなす距離
    std::vector<Ball> ball_array;

    // 新しい観測をキャッシュに吸収する
    void enroll_ball_array(const std::vector<Ball>& new_ball_array) {

        // 初回は丸ごとコピー
        if (ball_array.empty()) {
            ball_array = new_ball_array;
            for (auto& b : ball_array) {
                b.N = 1;
                b.onstage = true;
            }
            return;
        }

        // 既存ボールに対応付け
        for (auto& new_ball : new_ball_array) {
            bool matched = false;

            for (auto& ball : ball_array) {
                if (ball.color_id != new_ball.color_id)
                    continue;

                double dx = ball.x - new_ball.x;
                double dy = ball.y - new_ball.y;
                double d = std::sqrt(dx*dx + dy*dy);

                if (d < inclusive_radius) {
                    // 加重平均で更新
                    ball.x = (new_ball.x + ball.N * ball.x) / (ball.N + 1);
                    ball.y = (new_ball.y + ball.N * ball.y) / (ball.N + 1);
                    ball.N++;
                    ball.onstage = true;
                    matched = true;
                    break;
                }
            }

            // どの既存ボールにもマッチしなかった → 新規ボール
            if (!matched) {
                Ball b = new_ball;
                b.N = 1;
                b.onstage = true;
                ball_array.push_back(b);
            }
        }
    }
};

}