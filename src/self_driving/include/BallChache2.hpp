#pragma once
#include <stdio.h>
#include <vector>
#include <cmath>
#include <algorithm>
//これは重いかもしれないが大きなずれにも対応できる

struct Ball {
    int color_id; // 0 or 1 or 2
    float x;
    float y;
    bool onstage;
    size_t N;
};

struct BallChache {
    std::vector<Ball> balls;
    Ball fetch_real(size_t BallChache_ind) {
        BallChache_ind
        return ball;
    }
    void match(std::vector<Ball> new_balls) {
        std::sort(new_balls.begin(),new_balls.end(),
            [](Ball ball1, Ball ball2) {return ball1.x < ball2.x;}
        );

        if (balls.empty()){
            std::vector<Ball> balls(new_balls);
        } else {
            for (int first=0;<balls.size();i++){
                
            }
        }
    }
    void pull_over(std::vector<Ball> new_balls) {
        //カメラの仕組みを考えると、遠くなるほどボールの大きさの検出精度が下がる
        //すなわち距離で重みを補正したい->今回は
        //なので距離閾値を取って
        double va_x,va_y,m_x,m_y,sum_x,sum_y;
        std::vector<double> d2_v;
        for (Ball& new_ball: new_balls){
            double d2 = std::sqrt(std::pow(new_ball.x,2)+std::pow(new_ball.y,2));
            sum_x += new_ball.x/std::sqrt(d2);
            sum_y += new_ball.y/std::sqrt(d2);
            d2_v.push_back(d2);
        }
        m_x = sum_x/new_balls.size();
        m_y = sum_y/new_balls.size();
        auto it1 = d2_v.begin();
        auto it2 = new_balls.begin();
        for (;(it1!=d2_v.end())&&(it2!=new_balls.end());++it1,++it2) {
            va_x = (it2->x-m_x)/(*it1);
            va_y = (it2->y-m_y)/(*it2);
        }
        
        
    }
}