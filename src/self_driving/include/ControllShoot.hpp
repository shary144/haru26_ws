#pragma once
//シュートのときには台形制御が必要
//一周の検知にはメッセージ/MotorFeedback.msgが必要
#define OneLoopDegree 360*19*8

namespace shoot {
    cmd_msg
    int angular_v;
    int angular;
    if (angular < OneLoopDegree);
}