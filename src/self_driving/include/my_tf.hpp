#pragma once
//目標とする表記法
//以下座標系は標準のユークリッド直交座標系と姿勢を指す
//Mytf2 frameA
//Mytf2 frameB
//frameB.set_base(frameA(x,y,th)) ...座標系B上の基底x軸をを座標系A上のポーズ(x,y,th)から生やす
//parentはframeAに対するframeBのようなもの
//この時offset:(x,y,th)をframeAにもframeBにも紐づけて互いにインスタンス内の履歴として持つ。この親子関係が後からわかるように
//frameAからframeBの時
// B =(+x,+y)=(+th)=> A (child2parent)
// A =(-th)=(-x,-y)=> B (parent2child)
//さてset_baseが呼びだされたときの
//各インスタンス内履歴の形としては
//* 相手のframeインスタンス参照
//* 親か否か
//* x,y,th
//これらの履歴があれば変換の必要条件は満たす

//auto& pos = frameB(frameA(x,y,th)) ...座標系A上のポーズ(x,y,th)を座標系Bで表現
//引数で紐づけられる順番に処理self_frame(other_frame)になるときother_frameの値をもとにself_frameが処理
//self_frameがother_frameを履歴から探して上の変換処理をする
//親子関係をたどってたどりつけないときはエラー

//pos.x(), pos.y(), pos.th()のように使う値を引きだす
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>

struct Frame; // 前方宣言

// =========================
// Pose: 座標値 + どのフレーム基準か
// =========================
struct Pose {
    double x, y, th;
    const Frame* frame;
};

// =========================
// Link: フレーム間の双方向リンク
// =========================
struct Link {
    Frame* other;     // 相手フレーム
    bool is_parent;   // 自分が親か？
    Pose offset;      // 自分→相手 の変換
};

// =========================
// Frame: 座標系そのもの
// =========================
struct Frame {
    std::string name;
    std::vector<Link> links; // 双方向リンクの履歴

    Frame(const std::string& n) : name(n) {}

    // -------------------------
    // frameA(x,y,th)
    // → frameA 基準の Pose を作る
    // -------------------------
    Pose operator()(double x, double y, double th) const {
        return Pose{x,y,th,this};
    }

    // -------------------------
    // frameB( frameA_pose )
    // → frameA での座標を frameB で表現し直す
    // -------------------------
    Pose operator()(const Pose& src) const {
        return transform_between(src, *this);
    }

    // -------------------------
    // frameB.set_base( frameA(x,y,th) )
    // → frameA → frameB のリンク登録（双方向）
    // -------------------------
    void set_base(const Pose& pA) {
        Frame* A = const_cast<Frame*>(pA.frame);
        Frame* B = this;

        // child2parent = pA
        Link child2parent;
        child2parent.other = A;
        child2parent.is_parent = false;
        child2parent.offset = pA;

        // parent2child = 逆変換
        double c = std::cos(pA.th);
        double s = std::sin(pA.th);

        double rx = -c*pA.x - s*pA.y;
        double ry =  s*pA.x - c*pA.y;
        double rth = -pA.th;

        Link parent2child;
        parent2child.other = B;
        parent2child.is_parent = true;
        parent2child.offset = Pose{rx, ry, rth, B};

        // 双方向に登録
        B->links.push_back(child2parent);
        A->links.push_back(parent2child);
    }

    // =========================
    // 座標変換の内部処理
    // =========================

    // src.frame → dst.frame の変換
    static Pose transform_between(const Pose& src, const Frame& dst) {
        // BFS でフレーム間の経路を探す
        struct Node { const Frame* f; Pose pose; };
        std::vector<Node> q = { {src.frame, src} };
        std::vector<const Frame*> visited;

        while (!q.empty()) {
            Node cur = q.back();
            q.pop_back();

            if (cur.f == &dst)
                return Pose{cur.pose.x, cur.pose.y, cur.pose.th, &dst};

            visited.push_back(cur.f);

            for (const Link& lk : cur.f->links) {
                if (std::find(visited.begin(), visited.end(), lk.other) != visited.end())
                    continue;

                // cur.pose を lk.offset で変換
                double c = std::cos(lk.offset.th);
                double s = std::sin(lk.offset.th);

                double nx = lk.offset.x + c*cur.pose.x - s*cur.pose.y;
                double ny = lk.offset.y + s*cur.pose.x + c*cur.pose.y;
                double nth = lk.offset.th + cur.pose.th;

                q.push_back({lk.other, Pose{nx,ny,nth,lk.other}});
            }
        }

        throw std::runtime_error("Frame not reachable");
    }
};