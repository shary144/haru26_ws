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
//* Pose child{x,y,th,frame}
//これらの履歴があれば変換の必要条件は満たす
//さらにこれを上書きできるようにしたい
//よって上述の必要条件をLinkクラスにもたせて、上書きするようにする
//また、3つ巴以上の相互リンクではBFSの段階でつじつまが合わないなら
//矛盾するインスタンスは削除される。

//auto& pos = frameB(frameA(x,y,th)) ...座標系A上のポーズ(x,y,th)を座標系Bで表現
//引数で紐づけられる順番に処理self_frame(other_frame)になるときother_frameの値をもとにself_frameが処理
//self_frameがother_frameを履歴から探して上の変換処理をする
//親子関係をたどってたどりつけないときはエラー


#include <cmath>
#include <vector>
#include <algorithm>
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
    Pose offset;      // 常にchild座標系基準の座標で保存してればよし
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
        //先にframeAとframeB間で登録されたリンクがないか
        Frame* A = const_cast<Frame*>(pA.frame);
        Frame* B = this;
        auto itA = std::find_if(A->links.begin(),A->links.begin(),
            [A](const Link& link) { return link.other == B; });
        if (itA != A->links.end()){
            auto itB = std::find_if(B->links.begin(),B->links.begin(),
            [B](const Link& link) { return link.other == A; });
            itA->offset = pA;
            itB->offset = pA;
        }

        // child2parent = pA
        Link linkAB;
        Link linkBA;
        linkAB.other = B;
        linkBA.other = A;
        linkAB.offset = linkBA.offset = pA;

        // 双方向に登録
        A->links.push_back(linkAB);
        B->links.push_back(linkBA);
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
            //現在のフレームがdstの座標系に等しければ現在の座標系を返す
            if (cur.f == &dst)
                return Pose{cur.pose.x, cur.pose.y, cur.pose.th, &dst};

            visited.push_back(cur.f);

            for (const Link& lk : cur.f->links) {
                //visitedから見つかったらスタック処理をスキップ
                if (std::find(visited.begin(), visited.end(), lk.other) != visited.end())
                    continue;

                // cur.pose を lk.offset で変換
                double c = std::cos(lk.offset.th);
                double s = std::sin(lk.offset.th);

                double nx = lk.offset.x + c*cur.pose.x - s*cur.pose.y;
                double ny = lk.offset.y + s*cur.pose.x + c*cur.pose.y;
                double nth = lk.offset.th + cur.pose.th;
                // child側からかparent側からかで異なる
                double nth = 

                
                q.push_back({lk.other, Pose{nx,ny,nth,lk.other}});
            }
        }

        throw std::runtime_error("Frame not reachable");
    }
};