#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <memory>
#include <numbers>
struct Frame; // 前方宣言

// =========================
// Pose: 座標値 + どのフレーム基準か
// =========================
struct Pose {
    double x, y, th;
    const Frame* frame;
};

// =========================
// Link: フレーム間のリンク
// offset は「parent座標系で見たchildのPose」を共有ポインタで持つ
// =========================
struct Link {
    Frame* other;                      // 相手フレーム（parentかchildのどちらか）
    std::shared_ptr<Pose> offset;      // parent座標系で見たchildのPose（共有）
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
        return Pose{x, y, th, this};
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
    // → 「Parent = frameA 座標系で見た Child = frameB の Pose」を登録
    //    offset は常に Parent 座標系で見た Child の Pose を共有で持つ
    // -------------------------
    void set_base(const Pose& pA) {
        Frame* Parent = const_cast<Frame*>(pA.frame);
        Frame* Child  = this;

        // 自分自身を親にするのは不正
        if (Parent == Child) {
            throw std::runtime_error("set_base: parent and child are the same frame");
        }

        // 既存リンク探索
        auto find_link = [](Frame* self, Frame* other) -> Link* {
            for (auto& lk : self->links) {
                if (lk.other == other) return &lk;
            }
            return nullptr;
        };

        Link* pc = find_link(Parent, Child); // Parent → Child
        Link* cp = find_link(Child, Parent); // Child → Parent

        if (pc || cp) {
            // 片方だけあるのはデータ破損
            if (!(pc && cp)) {
                throw std::runtime_error("Broken link: one-way link exists between frames");
            }
            // 共有Poseを上書き
            std::shared_ptr<Pose> shared = pc->offset;
            shared->x = pA.x;
            shared->y = pA.y;
            shared->th = pA.th;
            shared->frame = Parent;
            return;
        }

        // 新規リンク作成：Parent基準で見たChildのPoseを共有
        auto shared = std::make_shared<Pose>(Pose{pA.x, pA.y, pA.th, Parent});

        Link linkPC;
        linkPC.other  = Child;
        linkPC.offset = shared;

        Link linkCP;
        linkCP.other  = Parent;
        linkCP.offset = shared;

        Parent->links.push_back(linkPC);
        Child->links.push_back(linkCP);
    }

    // =========================
    // 座標変換の内部処理
    // =========================

    // src.frame → dst.frame の変換
    static Pose transform_between(const Pose& src, const Frame& dstf) {
        // BFS でフレーム間の経路を探す
        std::vector<Pose> q = {src};
        std::vector<const Frame*> visited;

        while (!q.empty()) {
            Pose cur = q.back();
            q.pop_back();

            // 目的のフレームに到達したら返す
            if (cur.frame == &dstf)
                return Pose{cur.x, cur.y, cur.th, &dstf};

            visited.push_back(cur.frame);

            for (const Link& lk : cur.frame->links) {
                //今までに訪れたことがあれば処理をスキップ
                if (std::find(visited.begin(), visited.end(), lk.other) != visited.end())
                    continue;

                std::shared_ptr<Pose> off = lk.offset;   // Parent基準で見たChildのPose
                const Frame* Parent = off->frame;
                Frame* Other        = lk.other;

                double nx, ny, nth;
                

                if (cur.frame == Parent) {
                    // Parent → Child 変換
                    // Child_p = R(-th) * (Parent_p - offset)
                    double dx = cur.x - off->x;
                    double dy = cur.y - off->y;
                    double th = off->th;
                    double c  = std::cos(th);
                    double s  = std::sin(th);

                    nx  =  c * dx + s * dy;
                    ny  = -s * dx + c * dy;
                    nth = cur.th - th;

                    q.push_back(Pose{nx, ny, nth, Other});
                }
                else if (Other == Parent){
                    // Child → Parent 変換
                    // Parent_p = offset + R(th) * Child_p
                    double th = off->th;
                    double c  = std::cos(th);
                    double s  = std::sin(th);

                    nx  = off->x + c * cur.x - s * cur.y;
                    ny  = off->y + s * cur.x + c * cur.y;
                    nth = off->th + cur.th;

                    q.push_back(Pose{nx, ny, nth, Parent});
                }
                else {
                    // offset.frame と other のどちらとも一致しないのは構造破損
                    throw std::runtime_error("Broken link structure");
                }
            }
        }

        throw std::runtime_error("Frame not reachable");
    }
};